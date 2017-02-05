/**
 * https://devzone.nordicsemi.com/blogs/978/wireless-timer-synchronization-among-nrf5-devices/
 */

#include "time_sync.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "ble_gap.h"


#define TX_CHAIN_DELAY_PRESCALER_0 						0 //原值＝704		这个数值需要调试
#define SYNC_TIMER_PRESCALER 							0
#define SYNC_RTC_PRESCALER 								0
#if SYNC_TIMER_PRESCALER 								== 0
#define TX_CHAIN_DELAY 									TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif
#define TS_LEN_US                            			(1000UL)
#define TX_LEN_EXTENSION_US                  			(1000UL)
#define TS_SAFETY_MARGIN_US                  			(500UL)   /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  			(700UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */
#define MAIN_DEBUG                           			0x12345678UL
#define TIMER_MAX_VAL (0xFFFF)	 //16bit
#define RTC_MAX_VAL   (0xFFFFFF) //24bit
#define TIMESLOT_EXTEND_LENGTH 1000	//原值 200
#define TIMESLOT_TIMER_INTERRUPT_END_MARGIN				100	// 原值50 我觉得这个数值的大小取决于你的graceful shotdown要关掉什么


static nrf_radio_signal_callback_return_param_t 		signal_callback_return_param;
static nrf_radio_request_t  							m_timeslot_request;
static uint32_t             							m_slot_length;
static uint32_t      									total_timeslot_length 					= 0;
static uint32_t											normal_distance 						= 0;
static uint32_t     									m_ppi_chen_mask 						= 0;
static uint8_t											timer_offset_tolerance					= 2;
static volatile bool 									m_send_sync_pkt 						= false;
static volatile bool 									m_timer_update_in_progress 				= false;
static volatile bool 									timeslot_session_opened 				= false;
static volatile uint32_t 								extend_success_count;
static volatile uint32_t 								fail_count;
volatile uint32_t    									m_blocked_cancelled_count 				= 0;							// 这三个count有何意义？
volatile uint32_t 										m_test_count 							= 0;
volatile uint32_t 					  					m_rcv_count 							= 0;
volatile bool 											want_scan 								= false;
volatile bool 											alreadyinsync 							= false;
extern volatile bool 									first_time;
extern volatile bool 									want_shift; 															// 不要赋值
extern const ble_gap_scan_params_t 						m_scan_params;


static volatile struct
{
    int32_t timer_val;
    int32_t rtc_val;
} m_sync_pkt;

static volatile enum
{
    RADIO_STATE_IDLE, /* Default state */
    RADIO_STATE_RX,   /* Waiting for packets */
    RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;


void advertising_start(void);
static void sync_timer_offset_compensate(void);
static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);


uint32_t request_next_event_earliest(void)
{
    m_slot_length                                  = TIMESLOT_EXTEND_LENGTH;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_timeslot_request);
}


void configure_next_event_earliest(void)
{
    m_slot_length                                  = TIMESLOT_EXTEND_LENGTH;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000; // 1秒
    //nrf_gpio_pin_toggle(18);
}

void configure_next_event_normal(void)
{
    m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us  = normal_distance;
    m_timeslot_request.params.normal.length_us    = 1000;
}



void RADIO_IRQHandler(void) // 收到了sync packet后经过 radio callback，然后来到这里
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;
        
        // 如果是RX mode， Packet received
        if (m_radio_state==RADIO_STATE_RX&&(NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk)==(RADIO_CRCSTATUS_CRCSTATUS_CRCOk<<RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_timer_offset_compensate();
            ++m_rcv_count;
        }
    }
}


/**@brief   Function for handling timeslot events.
 *
 * "radio_callback"
 * will be called when the radio timeslot starts. From this point the NRF_RADIO and NRF_TIMER0 peripherals can be freely accessed by the application.
 * is called whenever the NRF_TIMER0 interrupt occurs.
 * is called whenever the NRF_RADIO interrupt occurs.
 * will be called at ARM interrupt priority level 0. This implies that none of the sd_* API calls can be used from p_radio_signal_callback().
 */
static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type) // This callback runs at lower-stack priority(the highest priority possible).
{
    switch (signal_type) {

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:														// 每个新的timeslot开始才会进入这个case，因为timeslot一旦结束，对设备的控制权就没了，所以要重新设置这些东西
    	NRF_LOG_INFO("CASE_START\r\n");
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
		NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);			// 4000多秒才会flowout
		NRF_TIMER0->PRESCALER = (4 << TIMER_PRESCALER_PRESCALER_Pos);								// TIMER0 is pre-configured for 1Mhz
		NRF_TIMER0->CC[0] = m_slot_length - TIMESLOT_TIMER_INTERRUPT_END_MARGIN;
		NVIC_EnableIRQ(TIMER0_IRQn);
        NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);		// turn on radio
        total_timeslot_length = m_slot_length;
        timeslot_begin_handler();																	// TXmode时，发送packet的频率就是进入这个case的频率
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
    	NRF_TIMER0->TASKS_STOP          = 1;
    	NRF_TIMER0->EVENTS_COMPARE[0] 	= 0;
    	if(m_send_sync_pkt)
    	{
    		timeslot_end_handler();
    		normal_distance = total_timeslot_length + 50000;										// 50000是和下一次timeslot的距离
			configure_next_event_normal();
			signal_callback_return_param.params.request.p_next = &m_timeslot_request;
			signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    	}else
    	{
    		if(total_timeslot_length >= 6000000)													// 可以把600000设置为变量，方便调解
    		{
    			timeslot_end_handler(); 															// 我发现这个有没有，之后都可以正常广播扫描
    			//NRF_LOG_INFO("Timeslot end %d microseconds\r\n", NRF_TIMER0->CC[0]); 				// 这个数值小的原因是剪去了 TIMESLOT_TIMER_INTERRUPT_END_MARGIN
    			NRF_LOG_INFO("waited too long, timeout!");
    			signal_callback_return_param.params.request.p_next = NULL;
    			signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
    			//configure_next_event_earliest();													// 想不停止就用这三句代替上面两句
				//signal_callback_return_param.params.request.p_next = &m_timeslot_request;
				//signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    		}else
    		{
				signal_callback_return_param.params.request.p_next = NULL;
				signal_callback_return_param.params.extend.length_us = TIMESLOT_EXTEND_LENGTH;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
    		}
    	}
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
    	RADIO_IRQHandler();
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
    	fail_count++;
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
		break;
    
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
    	NRF_TIMER0->TASKS_STOP          = 1;
    	NRF_TIMER0->EVENTS_COMPARE[0] 	= 0;														// 这个是事件，数值只有0，1，你不让他＝0，就会一直有interrupt
    	extend_success_count++;
    	total_timeslot_length += TIMESLOT_EXTEND_LENGTH;
		//NRF_TIMER0->CC[0] += TIMESLOT_EXTEND_LENGTH - TIMESLOT_TIMER_INTERRUPT_END_MARGIN; 			// WHY? 看来自己对compare的理解不对
		NRF_TIMER0->TASKS_CLEAR			= 1;
		NRF_TIMER0->CC[0] 				= TIMESLOT_EXTEND_LENGTH - TIMESLOT_TIMER_INTERRUPT_END_MARGIN;
		NRF_TIMER0->TASKS_START         = 1;

		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        break;
    
    default:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
        break;
    };


    return (&signal_callback_return_param);   														// Fall-through return: return with no action request
}


static void update_radio_parameters()
{   
	uint8_t        	rf_address[5] 			= {0xDE, 0xAD, 0xBE, 0xEF, 0x19};									// ?
	uint8_t			rf_channel         	 	= 125;																// ?

    // TX power
    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_0dBm   << RADIO_TXPOWER_TXPOWER_Pos;
    
    // RF bitrate
    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit    << RADIO_MODE_MODE_Pos;
    
    // Fast startup mode
    NRF_RADIO->MODECNF0  = RADIO_MODECNF0_RU_Fast 		<< RADIO_MODECNF0_RU_Pos;
    
    // CRC configuration
    NRF_RADIO->CRCCNF    = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos; 
    NRF_RADIO->CRCINIT   = 0xFFFFFFUL;    																		// Initial value
    NRF_RADIO->CRCPOLY   = 0x11021UL;     																		// CRC poly: x^16+x^12^x^5+1
    
    // Packet format 
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled        << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big              << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                   << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(m_sync_pkt)                  << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PACKETPTR = (uint32_t)&m_sync_pkt;
    
    // Radio address config
    NRF_RADIO->PREFIX0 		= rf_address[0];
    NRF_RADIO->BASE0  		= (rf_address[1] << 24 | rf_address[2] << 16 | rf_address[3] << 8 | rf_address[4]);
    
    NRF_RADIO->TXADDRESS    = 0;
    NRF_RADIO->RXADDRESSES  = (1 << 0);
    
    NRF_RADIO->FREQUENCY 	= rf_channel;
    NRF_RADIO->TXPOWER   	= RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;
    
    NRF_RADIO->EVENTS_END 	= 0;
    
    NRF_RADIO->INTENCLR 	= 0xFFFFFFFF;
    NRF_RADIO->INTENSET 	= RADIO_INTENSET_END_Msk;  															//radio活动结束时interrupt
    
    NVIC_EnableIRQ(RADIO_IRQn);
}


/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
void timeslot_end_handler(void)
{
    NRF_RADIO->TASKS_DISABLE = 1;																		// 关闭radio
    NRF_RADIO->INTENCLR      = 0xFFFFFFFF;
    NRF_PPI->CHENCLR = (1 << 3); 																		// disable 在begin handler里设置的ppi		channel 3
    m_radio_state            = RADIO_STATE_IDLE;
    //NRF_LOG_INFO("endhandler\r\n");
}


/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to initiate UESB RX/TX
  *
  *        This is the first timing-critical part of the code, which ensures that the
  *        timing master captures the free running timer value at a consistent time delta
  *        from when the packet is actually transmitted
  */
void timeslot_begin_handler(void)
{
    if (!m_send_sync_pkt) // 只要是 程序出于RX mode（没按button）就会 只 进入这个if
    {
    	// 没有在发送packet 并且RADIO也不是在RX状态
        if (m_radio_state != RADIO_STATE_RX || NRF_RADIO->STATE != (RADIO_STATE_STATE_Rx << RADIO_STATE_STATE_Pos)) //m_radio_state = RADIO_STATE_IDLE 才可进入
        {
            update_radio_parameters(); // 这里面设置了radio interrupt 但没开启radio, 因为前面已经开启了radio，所以现在出于开启状态，但不是在RX mode
            
            NRF_RADIO->SHORTS     	 = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
            NRF_RADIO->TASKS_RXEN    = 1;														//Enable RADIO in RX mode
            
            NRF_PPI->CH[3].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;					  		// trigger: Address sent or received  //What exactly is it?
            NRF_PPI->CH[3].TEP = (uint32_t) &NRF_TIMER2->TASKS_CAPTURE[1]; 						// capture timer value to timer2's CC[1] register
            NRF_PPI->CHENSET         = (1 << 3);												//就是说收到广播后获取本地timer2的时间，这个时间就是compensate里的本地时间
            
            m_radio_state = RADIO_STATE_RX;
        }
        return;
    }
    
    if (m_radio_state == RADIO_STATE_RX)														// 这个if是为了干什么？
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;  															// Disable RADIO
        while (NRF_RADIO->EVENTS_DISABLED == 0)
        {
            __NOP();																			// 到了这句话系统还能不能继续走下去？
        }
    }

    // 那也就是说只有当 m_radio_state != RX || IDLE  时才会执行下面的语句
    update_radio_parameters();
    
    // here to re-configure ppi for TX mode, "ppi_configure" is configured for RX mode.
    // NRF_TIMER3 is used to trigger "get nrf_timer0 value" and "radio transmission"
    // Use PPI to create fixed offset between timer capture and packet transmission
    
    NRF_PPI->CH[1].EEP = (uint32_t) &NRF_TIMER3->EVENTS_COMPARE[0];								// Compare event #0: Capture timer value for free running timer
    NRF_PPI->CH[1].TEP = (uint32_t) &NRF_TIMER2->TASKS_CAPTURE[1]; 								// capture the timer value and write to cc1 register
    NRF_PPI->CHENSET         = (1 << 1);											  			// enable channel1

    NRF_PPI->CH[2].EEP = (uint32_t) &NRF_TIMER3->EVENTS_COMPARE[1]; 							// Compare event #1: Trigger radio transmission
    NRF_PPI->CH[2].TEP = (uint32_t) &NRF_RADIO->TASKS_START;
    NRF_PPI->CHENSET          = (1 << 2);									        			//enable channel2
    
    NRF_TIMER3->PRESCALER   = 4; 																// 1 us resolution
    NRF_TIMER3->MODE        = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->SHORTS      = TIMER_SHORTS_COMPARE1_STOP_Msk | TIMER_SHORTS_COMPARE1_CLEAR_Msk;	// compare1 达到时会自动清零停止
    NRF_TIMER3->TASKS_STOP  = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->CC[0]       = 45; 																// Matches 40 us radio rampup time
    NRF_TIMER3->CC[1]       = 60; 																// Margin for timer readout
    
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->EVENTS_COMPARE[1] = 0;
    
    NRF_RADIO->SHORTS                        = RADIO_SHORTS_END_DISABLE_Msk;
    NRF_RADIO->TASKS_TXEN                    = 1;												//Enable RADIO in TX mode
    NRF_TIMER3->TASKS_START = 1;																//start nrf_timer3
    
    while (NRF_TIMER3->EVENTS_COMPARE[0] == 0)													//上一句刚开启timer3，所以这地方括号里就为true？
    {
        __NOP();																				// Wait for timer to trigger
    }																							// 一旦timer3 里的counter到45，就出来while了
    
    m_radio_state                = RADIO_STATE_TX;
    m_sync_pkt.timer_val         = NRF_TIMER2->CC[1];											// 这时CC[1]里的值就是上面用ppi capture到的？
    m_sync_pkt.rtc_val           = NRF_RTC1->COUNTER;											// 更新完要发送的packet的内容后就等待ppi激发radio发出去
    
    ++m_test_count;
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.  在main.c里call的		这个应该是系统有反应了就自动call
 */
void ts_on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS: //注意这里没有break，如果走的是这个case，出去后继续再往下走，而不是退出函数。
        case NRF_EVT_FLASH_OPERATION_ERROR:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        	/*
			 * The requested timeslot could not be scheduled due to a collision with other activities.
			 * The application should request a new timeslot either at the earliest possible, or at the next normal position.
			 * 去这里看看
			 * SoftDevices > S132 SoftDevice > S132 SoftDevice Specification > Multiprotocol support > Radio Timeslot API usage scenarios
			 */
        case NRF_EVT_RADIO_CANCELED:
        	{// Blocked events are rescheduled with normal priority. They could also
            // be rescheduled with high priority if necessary.
        	NRF_LOG_INFO("NRF_EVT_RADIO_CANCELED\r\n");
            m_blocked_cancelled_count++;
            break;}

        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_LOG_INFO("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;

        case NRF_EVT_RADIO_SESSION_IDLE:
        	{/*
			 * The session has no remaining scheduled timeslots. If this event is triggered, the application ends the session.
			 * Note that it is also possible to request new timeslots here(more on that later).
			 */
			NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE\r\n");
			//NRF_GPIO->OUTCLR = (1 << 17);
			sd_radio_session_close();
			break;}

        case NRF_EVT_RADIO_SESSION_CLOSED: //The session is closed and all acquired resources are released
        	timeslot_session_opened = false;
            NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED\r\n");
			NRF_LOG_INFO(" extend_success_count            = %d\r\n", extend_success_count);
			NRF_LOG_INFO(" extend_fail_count               = %d\r\n", fail_count);
			NRF_LOG_INFO(" total timeslot length           = %d\r\n", total_timeslot_length);
            break;

        default:
            // No implementation needed.
            NRF_LOG_INFO("Event: 0x%08x\r\n", sys_evt);
            break;
    }
}


void SWI3_EGU3_IRQHandler(void)											// it is used to shut down thoses PPIs that is used for clear timer2's offset
{
	uint32_t err_code;

    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;
        (void) NRF_EGU3->EVENTS_TRIGGERED[0];
        
        NRF_PPI->CHENCLR = m_ppi_chen_mask;								// 关闭对时的那几个channel
        NRF_LOG_INFO("m_ppi_chen_mask be cleared\r\n");
        
        m_timer_update_in_progress = false;
    }


    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];

            //NRF_LOG_INFO("EGU[1]\r\n");
            if(!timeslot_session_opened)
            {
            if(want_shift)
            {
            	want_shift = false;
            	return;

            }else
            {
            	if(first_time)
            	{
            		err_code = sd_ble_gap_scan_start(&m_scan_params);
            		APP_ERROR_CHECK(err_code);
            		first_time 	= false;
            		want_scan 	= false;
            	}else
            	{
					if(want_scan)
					{
						err_code = sd_ble_gap_adv_stop();
						APP_ERROR_CHECK(err_code);

						err_code = sd_ble_gap_scan_start(&m_scan_params);
						APP_ERROR_CHECK(err_code);

						NRF_LOG_INFO("scanning\r\n");
						want_scan = false;

					}else
					{
						err_code = sd_ble_gap_scan_stop();
						APP_ERROR_CHECK(err_code);

						advertising_start();

						NRF_LOG_INFO("broadcasting\r\n");
						want_scan = true;
					}
            	}
            }
            }
        }
}

/*
 * The second timing-critical part is how the receiver updates its local free running
 * timer when a sync beacon packet is received. Note the magic value "TX_CHAIN_DELAY" in the following code:
 */
static inline void sync_timer_offset_compensate(void)
{
    uint32_t peer_timer;
    uint32_t local_timer;
    uint32_t timer_offset;
    
    if (m_timer_update_in_progress)
    {
        return;
    }

    peer_timer  = m_sync_pkt.timer_val;
    peer_timer += TX_CHAIN_DELAY; 													// magic value
    local_timer = NRF_TIMER2->CC[1];   												// cc1 was captured in "timeslot begin handler"
    
    if (local_timer > peer_timer)
    {
        timer_offset = TIMER_MAX_VAL - local_timer + peer_timer;    				// TIMER_MAX_VAL = 2^16 = 65536
    }
    else
    {
        timer_offset = peer_timer - local_timer;
    }
    
    //if (timer_offset == 0 ||timer_offset == TIMER_MAX_VAL) // this one is original
    if ((timer_offset < timer_offset_tolerance) || (timer_offset > (TIMER_MAX_VAL - timer_offset_tolerance)))
    {
    	alreadyinsync = true;
    	NRF_LOG_INFO("already in sync\r\n");
    }else
    {
    	alreadyinsync = false;  													// 去掉这句，already in sync 后就再也不会变为false了。
    }
    
	if(!alreadyinsync)
	{
    NRF_TIMER2->CC[2] = (TIMER_MAX_VAL - timer_offset); // Write offset to timer compare register
    
    m_timer_update_in_progress = true;
    
    NRF_PPI->CHENSET = m_ppi_chen_mask;												// Enable PPI channels 开启对时的那几个channel
    NRF_LOG_INFO("not in sync, m_ppi_chen_mask be set\r\n");
	}
}

void ppi_configure(void)	// in this function, only configuration, not enable, CHENSET is enable
{
    m_ppi_chen_mask = (1 << 1) | (1 << 2) | (1 << 4); 					// 为了对channel 1，2，4 一起操作

    NRF_PPI->CHENCLR      = PPI_CHENCLR_CH1_Msk; // (1 << 1);			// PPI channel 1: clear timer when offset value is reached
    NRF_PPI->CH[1].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2]; 	//把channel0的eep设为timer0的compare event（与cc[2] register compare）
    NRF_PPI->CH[1].TEP = (uint32_t) &NRF_TIMER2->TASKS_CLEAR;       	//tep是清零timer
    
    NRF_PPI->CHENCLR      = (1 << 2);									// PPI channel 2: disable PPI channel 0 such that the timer is only reset once.
    NRF_PPI->CH[2].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2]; 	// 把channel0的eep设为timer0的compare event
    NRF_PPI->CH[2].TEP = (uint32_t) &NRF_PPI->TASKS_CHG[0].DIS; 		// TEP is 'disable ppi group'
    
    NRF_PPI->CHENCLR      = (1 << 4);
    NRF_PPI->CH[4].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[2];
    NRF_PPI->CH[4].TEP = (uint32_t) &NRF_EGU3->TASKS_TRIGGER[0];

    NRF_PPI->TASKS_CHG[0].DIS = 1;
    NRF_PPI->CHG[0]           = (1 << 1) | (1 << 4); 					// the ppi channel group CHG[0] contain channel 1 and channel

   	NRF_PPI->CHENCLR      = (1 << 5);
	NRF_PPI->CH[5].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[0];
	NRF_PPI->CH[5].TEP = (uint32_t) &NRF_EGU3->TASKS_TRIGGER[1];
	NRF_PPI->CHENSET   = PPI_CHENSET_CH5_Msk; 							// enable

    NRF_EGU3->INTENSET 		= EGU_INTENSET_TRIGGERED0_Msk; 				// Enable interrupt for TRIGGERED[0] event
    //NRF_EGU3->INTENSET 		= EGU_INTENSET_TRIGGERED1_Msk;				// for test
    NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
    NVIC_SetPriority(SWI3_EGU3_IRQn, 7);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);

	NRF_PPI->CHENCLR      = (1 << 0);									// for test
	NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER2->EVENTS_COMPARE[0];		// 为什么用ppi时不用清除compare event？：EVENTS_COMPARE[0]＝0
	NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
	NRF_PPI->CHENSET   = PPI_CHENSET_CH0_Msk; // enable
}


uint32_t ts_enable(void) 																// 这个function返回error_code，所以类型是uint32_t
{
    uint32_t       err_code;
    uint32_t 	   hfclk_is_running;

	//memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address)); 						// copy // 要不然就要对数组的每个unit单独赋值。要5句话
    //memcpy(&ts_params, &ts_params, sizeof(ts_params_t)); 								// copy p_params to ts_params

	ppi_configure();

    if (timeslot_session_opened) { return NRF_ERROR_INVALID_STATE; } 					// 这地方是true还是false ? 我觉得是 false

    sd_clock_hfclk_is_running(&hfclk_is_running);
    NRF_LOG_INFO("hfclk_is_running = %d\r\n", hfclk_is_running);

    if(!hfclk_is_running)
    {
    	err_code = sd_clock_hfclk_request();											// external crystal oscillator
    	if (err_code != NRF_SUCCESS) { return err_code; }
    }

    err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT); 								// Constant latency mode // How about low power mode?
    if (err_code != NRF_SUCCESS) { return err_code; }

    err_code = sd_radio_session_open(radio_callback); 									// Opens a session for radio timeslot requests
    if (err_code != NRF_SUCCESS) { return err_code; }

    err_code = request_next_event_earliest(); 											// request the first timeslot (which must be of type earliest possible)
	if (err_code != NRF_SUCCESS)
	{
		(void)sd_radio_session_close();
		return err_code;
	}
    /*
     * Successful requests will result in nrf_radio_signal_callback_t(@ref NRF_RADIO_CALLBACK_SIGNAL_TYPE_START).
     * Unsuccessful requests will result in a @ref NRF_EVT_RADIO_BLOCKED event, see @ref NRF_SOC_EVTS.
     */

    timeslot_session_opened    	= true;
    
    m_blocked_cancelled_count  	= 0;
    m_send_sync_pkt            	= false;
    m_radio_state              	= RADIO_STATE_IDLE;
    
    NRF_LOG_INFO("ts is successfully enabled\r\n");

    return NRF_SUCCESS;
}

uint32_t ts_disable(void)
{
	//m_timer_update_in_progress = false;
	//alreadyinsync = false;
	/*
	 *  m_timeslot_distance 不用管
	 *  m_send_sync_pkt 不用管
	 *  m_radio_state
	 *  m_timeslot_session_open 不用管
	 *  m_total_timeslot_length 不用管
	 *  m_ppi_chen_mask 不用管
	 *  no need to stop timer0, because softdevice need the timer0 anyway.
	 */
	return NRF_SUCCESS;
}

uint32_t ts_tx_start(void)														// controlled by button
{
    m_send_sync_pkt = true;
    return NRF_SUCCESS;
}
    
uint32_t ts_tx_stop()					   														// controlled by button
{
    m_send_sync_pkt = false;
    return NRF_SUCCESS;
}
