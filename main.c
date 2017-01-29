/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "app_error.h"
#include "time_sync.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"


#define CENTRAL_LINK_COUNT       		0  /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    		0  /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0 /**< 不懂Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_COMPANY_IDENTIFIER          0x0059  /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//不知道设置为0004和4000有什么区别，led看起来都闪的一样快。
//#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_INTERVAL           0x0400   //Scan interval or window is between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s).
//#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0400   //The scanWindow shall be less than or equal to the scanInterval.Scan window between 0x0004 and 0x4000
#define SCAN_ACTIVE             0                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000


#define APP_TIMER_PRESCALER             0   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4   /**< Size of timer operation queues. */
#define SYNC_BEACON_COUNT_PRINTOUT_INTERVAL APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

static ble_gap_adv_params_t m_adv_params;   /**< Parameters to be passed to the stack when starting advertising. */

//static bool                             m_advertising_running       = false;
static bool 							m_send_sync_pkt 			= false;  // time_sync.c 里也有这个变量，是不是不是一回事？
static bool 							ts_is_enabled 				= false;

volatile bool want_shift 	= false;

APP_TIMER_DEF(m_sync_count_timer_id);

/**
 * @brief Parameters used when scanning.   active scan means asking for scan response packet
 */
const ble_gap_scan_params_t m_scan_params =
  {
    .active      = SCAN_ACTIVE,
    .use_whitelist   = SCAN_SELECTIVE,
    .adv_dir_report = 0,
    .interval    = SCAN_INTERVAL,
    .window      = SCAN_WINDOW,
    .timeout     = SCAN_TIMEOUT
  };

/*
void HardFault_Handler(void)  //作者添加的
{
    uint32_t *sp = (uint32_t *) __get_MSP();
    uint32_t ia = sp[24/4];
    NRF_LOG_INFO("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    while(1)
        ;
}*/


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze how your product is supposed to react in case of Assert.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)  //作者添加的
{
    // static error variables - in order to prevent removal by optimizers
    static volatile struct
    {
        uint32_t        fault_id;
        uint32_t        pc;
        uint32_t        error_info;
        assert_info_t * p_assert_info;
        error_info_t  * p_error_info;
        ret_code_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0};

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;
    UNUSED_VARIABLE(loop);

    m_error_data.fault_id   = id;
    m_error_data.pc         = pc;
    m_error_data.error_info = info;

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            m_error_data.p_assert_info = (assert_info_t *)info;
            m_error_data.line_num      = m_error_data.p_assert_info->line_num;
            m_error_data.p_file_name   = m_error_data.p_assert_info->p_file_name;
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            m_error_data.p_error_info = (error_info_t *)info;
            m_error_data.err_code     = m_error_data.p_error_info->err_code;
            m_error_data.line_num     = m_error_data.p_error_info->line_num;
            m_error_data.p_file_name  = m_error_data.p_error_info->p_file_name;
            break;
    }

    UNUSED_VARIABLE(m_error_data);

    NRF_LOG_INFO("ASSERT\r\n\tError: 0x%08x\r\n\tLine: %d\r\n\tFile: %s\r\n", m_error_data.err_code, m_error_data.line_num, m_error_data.p_file_name);

    // If printing is disrupted, remove the irq calls, or set the loop variable to 0 in the debugger.
    __disable_irq();

    while(loop);

    __enable_irq();
} */


//这是APP TIMER's handler
static void sync_beacon_count_printout_handler(void * p_context)
{
    extern volatile uint32_t m_test_count;	// 发出去多少个 packet 									// 这三个count有何意义？
    extern volatile uint32_t m_rcv_count;	// 收到多少个  packet
    extern volatile uint32_t m_blocked_cancelled_count;

    if (m_test_count != 0)
    {
        NRF_LOG_INFO("TX(m_test_count): %d\r\n", m_test_count);
        m_test_count = 0;
    }
    if (m_rcv_count != 0)
    {
        NRF_LOG_INFO("RX(m_rcv_count): %d\r\n", m_rcv_count);
        m_rcv_count = 0;
    }
    if (m_blocked_cancelled_count != 0)
    {
        NRF_LOG_INFO("Blocked(m_blocked_cancelled_count): %d\r\n", m_blocked_cancelled_count);
        m_blocked_cancelled_count = 0;
    }
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack. Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    uint8_t data[] = "xxxxx"; // Our data to adverise。 scanner上显示的0x串中，最后是00，表示结束。

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = data;
    manuf_specific_data.data.size   = sizeof(data);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(&m_adv_params); // 括号里面的是个全局变量吗？
    APP_ERROR_CHECK(err_code);
}


//自己加的
void relay_adv_data(ble_evt_t * p_ble_evt)
{
	uint32_t index = 0;

	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
			{

					NRF_LOG_INFO("GOTCHYA\r\n");

			}

			index += field_length + 1;
	    }

	//sd_ble_gap_adv_data_set(p_data, sizeof(p_data), NULL, 0);

}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    relay_adv_data(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)														//这个function到底何时被call？
{
    ts_on_sys_evt(sys_evt); // this function is in "time_sync.c"
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL); // Initialize the SoftDevice handler module.

    ble_enable_params_t ble_enable_params;

    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);//Check the ram settings against the used number of links

    err_code = softdevice_enable(&ble_enable_params); // Enable BLE stack.
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch); // Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch); // Register with the SoftDevice handler module for System (SOC) events.
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


void GPIOTE_IRQHandler(void)
{
    uint32_t err_code;

    if (NRF_GPIOTE->EVENTS_IN[1] != 0) // button1 实现timesync广播的开启和关闭
    {
        nrf_delay_us(2000);

        NRF_GPIOTE->EVENTS_IN[1] = 0; // 这句话是为了防止按键一直被按着，如果没有这句话，handler就会一直被call

        if (m_send_sync_pkt)
        {
            m_send_sync_pkt  = false;

            err_code = ts_tx_stop();
            APP_ERROR_CHECK(err_code);
            NRF_GPIO->OUTSET = BSP_LED_1_MASK; // if ts_tx_stop, LED2 will be off

            NRF_LOG_INFO("Stop TX!\r\n");
        }
        else
        {
            m_send_sync_pkt  = true;

            err_code = ts_tx_start(100);
            APP_ERROR_CHECK(err_code);
            NRF_GPIO->OUTCLR = BSP_LED_1_MASK; // if ts_tx_start, LED2 will be off

            NRF_LOG_INFO("Start TX!\r\n");
        }
    }

    if (NRF_GPIOTE->EVENTS_IN[2] != 0) // button2 关闭 timeslot
    {
    	nrf_delay_us(2000);

        NRF_GPIOTE->EVENTS_IN[2] = 0;

        if(ts_is_enabled)
		{
			err_code = ts_disable();			// 我发现即使没关闭timeslot，再次开启timeslot也会出现fatal error，但是timeslot tutorial就不会。
			APP_ERROR_CHECK(err_code);

			NRF_LOG_INFO("ts disabled\r\n");

			ts_is_enabled = false;
		 }

    }

    if (NRF_GPIOTE->EVENTS_IN[3] != 0)		// shift
    {
    	nrf_delay_us(2000);

        NRF_GPIOTE->EVENTS_IN[3] = 0;

        if(!want_shift)
        {
        	want_shift = true;
			NRF_LOG_INFO("shift\r\n");
         }
    }

    if (NRF_GPIOTE->EVENTS_IN[4] != 0)		// for test, enable ts
        {
        	nrf_delay_us(2000);

            NRF_GPIOTE->EVENTS_IN[4] = 0;

            if(!ts_is_enabled)
            {
				err_code = ts_enable();
				APP_ERROR_CHECK(err_code);

				NRF_LOG_INFO("ts disabled\r\n");

				ts_is_enabled = true;
             }
        }
}


static void gpio_configure(void)
{
	NRF_GPIO->DIRSET = LEDS_MASK; // set register
	NRF_GPIO->OUTSET = LEDS_MASK; // clear register

	NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_2] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_3] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_4] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	nrf_delay_us(5000);																			// Do I have to delay?

	NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_High    << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
							(19                            << GPIOTE_CONFIG_PSEL_Pos);			// 19 is the pin number for testing

	NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_1                      << GPIOTE_CONFIG_PSEL_Pos);

	NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_2                      << GPIOTE_CONFIG_PSEL_Pos);

	// 自己加的
	NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_3                      << GPIOTE_CONFIG_PSEL_Pos);

	NRF_GPIOTE->CONFIG[4] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_4                      << GPIOTE_CONFIG_PSEL_Pos);

	// Interrupt
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk | GPIOTE_INTENSET_IN2_Msk | GPIOTE_INTENSET_IN3_Msk | GPIOTE_INTENSET_IN4_Msk;
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOWEST);
	NVIC_EnableIRQ(GPIOTE_IRQn); 											 					//declaration值得一看！！！有关IRQ

}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("############# System Started ###############\r\n");

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); //(0,4,false)
    /*
     *  PRESCALER: will be written to the RTC1 PRESCALER register. This determines the time resolution of the timer,
     *  and thus the amount of time it can count before it wrap around. On the nRF52 the RTC is a 24-bit counter with
     *  a 12 bit prescaler that run on the 32.768 LFCLK. The counter increment frequency (tick rate) fRTC [kHz] = 32.768/(PRESCALER+1).
     *  For example, a prescaler value of 0 means that the tick rate or time resolution is 32.768 kHz * 1/(0+1) = 32.768 kHz
     *  and the timer will wrap around every (2^24) * 1/32.768 kHz = 512 s.
     *
     *  OP_QUEUES_SIZE: determines the maximum number of events that can be queued. Let's say you are calling the API function several
     *  times in a row to start a single shot timer, this determines how many times you can have queued before the queue gets full.
     *
     *  SCHEDULER_FUNC: should be set to false when scheduler is not used
     */
    err_code = app_timer_create(&m_sync_count_timer_id, APP_TIMER_MODE_REPEATED, sync_beacon_count_printout_handler);
    APP_ERROR_CHECK(err_code);

    ble_stack_init();

    advertising_init();

    err_code = app_timer_start(m_sync_count_timer_id, SYNC_BEACON_COUNT_PRINTOUT_INTERVAL, NULL); // 每1000ms就触发一次handler
    APP_ERROR_CHECK(err_code);

    gpio_configure(); // 注意gpio和timesync是相对独立的，同步时钟本质上不需要gpio


    //ppi_configure();

// 设置并开启 timer2 这个就是被同步的timer
	NRF_TIMER2->TASKS_STOP  = 1;
	NRF_TIMER2->TASKS_CLEAR = 1;
	NRF_TIMER2->PRESCALER   = 8; // 原值为：SYNC_TIMER_PRESCALER // frequency = 16000000/(2^8) = 62500 hz
	NRF_TIMER2->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;		//16 bit timer
	NRF_TIMER2->CC[0]       = (0xFFFF);
	//NRF_TIMER2->CC[3]       = (0x7FFF); // Only used for debugging purposes such as pin toggling
	NRF_TIMER2->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk | TIMER_SHORTS_COMPARE3_CLEAR_Msk; // 让event_compare register达到cc的值就清零
	NRF_TIMER2->TASKS_START = 1;

	NRF_TIMER2->EVENTS_COMPARE[3] = 0; // 自己加的 event_compare register 的初始值

	NRF_LOG_INFO("timer2 started\r\n");


    //err_code = ts_enable();
    //APP_ERROR_CHECK(err_code);


    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
