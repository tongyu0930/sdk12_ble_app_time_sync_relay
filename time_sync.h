#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

/**@brief Data handler type. */
typedef void (*ts_evt_handler_t)(uint32_t time);

typedef struct
{
    uint8_t          rf_chn;          /** RF Channel [0-80] */
    uint8_t          rf_addr[5];      /** 5-byte RF address */
    uint8_t          ppi_chns[4];     /** PPI channels */
    uint8_t          ppi_chhg;        /** PPI Channel Group */
    NRF_TIMER_Type * high_freq_timer[2]; /** 16 MHz timer (e.g. NRF_TIMER2) */
    NRF_RTC_Type   * rtc;
    NRF_EGU_Type   * egu;			//Event Generator Unit
    IRQn_Type        egu_irq_type;
} ts_params_t;

/**@brief SoftDevice system event handler. Must be called when a system event occurs */
void ts_on_sys_evt(uint32_t sys_evt);

void ppi_configure(void);


/**@brief Enable time sync library. This will enable reception of sync packets.
 *
 * @retval NRF_SUCCESS if successful
 */
uint32_t ts_enable(void);

/**@brief Disable time sync library. 
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_disable(void);

/**@brief Start sync packet transmission (become timing master).
 *
 * @note @ref ts_enable() must be called prior to calling this function
 * @note Expect some jitter depending on BLE activity.
 *
 * @param[in] sync_freq_hz Frequency of transmitted sync packets. 
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_tx_start(void);

/**@brief Stop sync packet transmission (become timing slave again).
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_tx_stop(void);


#endif /* __TIME_SYNC_H__ */
