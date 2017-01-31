
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"




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
					NRF_LOG_INFO("rssi = %d\r\n", p_adv_report->rssi);
					//NRF_LOG_INFO("dlen = %d\r\n", p_adv_report->dlen); // 这个就是p_data（安卓手机上raw data）的length
			}
			index += field_length + 1;
	    }
}
