
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"


void relay_adv_data(ble_evt_t * p_ble_evt)
{
	uint32_t index = 0;
	uint32_t err_code;

	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
			{

				uint8_t a=index+2;

				while(a <= (index+field_length))
				{
					uint8_t field_data = p_data[a];
					if(field_data == 0x78)
					{
						NRF_EGU3->INTENCLR 		= EGU_INTENCLR_TRIGGERED1_Msk;
						err_code = sd_ble_gap_scan_stop();
						APP_ERROR_CHECK(err_code);
						return; //不要重复stop scan！
					}
					a++;
				}
			/*
				uint8_t c = field_length +1;
				uint8_t pp_data[c];
				uint8_t a = index+2;
				uint8_t b = 2;

				pp_data[0] = field_length; // 你要relay的数据的长度和你扫描到的一样长
				pp_data[1] = field_type;

				while(a <= (index+field_length))
				{
					pp_data[b] = p_data[a];
					NRF_LOG_INFO("p_data = %x\r\n", p_data[a]);
					a++;
					b++;
				}

				sd_ble_gap_adv_data_set(pp_data, sizeof(pp_data), NULL, 0);

				NRF_LOG_INFO("rssi = %d\r\n", p_adv_report->rssi);
				//NRF_LOG_INFO("dlen = %d\r\n", p_adv_report->dlen); // 这个就是p_data（安卓手机上raw data）的length
			*/

			}
			index += field_length + 1;
	    }
}
