#include "access_point.h"

/**
 * @brief Creates an access point in direct connection mode.
 * @retval Success of procedure.
 */
bool createAP(void)
{
	if(WIFI_Init() ==  WIFI_STATUS_OK) {
		if(WIFI_ConfigureAP(AP_SSID, AP_PASSWORD, WIFI_ECN_WPA2_PSK, AP_CHANNEL, AP_MAX_CONNECTIONS) == WIFI_STATUS_OK)
		{
			return true;
		} else {
		  return false;
		}
	} else {
	  return false;
	}
}
