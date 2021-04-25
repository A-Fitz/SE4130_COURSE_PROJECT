#include "access_point.h"

void createAP(void)
{
	if(WIFI_Init() ==  WIFI_STATUS_OK) {
		// If I uncomment this, then osDelay still gets stuck, but a hard fault does not occur.
		if(WIFI_ConfigureAP(AP_SSID, AP_PASSWORD, WIFI_ECN_WPA2_PSK, AP_CHANNEL, AP_MAX_CONNECTIONS) == WIFI_STATUS_OK)
		{
			accessPointCreated = true;
		}
		return;
	}
	return;
}
