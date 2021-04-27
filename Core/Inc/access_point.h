#ifndef __ACCESS_POINT_H
#define __ACCESS_POINT_H

#include <stdbool.h>
#include <wifi_api.h>

#define AP_SSID "STM32F413H-DISCO"
#define AP_PASSWORD "12345678"
#define AP_CHANNEL 11
#define AP_MAX_CONNECTIONS 2

WIFI_APSettings_t APSettings;
WIFI_AP_Clients_t APClients;

bool createAP(void);
bool waitForClientConnection(void);
bool getClients(void);

#endif /*__ACCESS_POINT_H*/
