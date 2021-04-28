#ifndef __ACCESS_POINT_H
#define __ACCESS_POINT_H

#include <stdbool.h>
#include <wifi_api.h>
#include <time.h>
#include <stdlib.h>
#include "main.h"

#define AP_SSID "STM32F413H-DISCO"
#define AP_PASSWORD "12345678"
#define AP_CHANNEL 11
#define AP_MAX_CONNECTIONS 2
#define PORT 8080

WIFI_APSettings_t APSettings;
WIFI_AP_Clients_t APClients;
int32_t APSocket;
static   uint8_t resp[1024];
uint16_t respLen;

bool createAP(void);
bool waitForClientConnection(void);
bool getClients(void);
bool startWebServer(void);
bool receiveData(void);

#endif /*__ACCESS_POINT_H*/
