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
#define TCP_PORT 8080
#define REC_DATA_SIZE 1024

WIFI_APSettings_t APSettings;
WIFI_AP_Clients_t APClients;
uint8_t socket;
uint8_t recData[REC_DATA_SIZE];
uint8_t recDataLen;
uint8_t sentDataLen;

bool createAP(void);
bool getClients(void);
bool startWebServer(void);
bool receiveData(void);
bool sendData(uint8_t *sendData, uint8_t *sendDataLen);

#endif /*__ACCESS_POINT_H*/
