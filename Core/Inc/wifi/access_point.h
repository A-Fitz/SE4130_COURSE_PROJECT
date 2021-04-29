#ifndef __ACCESS_POINT_H
#define __ACCESS_POINT_H

#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include "wifi/wifi_api.h"

#define AP_SSID "STM32F413H-DISCO"
#define AP_PASSWORD "12345678"
#define AP_CHANNEL 11
#define AP_MAX_CONNECTIONS 2
#define TCP_PORT 8080
#define TCP_WAIT_TIMEOUT 60000
#define REC_DATA_SIZE 1024
#define REC_PAYLOAD_SIZE 1200

WIFI_APSettings_t APSettings;
WIFI_AP_Clients_t APClients;

uint8_t socket;
uint8_t remoteIP;
uint16_t remotePort;

uint8_t recData[REC_DATA_SIZE];
uint16_t recDataLen;
uint16_t sentDataLen;

bool createAP(void);
bool getClients(void);
bool startTCPServer(void);
bool waitForTCPConnection(void);
bool receiveData(void);
bool sendData(uint8_t *sendData, uint8_t *sendDataLen);

#endif /*__ACCESS_POINT_H*/
