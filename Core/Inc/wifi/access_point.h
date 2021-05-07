#ifndef __ACCESS_POINT_H
#define __ACCESS_POINT_H

#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include "wifi/wifi_api.h"

#define AP_SSID "STM32F413H-DISCO"
#define AP_PASSWORD "12345678"
#define AP_CHANNEL 11
#define TCP_PORT 8080
#define TCP_WAIT_TIMEOUT 10000
#define TCP_RECEIVE_TIMEOUT 50
#define TCP_SEND_TIMEOUT 500

WIFI_Status_t wifiStatus;

WIFI_APSettings_t APSettings;
WIFI_AP_Clients_t APClients;

uint8_t socket;
uint8_t remoteIP;
uint16_t remotePort;

uint8_t recData[ES_WIFI_PAYLOAD_SIZE];
uint16_t recDataLen;
uint16_t sentDataLen;

void AP_CreateAP(void);
void AP_CloseAP(void);
void AP_PollForConnectionToAP(void);
void AP_CreateTCPServer(void);
void AP_CloseTCPServer(void);
void AP_PollForTCPClient(void);
void AP_ReceiveData(void);
void AP_SendData(uint8_t *sendData, uint8_t dataLen);

#endif /*__ACCESS_POINT_H*/
