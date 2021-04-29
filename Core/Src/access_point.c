#include "access_point.h"

/**
 * @brief Creates an access point in direct connection mode.
 * @retval Success of procedure.
 */
bool createAP(void)
{
	return WIFI_Init() ==  WIFI_STATUS_OK &&
			WIFI_ConfigureAP(AP_SSID, AP_PASSWORD, WIFI_ECN_WPA2_PSK, AP_CHANNEL, AP_MAX_CONNECTIONS) == WIFI_STATUS_OK;
}

/**
 * @brief Finds all clients connected to the AP, stores in APClients variable.
 * @retval Operation success
 */
bool getClients(void)
{
	return WIFI_ListAPClients(&APClients) == WIFI_STATUS_OK;
}

bool startTCPServer(void)
{
	socket = 0;

	return WIFI_StartServer(socket, WIFI_TCP_PROTOCOL, TCP_PORT) == WIFI_STATUS_OK;
}

bool receiveData(void)
{
	return WIFI_ReceiveData(socket, recData, REC_DATA_SIZE, &recDataLen) == WIFI_STATUS_OK;
}

bool sendData(uint8_t *sendData, uint8_t *sendDataLen)
{
	return WIFI_SendData(socket, sendData, sendDataLen, &sentDataLen) == WIFI_STATUS_OK;
}
