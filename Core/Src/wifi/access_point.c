#include "wifi/access_point.h"

/**
 * @brief Creates an access point in direct connection mode.
 */
void CreateAPTask(void)
{
	wifiStatus = WIFI_Init();
	if(wifiStatus == WIFI_STATUS_OK)
	{
		wifiStatus = WIFI_ConfigureAP(AP_SSID, AP_PASSWORD, WIFI_ECN_WPA2_PSK, AP_CHANNEL, ES_WIFI_MAX_AP_CLIENTS);
	}
}

/**
 * @brief Terminate the access point.
 */
void CloseAPTask(void)
{
	wifiStatus = WIFI_TerminateAP();
}

/**
 * @brief Waits for a single connection to the AP.
 */
void PollForConnectionToAPTask(void)
{
	wifiStatus = WIFI_ListAPClients(&APClients);
}

/**
 * @brief Creates a single connection TCP server in multi accept mode (so we don't have to wait for acceptance).
 */
void CreateTCPServerTask(void)
{
	socket = 0;

	wifiStatus = WIFI_StartServer(socket, WIFI_TCP_PROTOCOL, TCP_PORT);
}

/**
 * @brief Terminate the TCP server.
 */
void CloseTCPServerTask(void)
{
	wifiStatus = WIFI_StopServer();
}

/**
 * @brief Poll for a client connection to the running TCP server. Wait TCP_WAIT_TIMEOUT ms for the connection.
 */
void PollForTCPClientTask(void)
{
	wifiStatus = WIFI_WaitServerConnection(socket, TCP_WAIT_TIMEOUT, &remoteIP, &remotePort);
}

/*
 * @brief Take user input to either accept or deny a TCP connection.
 */
void CheckTCPClientAcceptanceTask(void)
{
	/* TODO [@fitzgeralaus]
	 *
	 * - Need to ask user if they want to accept the client connection.
	 * - If accepted, then continue.
	 * - If not accepted, then close socket and restart PollForTCPClientTask.
	 */
}

/**
 * @brief Receive any data that has been sent over the TCP server. Wait TCP_RECEIVE_TIMEOUT ms for the data.
 */
void ReceiveDataTask(void)
{
	wifiStatus = WIFI_ReceiveData(socket, recData, ES_WIFI_PAYLOAD_SIZE, &recDataLen, TCP_RECEIVE_TIMEOUT);
}

/**
 * @brief Send some data over the TCP server. Wait TCP_SEND_TIMEOUT ms for the client to accept the data.
 * @param sendData : Pointer to the data to send.
 * @param dataLen : Pointer to the length of the data to send.
 */
void SendDataTask(uint8_t *sendData, uint8_t *dataLen)
{
	wifiStatus = WIFI_SendData(socket, sendData, *dataLen, &sentDataLen, TCP_SEND_TIMEOUT);
}
