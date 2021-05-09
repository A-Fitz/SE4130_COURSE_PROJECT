/**
 ******************************************************************************
 * @file    wifi.c
 * @author  MCD Application Team
 * @brief   WIFI interface file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "wifi/wifi_api.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ES_WIFIObject_t EsWifiObj;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initialiaze the LL part of the WIFI core
 * @param  None
 * @retval Operation status
 */
WIFI_Status_t WIFI_Init(void) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_RegisterBusIO(&EsWifiObj, SPI_WIFI_Init, SPI_WIFI_DeInit,
			SPI_WIFI_Delay, SPI_WIFI_SendData, SPI_WIFI_ReceiveData)
			== ES_WIFI_STATUS_OK) {

		if (ES_WIFI_Init(&EsWifiObj) == ES_WIFI_STATUS_OK) {
			ret = WIFI_STATUS_OK;
		}
	}
	return ret;
}

/**
 * @brief Configure a new direct connect access point.
 * @param ssid : pointer to the SSID string
 * @param pass : pointer to the password string
 * @param ecn : the type of security
 * @param channel : the channel value
 * @param max_conn : the maximum number of connections
 * @retval Operation success
 */
WIFI_Status_t WIFI_ConfigureAP(const char *ssid, const char *pass, WIFI_Ecn_t ecn, uint8_t channel, uint8_t max_conn) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;
	ES_WIFI_APConfig_t ApConfig;

	strncpy((char*) ApConfig.SSID, ssid, ES_WIFI_MAX_SSID_NAME_SIZE);
	strncpy((char*) ApConfig.Pass, pass, ES_WIFI_MAX_PSWD_NAME_SIZE);
	ApConfig.Channel = channel;
	ApConfig.MaxConnections = ES_WIFI_MAX_AP_CLIENTS;
	ApConfig.Security = (ES_WIFI_SecurityType_t) ecn;

	if (ES_WIFI_ActivateAP(&EsWifiObj, &ApConfig) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief Terminate the access point.
 * @retval Operation success
 */
WIFI_Status_t WIFI_TerminateAP() {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_DeactivateAP(&EsWifiObj) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief List the connected AP clients
 * @param APClients: Pointer to array of AP clients
 * @retval Operation success
 */
WIFI_Status_t WIFI_ListAPClients(WIFI_AP_Clients_t *APClients) {
	uint8_t APClientCount;
	WIFI_Status_t ret = WIFI_STATUS_ERROR;
	ES_WIFI_AP_Clients_t esWifiAPClients;

	if (ES_WIFI_ListAPClients(&EsWifiObj, &esWifiAPClients)
			== ES_WIFI_STATUS_OK) {
		if (esWifiAPClients.count > 0) {
			APClients->count = esWifiAPClients.count;
			memcpy(&(APClients->count), &(esWifiAPClients.count),
					sizeof(esWifiAPClients.count));
			for (APClientCount = 0; APClientCount < APClients->count;
					APClientCount++) {
				APClients->Clients[APClientCount].ClientNumber =
						esWifiAPClients.Clients[APClientCount].ClientNumber;
				memcpy(APClients->Clients[APClientCount].ClientMAC,
						esWifiAPClients.Clients[APClientCount].ClientMAC, 6);
				APClients->Clients[APClientCount].ClientRSSI =
						esWifiAPClients.Clients[APClientCount].ClientRSSI;
			}
		}
		ret = WIFI_STATUS_OK;
	}

	return ret;
}

/**
 * @brief  This function retrieves the WiFi interface's MAC address.
 * @retval Operation Status.
 */
WIFI_Status_t WIFI_GetMAC_Address(uint8_t *mac) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_GetMACAddress(&EsWifiObj, mac) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  This function retrieves the WiFi interface's IP address.
 * @retval Operation Status.
 */
WIFI_Status_t WIFI_GetIP_Address(uint8_t *ipaddr) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (EsWifiObj.NetSettings.IsConnected) {
		memcpy(ipaddr, EsWifiObj.NetSettings.IP_Addr, 4);
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  This function retrieves the WiFi interface's Gateway address.
 * @retval Operation Status.
 */
WIFI_Status_t WIFI_GetGateway_Address(uint8_t *Gateway_addr) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (EsWifiObj.NetSettings.IsConnected) {
		memcpy(Gateway_addr, EsWifiObj.NetSettings.Gateway_Addr, 4);
		ret = WIFI_STATUS_OK;
	}
	return ret;
}
/**
 * @brief  Ping an IP address in the network
 * @param  ipaddr : array of the IP address
 * @retval Operation status
 */
WIFI_Status_t WIFI_Ping(uint8_t *ipaddr, uint16_t count, uint16_t interval_ms) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_Ping(&EsWifiObj, ipaddr, count, interval_ms)
			== ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Configure and start a Server
 * @param  type : Connection type TCP/UDP
 * @param  name : name of the connection
 * @param  port : Remote port
 * @retval Operation status
 */
WIFI_Status_t WIFI_StartServer(uint32_t socket, WIFI_Protocol_t protocol, uint16_t port) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;
	ES_WIFI_Conn_t conn;
	conn.Number = socket;
	conn.LocalPort = port;
	conn.Type =
			(protocol == WIFI_TCP_PROTOCOL) ?
					ES_WIFI_TCP_CONNECTION : ES_WIFI_UDP_CONNECTION;
	if (ES_WIFI_StartServerSingleConn(&EsWifiObj, &conn) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Wait for a client connection to the server
 * @param  socket : socket
 * @retval Operation status
 */
WIFI_Status_t WIFI_WaitServerConnection(int socket, uint32_t Timeout, uint8_t *RemoteIp, uint16_t *RemotePort) {
	ES_WIFI_Conn_t conn;
	ES_WIFI_Status_t ret;

	conn.Number = socket;

	ret = ES_WIFI_WaitServerConnection(&EsWifiObj, Timeout, &conn);

	if (ES_WIFI_STATUS_OK == ret) {
		if (RemotePort)
			*RemotePort = conn.RemotePort;
		if (RemoteIp) {
			memcpy(RemoteIp, conn.RemoteIP, sizeof(conn.RemoteIP));
		}
		return WIFI_STATUS_OK;
	}

	if (ES_WIFI_STATUS_TIMEOUT == ret) {
		if (RemotePort)
			*RemotePort = 0;
		if (RemoteIp) {
			memset(RemoteIp, 0, sizeof(conn.RemoteIP));
		}
		return WIFI_STATUS_TIMEOUT;
	}

	return WIFI_STATUS_ERROR;
}

/**
 * @brief Close the socket for the server. Allows for more connections.
 * @retval Operation success
 */
WIFI_Status_t WIFI_CloseSocket() {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_CloseSocketSingleConn(&EsWifiObj) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Stop a server
 * @retval Operation status
 */
WIFI_Status_t WIFI_StopServer() {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_StopServerSingleConn(&EsWifiObj) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}
/**
 * @brief  Send Data on a socket
 * @param  pdata : pointer to data to be sent
 * @param  Reqlen : packet size
 * @param  SentDatalen : length of data to be sent
 * @param  Timeout : how long to wait to send
 * @retval Operation status
 */
WIFI_Status_t WIFI_SendData(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *SentDatalen, uint32_t Timeout) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_SendData(&EsWifiObj, socket, pdata, Reqlen, SentDatalen, Timeout)
			== ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}

	return ret;
}

/**
 * @brief  Receive Data from a socket
 * @param  pdata : pointer to Rx buffer
 * @param  Reqlen : packet size
 * @param  *len :  pointer to length of data
 * @param  Timeout : how long to wait to receive
 * @retval Operation status
 */
WIFI_Status_t WIFI_ReceiveData(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *RcvDatalen, uint32_t Timeout) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_ReceiveData(&EsWifiObj, socket, pdata, Reqlen, RcvDatalen, Timeout)
			== ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Customize module data
 * @param  name : MFC name
 * @param  Mac :  Mac Address
 * @retval Operation status
 */
WIFI_Status_t WIFI_SetOEMProperties(const char *name, uint8_t *Mac) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_SetProductName(&EsWifiObj, (uint8_t*) name)
			== ES_WIFI_STATUS_OK) {
		if (ES_WIFI_SetMACAddress(&EsWifiObj, Mac) == ES_WIFI_STATUS_OK) {
			ret = WIFI_STATUS_OK;
		}
	}
	return ret;
}

/**
 * @brief  Reset the WIFI module
 * @retval Operation status
 */
WIFI_Status_t WIFI_ResetModule(void) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_ResetModule(&EsWifiObj) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Restore module default configuration
 * @retval Operation status
 */
WIFI_Status_t WIFI_SetModuleDefault(void) {
	WIFI_Status_t ret = WIFI_STATUS_ERROR;

	if (ES_WIFI_ResetToFactoryDefault(&EsWifiObj) == ES_WIFI_STATUS_OK) {
		ret = WIFI_STATUS_OK;
	}
	return ret;
}

/**
 * @brief  Update module firmware
 * @param  location : Binary Location IP address
 * @retval Operation status
 */
WIFI_Status_t WIFI_ModuleFirmwareUpdate(const char *location) {
	return WIFI_STATUS_NOT_SUPPORTED;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

