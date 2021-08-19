/////////////////////////////////////////////////////////////////////////////////
// File Name: spi_api.h
//
// Description: Header file for API specification
//
// Author: SEIKO EPSON
//
// History: 2008/04/18 1st. design
//
// Copyright(c) SEIKO EPSON CORPORATION 2008, All rights reserved.
//
// $Id: spi_api.h,v 1.1.1.1 2008/08/28 07:12:47 bish2310 Exp $
/////////////////////////////////////////////////////////////////////////////////

#ifndef	_SPI_API_H_
#define	_SPI_API_H_
#ifdef __cplusplus
extern "C" {
#endif
// SPI transfer
#define	SPI_MSGRDY_TIMEOUT	1
// Error definition for API function
#define SPIERR_TIMEOUT				1
#define SPIERR_SUCCESS				0
#define SPIERR_NULL_PTR				-1
#define SPIERR_GET_ERROR_CODE		-2
#define SPIERR_RESERVED_MESSAGE_ID	-3
#define SPIERR_ISC_VERSION_RESP		-4
// Definition of checksum function
//#define CHECKSUM 0 No need in simple application
cy_rslt_t EPSON_Initialize(void);
int S1V30340_Play_Specific_Audio(unsigned char aucIscSequencer_element);
unsigned char SPI_SendReceiveByte(unsigned char	ucSendData);
int SPI_SendMessageSPI_SendMessage(unsigned char *pucSendMessage, unsigned short *pusReceivedMessageID);
// function for test
unsigned short GetMessageErrorCode(void);
unsigned short GetBlockedMessageID(void);
unsigned short GetSequenceStatus(void);
#ifdef __cplusplus
}
#endif
#endif //!_SPI_API_H_