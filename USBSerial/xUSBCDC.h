/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef XUSBCDC_H
#define XUSBCDC_H

/* These headers are included for child class. */
#include "xUSBEndpoints.h"
#include "xUSBDescriptor.h"
#include "xUSBDevice_Types.h"

#include "xUSBDevice.h"

class xUSBCDC: public xUSBDevice {
public:

    /*
    * Constructor
    *
    * @param vendor_id Your vendor_id
    * @param product_id Your product_id
    * @param product_release Your preoduct_release
    * @param connect_blocking define if the connection must be blocked if USB not plugged in
    */
    xUSBCDC(uint16_t vendor_id, uint16_t product_id, uint16_t product_release, bool connect_blocking);

protected:

    /*
    * Get device descriptor. Warning: this method has to store the length of the report descriptor in reportLength.
    *
    * @returns pointer to the device descriptor
    */
    virtual const uint8_t * deviceDesc();

    /*
    * Get string product descriptor
    *
    * @returns pointer to the string product descriptor
    */
    virtual const uint8_t * stringIproductDesc();

    /*
    * Get string interface descriptor
    *
    * @returns pointer to the string interface descriptor
    */
    virtual const uint8_t * stringIinterfaceDesc();

    /*
    * Get configuration descriptor
    *
    * @returns pointer to the configuration descriptor
    */
    virtual const uint8_t * configurationDesc();

    /*
    * Send a buffer
    *
    * @param endpoint endpoint which will be sent the buffer
    * @param buffer buffer to be sent
    * @param size length of the buffer
    * @returns true if successful
    */
    bool send(uint8_t * buffer, uint32_t size);

	/*
	 * start an sync read request
	 */
	bool issueReadStart();
    /*
    * Read a buffer from a certain endpoint. Warning: blocking
    *
    * @param endpoint endpoint to read
    * @param buffer buffer where will be stored bytes
    * @param size the number of bytes read will be stored in *size
    * @param maxSize the maximum length that can be read
    * @returns true if successful
    */
    bool readEP(uint8_t * buffer, uint32_t * size, bool readstart = true);

    /*
    * Read a buffer from a certain endpoint. Warning: non blocking
    *
    * @param endpoint endpoint to read
    * @param buffer buffer where will be stored bytes
    * @param size the number of bytes read will be stored in *size
    * @param maxSize the maximum length that can be read
    * @returns true if successful
    */
    bool readEP_NB(uint8_t * buffer, uint32_t * size, bool readstart = true);

    /*
    * Called by USBCallback_requestCompleted when CDC line coding is changed
    * Warning: Called in ISR
    *
    * @param baud The baud rate
    * @param bits The number of bits in a word (5-8)
    * @param parity The parity
    * @param stop The number of stop bits (1 or 2)
    */
    virtual void lineCodingChanged(int baud, int bits, int parity, int stop) {};
    
    /*
     * The DTR/RTS has been updated by the host
	 */
    virtual void lineStateChanged(bool dtr, bool rts) {};


protected:
	struct LineCoding {
		uint32_t 	baud;
		uint8_t		stopbits;
		uint8_t		parity;
		uint8_t		bits;
	} __attribute__((packed));
	MBED_STATIC_ASSERT(sizeof(LineCoding) == 7, "wrong LineCoding size");

	struct LineState {
		bool DTR : 1;
		bool RTS : 1;
		uint16_t reserved : 14;
	} __attribute__((packed));
	MBED_STATIC_ASSERT(sizeof(LineState) == 2, "wrong LineState size");

	struct RingerParms {
		uint8_t	ringerPattern : 8;
		uint8_t	ringerVolume : 8;
		uint16_t reserved : 15;
		bool	hasRinger : 1;
	} __attribute__((packed));
	MBED_STATIC_ASSERT(sizeof(RingerParms) == 4, "wrong RingerParms size");

	struct UARTState {
		bool bRxCarrier : 1;
		bool bTxCarrier : 1;
		bool bBreak : 1;
		bool bRingSignal : 1;
		bool bFraming : 1;
		bool bParity : 1;
		bool bOverRun : 1;
		uint16_t reserved : 9;
	} __attribute__((packed));
	MBED_STATIC_ASSERT(sizeof(UARTState) == 2, "wrong UARTState size");


	enum ClassTypeCmds {
		CDC_SEND_ENCAPSULATED_COMMAND	= 0x00,
		CDC_GET_ENCAPSULATED_RESPONSE	= 0x01,
		CDC_SET_COMM_FEATURE			= 0x02,
		CDC_GET_COMM_FEATURE			= 0x03,
		CDC_CLEAR_COMM_FEATURE			= 0x04,
		CDC_SET_LINE_CODING        		= 0x20,
		CDC_GET_LINE_CODING        		= 0x21,
		CDC_SET_CONTROL_LINE_STATE		= 0x22,
		CDC_SEND_BREAK			   		= 0x23
	};
	enum StateCmds { // type == 0xa1
		CDC_SERIAL_STATE				= 0x20,
	};
	LineCoding cdc_line_coding;
	LineState _lineState;
	bool _gotBreakReset;
    virtual bool USBCallback_request();
    virtual void USBCallback_requestCompleted(uint8_t *buf, uint32_t length);
    virtual bool USBCallback_setConfiguration(uint8_t configuration);
    virtual void USBCallback_busReset(void);
    volatile bool terminal_connected;

};

#endif
