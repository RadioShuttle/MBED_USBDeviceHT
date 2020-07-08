#include "mbed.h"
#include "xPinMap.h"

#ifdef FEATURE_USBSERIAL

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

#include "stdint.h"
#include "xUSBCDC.h"

#ifndef DPRINTF_AVAILABLE
#define	dprintf(...)	void()
#endif
#define DEFAULT_CONFIGURATION (1)


#define MAX_CDC_REPORT_SIZE MAX_PACKET_SIZE_EPBULK

xUSBCDC::xUSBCDC(uint16_t vendor_id, uint16_t product_id, uint16_t product_release, bool connect_blocking): xUSBDevice(vendor_id, product_id, product_release) {
    
    terminal_connected = false;

    memset(&cdc_line_coding, 0, sizeof(cdc_line_coding));
	cdc_line_coding.baud = 9600;
    cdc_line_coding.stopbits = 0;
    cdc_line_coding.parity = 0;
    cdc_line_coding.bits = 8;

	memset(&_lineState, 0, sizeof(_lineState));
	_gotBreakReset = false;

    xUSBDevice::connect(connect_blocking);
}

void xUSBCDC::USBCallback_busReset(void) {
	if (terminal_connected)
		sleep_manager_unlock_deep_sleep();
    terminal_connected = false;
};

bool xUSBCDC::USBCallback_request(void) {
    /* Called in ISR context */

    bool success = false;
    CONTROL_TRANSFER * transfer = getTransferPtr();

    /* Process class-specific requests */

    if (transfer->setup.bmRequestType.Type == CLASS_TYPE) {
		ClassTypeCmds cmd = (ClassTypeCmds)transfer->setup.bRequest;
		switch (cmd) {
            case CDC_GET_LINE_CODING:
				dprintf("CDC_GET_LINE_CODING");
                transfer->remaining = sizeof(cdc_line_coding);
                transfer->ptr = (uint8_t *)&cdc_line_coding;
                transfer->direction = DEVICE_TO_HOST;
                success = true;
                break;
            case CDC_SET_LINE_CODING:
                transfer->remaining = sizeof(cdc_line_coding);
                transfer->notify = true;
                success = true;
                break;
            case CDC_SET_CONTROL_LINE_STATE:
            	memcpy(&_lineState, &transfer->setup.wValue, sizeof(_lineState));
				dprintf("CDC_SET_CONTROL_LINE_STATE: DTR=%d RTS=%d", _lineState.DTR , _lineState.RTS);
                if (_lineState.DTR) {
                	_gotBreakReset = false;
                    terminal_connected = true;
					sleep_manager_lock_deep_sleep();
                } else {
                	if (_gotBreakReset) {
                    	_gotBreakReset = false;
						if (terminal_connected)
							sleep_manager_unlock_deep_sleep();
                    	terminal_connected = false;
                    }
                }
				lineStateChanged(_lineState.DTR, _lineState.RTS);
                success = true;
                break;
			case CDC_SEND_BREAK:
					if (transfer->setup.wValue == 0 && transfer->setup.wLength == 4) {
						// SET_RINGER_PARMS case
						dprintf("SET_RINGER_PARMS: ");
						RingerParms r;
						memcpy(&r, transfer->ptr, 4);
						success = true;
						break;
					}
					if (transfer->setup.wValue == 0) {
						dprintf("CDC_SEND_BREAK: reset");
						_gotBreakReset = true;
					} else {
						dprintf("CDC_SEND_BREAK: delay_ms: %d", transfer->setup.wValue);
					}
					if (!_lineState.DTR && terminal_connected) {
						/*
						 * Delayed discounnt for the Arduino ESP32
						 */
						if (terminal_connected)
							sleep_manager_unlock_deep_sleep();
                    	terminal_connected = false;
                    	_gotBreakReset = false;
					}
					success = true;
				break;
            default:
            	dprintf("UNKNOWN CDC REQEST: %d", transfer->setup.bRequest);
                break;
        }
    }
    if (transfer->setup.bmRequestType.Type == 0xa1 && transfer->setup.bRequest == CDC_SERIAL_STATE) {
    	UARTState state;
    	memset(&state, 0, sizeof(state));
    	// TODO Uart state
		success = true;
	}

    return success;
}

void xUSBCDC::USBCallback_requestCompleted(uint8_t *buf, uint32_t length) {

    CONTROL_TRANSFER * transfer = getTransferPtr();

    /* Process class-specific requests */
    if (transfer->setup.bmRequestType.Type == CLASS_TYPE) {
		ClassTypeCmds cmd = (ClassTypeCmds)transfer->setup.bRequest;
		switch(cmd) {
		case CDC_SET_LINE_CODING:
			if (length == sizeof(cdc_line_coding)) {
				dprintf("CDC_SET_LINE_CODING: baud rate: %ld", cdc_line_coding.baud);
				if (memcmp(&cdc_line_coding, buf, sizeof(cdc_line_coding))) {
					memcpy(&cdc_line_coding, buf, sizeof(cdc_line_coding));

					lineCodingChanged(cdc_line_coding.baud, cdc_line_coding.bits, cdc_line_coding.parity, cdc_line_coding.stopbits);
				}
			} else {
				dprintf("CDC Callback: CDC_SET_LINE_CODING: wrong size: %ld", length);
			}
			break;
		default:
			dprintf("CDC Callback: unkown command: %d", cmd);
		}
    }
}

// Called in ISR context
// Set configuration. Return false if the
// configuration is not supported.
bool xUSBCDC::USBCallback_setConfiguration(uint8_t configuration) {
    if (configuration != DEFAULT_CONFIGURATION) {
        return false;
    }

    // Configure endpoints > 0
    addEndpoint(EPINT_IN, MAX_PACKET_SIZE_EPINT);
    addEndpoint(EPBULK_IN, MAX_PACKET_SIZE_EPBULK);
    addEndpoint(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

    // We activate the endpoint to be able to recceive data
    readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
    return true;
}

bool xUSBCDC::send(uint8_t * buffer, uint32_t size) {
    return xUSBDevice::write(EPBULK_IN, buffer, size, MAX_CDC_REPORT_SIZE);
}

bool xUSBCDC::issueReadStart() {
    return readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
}

bool xUSBCDC::readEP(uint8_t * buffer, uint32_t * size, bool readstart) {
    if (!xUSBDevice::readEP(EPBULK_OUT, buffer, size, MAX_CDC_REPORT_SIZE)) {
        return false;
	}
	if (readstart) {
    	if (!readStart(EPBULK_OUT, MAX_CDC_REPORT_SIZE))
        	return false;
	}
    return true;
}

bool xUSBCDC::readEP_NB(uint8_t * buffer, uint32_t * size, bool readstart) {
    if (!xUSBDevice::readEP_NB(EPBULK_OUT, buffer, size, MAX_CDC_REPORT_SIZE)) {
        return false;
	}
	if (readstart) {
    	if (!readStart(EPBULK_OUT, MAX_CDC_REPORT_SIZE))
	        return false;
	}
    return true;
}


const uint8_t * xUSBCDC::deviceDesc() {
    uint8_t deviceDescriptorTemp[] = {
        18,                   // bLength
        1,                    // bDescriptorType
        0x10, 0x01,           // bcdUSB
        2,                    // bDeviceClass
        0,                    // bDeviceSubClass
        0,                    // bDeviceProtocol
        MAX_PACKET_SIZE_EP0,  // bMaxPacketSize0
        (uint8_t)(LSB(VENDOR_ID)), (uint8_t)(MSB(VENDOR_ID)),  // idVendor
        (uint8_t)(LSB(PRODUCT_ID)), (uint8_t)(MSB(PRODUCT_ID)),// idProduct
        0x00, 0x01,           // bcdDevice
        1,                    // iManufacturer
        2,                    // iProduct
        3,                    // iSerialNumber
        1                     // bNumConfigurations
    };
    MBED_ASSERT(sizeof(deviceDescriptorTemp) == sizeof(deviceDescriptor));
    memcpy(deviceDescriptor, deviceDescriptorTemp, sizeof(deviceDescriptor));
    return deviceDescriptor;
}

const uint8_t * xUSBCDC::stringIinterfaceDesc() {
    static const uint8_t stringIinterfaceDescriptor[] = {
        0x08,
        STRING_DESCRIPTOR,
        'C',0,'D',0,'C',0,
    };
    return stringIinterfaceDescriptor;
}

const uint8_t * xUSBCDC::stringIproductDesc() {
    static const uint8_t stringIproductDescriptor[] = {
        0x16,
        STRING_DESCRIPTOR,
        'C',0,'D',0,'C',0,' ',0,'D',0,'E',0,'V',0,'I',0,'C',0,'E',0
    };
    return stringIproductDescriptor;
}


#define CONFIG1_DESC_SIZE (9+8+9+5+5+4+5+7+9+7+7)

const uint8_t * xUSBCDC::configurationDesc() {
    static const uint8_t configDescriptor[] = {
        // configuration descriptor
        9,                      // bLength
        2,                      // bDescriptorType
        LSB(CONFIG1_DESC_SIZE), // wTotalLength
        MSB(CONFIG1_DESC_SIZE),
        2,                      // bNumInterfaces
        1,                      // bConfigurationValue
        0,                      // iConfiguration
        0x80,                   // bmAttributes
        50,                     // bMaxPower

        // IAD to associate the two CDC interfaces
        0x08,                   // bLength
        0x0b,                   // bDescriptorType
        0x00,                   // bFirstInterface
        0x02,                   // bInterfaceCount
        0x02,                   // bFunctionClass
        0x02,                   // bFunctionSubClass
        0,                      // bFunctionProtocol
        0,                      // iFunction

        // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        9,                      // bLength
        4,                      // bDescriptorType
        0,                      // bInterfaceNumber
        0,                      // bAlternateSetting
        1,                      // bNumEndpoints
        0x02,                   // bInterfaceClass
        0x02,                   // bInterfaceSubClass
        0x01,                   // bInterfaceProtocol
        0,                      // iInterface

        // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x00,                   // bDescriptorSubtype
        0x10, 0x01,             // bcdCDC

        // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x01,                   // bDescriptorSubtype
        0x03,                   // bmCapabilities
        1,                      // bDataInterface

        // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
        4,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x02,                   // bDescriptorSubtype
        0x06,                   // bmCapabilities

        // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x06,                   // bDescriptorSubtype
        0,                      // bMasterInterface
        1,                      // bSlaveInterface0

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH,     // bLength
        ENDPOINT_DESCRIPTOR,            // bDescriptorType
        PHY_TO_DESC(EPINT_IN),          // bEndpointAddress
        E_INTERRUPT,                    // bmAttributes (0x03=intr)
        LSB(MAX_PACKET_SIZE_EPINT),     // wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPINT),     // wMaxPacketSize (MSB)
        16,                             // bInterval




        // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        9,                          // bLength
        4,                          // bDescriptorType
        1,                          // bInterfaceNumber
        0,                          // bAlternateSetting
        2,                          // bNumEndpoints
        0x0A,                       // bInterfaceClass
        0x00,                       // bInterfaceSubClass
        0x00,                       // bInterfaceProtocol
        0,                          // iInterface

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        PHY_TO_DESC(EPBULK_IN),     // bEndpointAddress
        E_BULK,                     // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (MSB)
        0,                          // bInterval

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        PHY_TO_DESC(EPBULK_OUT),    // bEndpointAddress
        E_BULK,                     // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (MSB)
        0                           // bInterval
    };
    return configDescriptor;
}

#endif // FEATURE_USBSERIAL
