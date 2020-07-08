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
#include "xUSBSerial.h"
#include "main.h"

int xUSBSerial::_putc(int c) {
    if (!terminal_connected)
        return 0;
    send((uint8_t *)&c, 1);
    return 1;
}

int xUSBSerial::_getc() {
    if (!_rx_in_progress && buf.freespace() >= MAX_PACKET_SIZE_EPBULK * 2) {
		if (issueReadStart())
			_rx_in_progress = true;
	}
	
    uint8_t c = 0;
    while (buf.isEmpty());
    buf.dequeue(&c);
    return c;
}


bool xUSBSerial::writeBlock(uint8_t * buf, uint16_t size) {
    if(size > MAX_PACKET_SIZE_EPBULK) {
        return false;
    }
    if(!send(buf, size)) {
        return false;
    }
    return true;
}

int xUSBSerial::readable() {
    if (!_rx_in_progress && buf.freespace() >= MAX_PACKET_SIZE_EPBULK * 2) {
		if (issueReadStart())
			_rx_in_progress = true;
	}
	return available() ? 1 : 0;
}


bool xUSBSerial::EPBULK_OUT_callback() {
    uint8_t c[MAX_PACKET_SIZE_EPBULK];
    uint32_t size = 0;
	
	//we read the packet received and put it on the circular buffer

	if (!readEP(c, &size, false)) // will read without new async request
		return false;

    for (uint32_t i = 0; i < size; i++) {
        buf.queue(c[i]);
    }
	if (buf.freespace() >= MAX_PACKET_SIZE_EPBULK * 2)
		issueReadStart(); // issue new sync read request
	else
		_rx_in_progress = false;

    //call a potential handlenr
    if (rx)
        rx.call();

    return true;
}

size_t xUSBSerial::available() {
    return buf.available();
}

bool xUSBSerial::connected() {
    return terminal_connected;
}

#endif // FEATURE_USBSERIAL
