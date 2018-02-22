/*
 * $Id: bulkserial.h,v 1.3 2018/02/23 15:04:29 grimrath Exp $
 * This is an unpublished work copyright (c) 2018 HELIOS Software GmbH
 * 30827 Garbsen, Germany
 */
#ifndef __USBSERIALBUFFERED_H__
#define __USBSERIALBUFFERED_H__

#include "USBSerial.h"

/** This class is a wrapper around @ref USBSerial such that sending
 * of serial data over USB is supported in (and outside) of interrupt context.
 * In addition it buffers characters (similiar to the I/O buffering of stdio)
 * before starting a USB data transmit. Silently discards data if the
 * @ref USBSerial object is not connected to the USB host.
 */
class USBSerialBuffered : public USBSerial {
public:
    USBSerialBuffered(int MaxBuffSize = 128, uint16_t vendor_id = 0x1f00, uint16_t product_id = 0x2012, uint16_t product_release = 0x0001, bool connect_blocking = true);
    ~USBSerialBuffered();

    /** sends internally queued but not yet sent data. Is blocking. Must not
     * be called from interrupt context. */
    void flush();

    /** Writes a formatted string into an internal buffer for later sending.
     * with e.g. @ref flush . Explicitly designed to be called from interrupt
     * context. If the string does not fit into the internal buffer it is
     * silently truncated. */
    int printf_irqsafe(const char *fmt, ...) __attribute__((format(printf,2,3)));

    /** the varargs variant of printf_ */
    int vprintf_irqsafe(const char *fmt, std::va_list ap);

protected:
    /** Called from @ref Stream::printf and other stdio-like methods from the base class @ref Stream . */
    virtual int _putc(int c);

private:
    enum {
        CorkBuffSize = MAX_PACKET_SIZE_EPBULK,
    };

    static inline bool RunningInInterrupt() {
        return SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
    }

    int irqbuff_acquire();
    void irqbuff_release(int buffused);

    void putc_normal(int c);

private:
    int mFullBuffSize;
    char *m_buff;
    int m_irq_buffused;
};

#endif // __USBSERIALBUFFERED_H__
