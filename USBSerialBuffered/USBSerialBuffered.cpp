/*
 * $Id: bulkserial.h,v 1.3 2018/02/23 15:04:29 grimrath Exp $
 * This is an unpublished work copyright (c) 2017 HELIOS Software GmbH
 * 30827 Garbsen, Germany
 */

#include <algorithm>
#include <mbed.h>
// #include "arch.h"

#ifdef FEATURE_USBSERIAL

#include "USBSerial.h"
#include "USBSerialBuffered.h"

USBSerialBuffered::USBSerialBuffered(int MaxBuffSize, uint16_t vendor_id, uint16_t product_id, uint16_t product_release, bool connect_blocking)
: USBSerial(vendor_id, product_id, product_release, connect_blocking)
, mFullBuffSize(MaxBuffSize)
{
    ASSERT(mFullBuffSize > CorkBuffSize && "FullBuff must be larger than CorkBuff");
    m_buff = new char[mFullBuffSize];
    m_irq_buffused = 0;
    POISONMEM(m_buff, mFullBuffSize);
}

USBSerialBuffered::~USBSerialBuffered() {
    delete[] m_buff;
}

//-----------------------------------------------------------------------------

int USBSerialBuffered::irqbuff_acquire() {
    core_util_critical_section_enter();
    return help_atomic_load_relaxed(&m_irq_buffused);
}

void USBSerialBuffered::irqbuff_release(int buffused) {
    help_atomic_store_relaxed(&m_irq_buffused, buffused);
    irq_barrier();
    core_util_critical_section_exit();
}

//-----------------------------------------------------------------------------

int USBSerialBuffered::printf_irqsafe(const char *fmt, ...) {
    std::va_list va;
    va_start(va, fmt);
    int nchars = vprintf_irqsafe(fmt, va);
    va_end(va);
    return nchars;
}

int USBSerialBuffered::vprintf_irqsafe(const char *fmt, std::va_list va) {
    if (RunningInInterrupt()) {
        int buffused = irqbuff_acquire();
        int bspc = mFullBuffSize - buffused;
        ASSERT(bspc >= 0);
        int nchars = vsnprintf(m_buff + buffused, bspc, fmt, va);
        if (nchars >= bspc) {
            memcpy(m_buff + mFullBuffSize - 4, "...\n", 4);
            buffused = mFullBuffSize;
        } else {
            buffused += nchars;
        }
        irqbuff_release(buffused);
        return nchars;
    } else {
        return USBSerial::vprintf(fmt, va);
    }
}

//-----------------------------------------------------------------------------

void USBSerialBuffered::flush() {
    int flushedbytes = 0;
    int buffused = 0;
    while (1) {
        bool wasequal = help_atomic_compare_and_swap(&m_irq_buffused, buffused, 0);
        if (wasequal)
            break;
        //
        // This only works because @ref print_irq always _increases_ @c m_irq_buffused (but never(!) decreases this variable)
        //
        buffused = help_atomic_load_relaxed(&m_irq_buffused);   // __sync_* implementation requires this refetch
        while (buffused != flushedbytes) {
            int towrite = std::min(buffused - flushedbytes, static_cast<int>(CorkBuffSize));
            if (connected()) {
                writeBlock(reinterpret_cast<uint8_t *>(m_buff + flushedbytes), towrite);
            }
            flushedbytes += towrite;
        }
    }
}

void USBSerialBuffered::putc_normal(int c) {
    while (1) {
        int buffused = help_atomic_load_relaxed(&m_irq_buffused);
        if (buffused >= CorkBuffSize) {
            flush();
        } else {
//             static bool TESTonce;
//             if (! TESTonce) {
//                 printf_irqsafe("ppp");
//                 TESTonce = true;
//             }
            ASSERT(buffused + 1 <= CorkBuffSize);
            bool wasequal = help_atomic_compare_and_swap(&m_irq_buffused, buffused, buffused + 1);
            if (wasequal) {
                m_buff[buffused] = c;   // alloc successful
                return;
            } else {
                // An irq extended m_irq_buffused, start over
            }
        }
    }
}

int USBSerialBuffered::_putc(int c) {
    putc_normal(c);
    if (c == '\n') {
        flush();
    }
    return connected() ? 1 : 0;
}

#endif
