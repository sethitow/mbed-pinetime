
#include "mbed-rtt.h"
#include "SEGGER_RTT.h"

#define MBED_SEGGER_RTT_TERMINAL_INDEX 0

SeggerRTT::SeggerRTT(void)
{
    /* Initialize ITM using internal init function. */
	SEGGER_RTT_ConfigUpBuffer(MBED_SEGGER_RTT_TERMINAL_INDEX, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
}

ssize_t SeggerRTT::write(const void *buffer, size_t size)
{
    SEGGER_RTT_Write(MBED_SEGGER_RTT_TERMINAL_INDEX, buffer, size);

    return size;
}

