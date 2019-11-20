#include "SEGGER_RTT.h"

#include "mbed.h"


int main()
{
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	SEGGER_RTT_WriteString(0, "Hello World!\r\n\r\n");
	int x = 5;
	while(true)
	{
		SEGGER_RTT_printf(0, "printf Test: %%d,      x : %d.\r\n", x);
		x++;
		ThisThread::sleep_for(1000);
	}

}