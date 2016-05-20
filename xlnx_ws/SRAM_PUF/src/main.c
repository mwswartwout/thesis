/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include <ff.h>
#include "sram_puf.h"

void print(char *str);

int main()
{
	int *OCM0_ADDR = 0x0;
	int *OCM1_ADDR = 0x10000;
	int *OCM2_ADDR = 0x20000;
	int *OCM3_ADDR = 0xFFFF0000;

	char OCM0_FN[32] = "ocm_0.bin";
	char OCM1_FN[32] = "ocm_1.bin";
	char OCM2_FN[32] = "ocm_2.bin";
	char OCM3_FN[32] = "ocm_3.bin";

	int OCM0 = xilffs_polled_example(OCM0_ADDR, OCM0_FN);
	int OCM1 = xilffs_polled_example(OCM1_ADDR, OCM1_FN);
	int OCM2 = xilffs_polled_example(OCM2_ADDR, OCM2_FN);
	int OCM3 = xilffs_polled_example(OCM3_ADDR, OCM3_FN);

	return (OCM0 || OCM1 || OCM2 || OCM3);
}

int read_sram() {
	print("About to init_platform\n\r");
    init_platform();
    print("Hello World\n\r");

    print("Attempting to mount file system...\n\r");
    FATFS fs; // File system object
    FRESULT status; // FatFs return code

    status = f_mount(&fs, "", 0);
    switch (status) {
		case FR_OK:
			print("Mounting successful!\n\r");
			break;
		case FR_INVALID_DRIVE:
			print("Return code FR_INVALID_DRIVE\n\r");
		case FR_DISK_ERR:
			print("Return code FR_DISK_ERR\n\r");
			break;
		case FR_NOT_READY:
			print("Return code FR_NOT_READY\n\r");
			break;
		case FR_NO_FILESYSTEM:
			print("Return code FR_NO_FILESYSTEM\n\r");
			break;
		default:
			print("Return code was not a possible return value from f_mount\n\r");
    }

    FIL file; // File object
    char line[82]; // Line buffer
    print("Opening hello.txt...\n\r");
    status = f_open(&file, "hello.txt", FA_READ);
    if (status == FR_OK) {
    	print("Successfully opened file!\n\r");
    	while (f_gets(line, sizeof line, &file)) {
    		print(line);
    		print("\r");
    	}
    	f_close(&file);
    }

    print("All done, cleaning up now...\n\r");
    cleanup_platform();
    print("Exiting...\n\r");
    return 0;
}
