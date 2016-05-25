/******************************************************************************
*
* Copyright (C) 2013 - 2015 Xilinx, Inc.  All rights reserved.
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
/*****************************************************************************/
/**
*
* @file xilffs_polled_example.c
*
*
* @note This example uses file system with SD to write to and read from
* an SD card using ADMA2 in polled mode.
* To test this example File System should not be in Read Only mode.
*
* This example was tested using SD2.0 card and eMMC (using eMMC to SD adaptor).
*
* None.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who Date     Changes
* ----- --- -------- -----------------------------------------------
* 1.00a hk  10/17/13 First release
* 2.2   hk  07/28/14 Make changes to enable use of data cache.
* 2.5   sk  07/15/15 Used File size as 8KB to test on emulation platform.
*
*</pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xparameters.h"	/* SDK generated parameters */
#include "xsdps.h"		/* SD device driver */
#include "xil_printf.h"
#include "ff.h"
#include "xil_cache.h"
#include "xplatform_info.h"

#include "sram_puf.h"

/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
int FfsSdPolledExample(int *OCM_ADDR, char FileName[32]);

/************************** Variable Definitions *****************************/
static FIL fil;		/* File object */
static FATFS fatfs;
//static char FileName[32] = "Test.bin";
static char *SD_File;
u32 Platform;

#ifdef __ICCARM__
#pragma data_alignment = 32
u8 DestinationAddress[10*1024];
u8 SourceAddress[10*1024];
#pragma data_alignment = 4
#else
u8 DestinationAddress[10*1024] __attribute__ ((aligned(32)));
u8 SourceAddress[10*1024] __attribute__ ((aligned(32)));
#endif

#define TEST 7

/*****************************************************************************/
/**
*
* Main function to call the SD example.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int xilffs_polled_example(int *OCM_ADDR, char FileName[32])
{
	int Status;

	xil_printf("SD Polled File System Example Test \r\n");

	Status = FfsSdPolledExample(OCM_ADDR, FileName);
	if (Status != XST_SUCCESS) {
		xil_printf("SD Polled File System Example Test failed \r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran SD Polled File System Example Test \r\n");

	return XST_SUCCESS;

}

/*****************************************************************************/
/**
*
* File system example using SD driver to write to and read from an SD card
* in polled mode. This example creates a new file on an
* SD card (which is previously formatted with FATFS), write data to the file
* and reads the same data back to verify.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int FfsSdPolledExample(int* OCM_ADDR, char FileName[32])
{
	FRESULT Res;
	UINT NumBytesRead;
	UINT NumBytesWritten;
	u32 BuffCnt;
	u32 FileSize = (8*1024); // 8 KB
	TCHAR *Path = "0:/";


	Platform = XGetPlatform_Info();
	if (Platform == XPLAT_ZYNQ_ULTRA_MP) {
		/*
		 * Since 8MB in Emulation Platform taking long time, reduced
		 * file size to 8KB.
		 */
		FileSize = 8*1024;
	}

	for(BuffCnt = 0; BuffCnt < FileSize; BuffCnt++){
		SourceAddress[BuffCnt] = OCM_ADDR[BuffCnt];
	}

	/*
	 * Register volume work area, initialize device
	 */
	Res = f_mount(&fatfs, Path, 0);

	if (Res != FR_OK) {
		xil_printf("f_mount failed\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	/*
	 * Open file with required permissions.
	 * Here - Creating new file with read/write permissions. .
	 * To open file with write permissions, file system should not
	 * be in Read Only mode.
	 */
	SD_File = (char *)FileName;

	Res = f_open(&fil, SD_File, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if (Res) {
		xil_printf("f_open failed\n\r");
		xil_printf("Filename passed in was: ");
		xil_printf(SD_File);
		xil_printf("\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	/*
	 * Pointer to beginning of file .
	 */
	Res = f_lseek(&fil, 0);
	if (Res) {
		xil_printf("f_lseek failed\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	/*
	 * Write data to file.
	 */
	Res = f_write(&fil, (const void*)SourceAddress, FileSize,
			&NumBytesWritten);
	if (Res) {
		xil_printf("f_write failed\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	/*
	 * Pointer to beginning of file .
	 */
	Res = f_lseek(&fil, 0);
	if (Res) {
		xil_printf("f_lseek after data write failed\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	/*
	 * Read data from file.
	 */
/*
   Res = f_read(&fil, (void*)DestinationAddress, FileSize,
			&NumBytesRead);
	if (Res) {
		return XST_FAILURE;
	}
*/
	/*
	 * Data verification
	 */
/*
	for(BuffCnt = 0; BuffCnt < FileSize; BuffCnt++){
		if(SourceAddress[BuffCnt] != DestinationAddress[BuffCnt]){
			return XST_FAILURE;
		}
	}
*/
	/*
	 * Close file.
	 */
	Res = f_close(&fil);
	if (Res) {
		xil_printf("f_close failed\n\r");
		handle_error(Res);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

void handle_error(FRESULT result) {
	switch(result) {
	case FR_OK:
		xil_printf("(0) Succeeded\n\r");
		break;
	case FR_DISK_ERR:
		xil_printf(" (1) A hard error occurred in the low level disk I/O layer\n\r");
		break;
	case FR_INT_ERR:
		xil_printf(" (2) Assertion failed \n\r");
		break;
	case FR_NOT_READY:
		xil_printf(" (3) The physical drive cannot work\n\r");
		break;
	case FR_NO_FILE:
		xil_printf(" (4) Could not find the file\n\r");
		break;
	case FR_NO_PATH:
		xil_printf(" (5) Could not find the path\n\r");
		break;
	case FR_INVALID_NAME:
		xil_printf(" (6) The path name format is invalid\n\r");
		break;
	case FR_DENIED:
		xil_printf(" (7) Access denied due to prohibited access or directory full\n\r");
		break;
	case FR_EXIST:
		xil_printf(" (8) Access denied due to prohibited access\n\r");
		break;
	case FR_INVALID_OBJECT:
		xil_printf(" (9) The file/directory object is invalid\n\r");
		break;
	case FR_WRITE_PROTECTED:
		xil_printf(" (10) The physical drive is write protected\n\r");
		break;
	case FR_INVALID_DRIVE:
		xil_printf(" (11) The logical drive number is invalid\n\r");
		break;
	case FR_NOT_ENABLED:
		xil_printf(" (12) The volume has no work area\n\r");
		break;
	case FR_NO_FILESYSTEM:
		xil_printf(" (13) There is no valid FAT volume\n\r");
		break;
	case FR_MKFS_ABORTED:
		xil_printf(" (14) The f_mkfs() aborted due to any parameter error\n\r");
		break;
	case FR_TIMEOUT:
		xil_printf(" (15) Could not get a grant to access the volume within defined period\n\r");
		break;
	case FR_LOCKED:
		xil_printf(" (16) The operation is rejected according to the file sharing policy\n\r");
		break;
	case FR_NOT_ENOUGH_CORE:
		xil_printf(" (17) LFN working buffer could not be allocated\n\r");
		break;
	case FR_TOO_MANY_OPEN_FILES:
		xil_printf(" (18) Number of open files > _FS_SHARE\n\r");
		break;
	case FR_INVALID_PARAMETER:
		xil_printf(" (19) Given parameter is invalid\n\r");
		break;
	default:
		xil_printf("Returned error code not recognized as a valid FatFS error\n\r");
	}
}
