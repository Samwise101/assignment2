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
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include <math.h>

#include "xfilter_image.h"

#define DATA_SIZE 120

int main()
{
    init_platform();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");

	XFilter_image ip_inst;
	XFilter_image_Config *ip_cfg;

	// Initialize the IP core
	ip_cfg = XFilter_image_LookupConfig(XPAR_XFILTER_IMAGE_0_DEVICE_ID);


	if (ip_cfg == NULL) {
		printf("Error: Could not find the IP core configuration.\n");
		return XST_FAILURE;
	}

	int status = XFilter_image_CfgInitialize(&ip_inst, ip_cfg);
	if (status != XST_SUCCESS) {
		printf("Error: Could not initialize the IP core.\n");
		return XST_FAILURE;
	}

	char s[4];

	uint8_t rowAbove[DATA_SIZE];
    uint8_t rowCenter[DATA_SIZE];
    uint8_t rowBelow[DATA_SIZE];
    uint8_t outRow[DATA_SIZE];

    for(uint8_t i = 0; i < DATA_SIZE; i++){
        rowAbove[i] = 0;
        rowCenter[i] = 0;
        rowBelow[i] = 0;
    }

    uint8_t image[DATA_SIZE][DATA_SIZE];
    int cx = DATA_SIZE / 2;
    int cy = DATA_SIZE / 2;

    printf("IMAGE");
    for (int y = 0; y < DATA_SIZE; y++) {
         for (int x = 0; x < DATA_SIZE; x++) {
             uint8_t checker = ((x / 32) % 2 == (y / 32) % 2) ? 200 : 50;

             // Approximate distance-based circles using Manhattan distance
             int manhattan_distance = abs(x - cx) + abs(y - cy);
             uint8_t circle_pattern = (manhattan_distance / 32) % 2 == 0 ? 150 : 100;

             // Combine patterns by averaging
             image[y][x] = (checker + circle_pattern) / 2;
     		 printf("%d ", image[y][x]);

         }
     }

    printf("FILTERED\n\r");
	for(int i = 0; i < DATA_SIZE; i++){
		for(int j = 0; j < DATA_SIZE; j++){
			rowAbove[j] = image[i][j];
			rowCenter[j] = image[i+1][j];
			rowBelow[j] = image[i+2][j];;
		}
		XFilter_image_Write_rowAbove_Bytes(&ip_inst, 0, (int*)rowAbove, DATA_SIZE);
		XFilter_image_Write_rowCenter_Bytes(&ip_inst, 0, (int*)rowCenter, DATA_SIZE);
		XFilter_image_Write_rowBelow_Bytes(&ip_inst, 0, (int*)rowBelow, DATA_SIZE);

		XFilter_image_Start(&ip_inst);

	    // Wait for the IP core to finish
	    while (!XFilter_image_IsDone(&ip_inst));

		XFilter_image_Read_outputRow_Words(&ip_inst, 0, outRow, DATA_SIZE);

		for(int i = 0; i < DATA_SIZE; i++){
			printf("%d ", outRow[i]);
		}
	}


    cleanup_platform();
    return 0;
}
