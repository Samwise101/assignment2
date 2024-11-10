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

#include "xparameters.h"
#include "xaxidma.h"
#include "xil_printf.h"
#include "xstatus.h"

// Define the DMA Device ID (replace with the correct ID if necessary)
#define DMA_DEV_ID XPAR_AXIDMA_0_DEVICE_ID

// Buffer size
#define BUFFER_SIZE 30

// Declare DMA instance
XAxiDma AxiDma;

// Buffer declarations (aligned for DMA transfer)
u32 TxBuffer[BUFFER_SIZE] __attribute__ ((aligned(32)));
u32 RxBuffer[BUFFER_SIZE] __attribute__ ((aligned(32)));

const char msg[] = "Validating DMA subsystem";

// Function to initialize DMA
int init_dma() {
    XAxiDma_Config *CfgPtr;
    int status;

    // Look up the DMA configuration
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("No config found for %d\n", DMA_DEV_ID);
        return XST_FAILURE;
    }

    // Initialize the DMA
    status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (status != XST_SUCCESS) {
        xil_printf("DMA initialization failed\n");
        return XST_FAILURE;
    }

    // Check for Scatter Gather mode (we assume simple mode here)
    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Device configured as SG mode\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

// Function to run the DMA transfer
int run_dma_transfer() {
    int status;

    // Fill TxBuffer with sample data
    for (int i = 0; i < BUFFER_SIZE; i++) {
    	if(i < sizeof(msg) - 1)
    		TxBuffer[i] = msg[i];
    	else
    		TxBuffer[i] = '\0';
    }

    // Flush the buffers before using DMA
    Xil_DCacheFlushRange((UINTPTR)TxBuffer, BUFFER_SIZE * sizeof(u32));
    Xil_DCacheFlushRange((UINTPTR)RxBuffer, BUFFER_SIZE * sizeof(u32));

    // Start DMA transfer: Transmit data from TxBuffer to RxBuffer
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)RxBuffer, BUFFER_SIZE * sizeof(u32), XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        xil_printf("Failed RX transfer\n");
        return XST_FAILURE;
    }

    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)TxBuffer, BUFFER_SIZE * sizeof(u32), XAXIDMA_DMA_TO_DEVICE);
    if (status != XST_SUCCESS) {
        xil_printf("Failed TX transfer\n");
        return XST_FAILURE;
    }

    // Wait for both TX and RX to complete
    while ((XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) || (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE))) {
        // Waiting for transfer to complete
    }

    // Invalidate the cache to fetch updated data in RxBuffer
    Xil_DCacheInvalidateRange((UINTPTR)RxBuffer, BUFFER_SIZE * sizeof(u32));

    // Check data
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (RxBuffer[i] != TxBuffer[i]) {
            xil_printf("Data mismatch at index %d: %x != %x\n", i, RxBuffer[i], TxBuffer[i]);
            return XST_FAILURE;
        }
    }


    xil_printf("DMA transfer completed successfully\r\n");
    return XST_SUCCESS;
}

int main() {
    int status;
    init_platform();
    xil_printf("System is running\r\n");

    // Initialize DMA
    status = init_dma();
    if (status != XST_SUCCESS) {
        xil_printf("DMA init failed\n");
        return XST_FAILURE;
    }

    // Run DMA transfer and check result
    status = run_dma_transfer();
    if (status != XST_SUCCESS) {
        xil_printf("DMA transfer failed\n");
        return XST_FAILURE;
    }

    xil_printf("Application completed successfully\r\n");
    xil_printf("Received data from Rx Buffer:\r\n");

    for(int i = 0; i < BUFFER_SIZE; i++){
    	if(i < sizeof(msg) - 1){
    		xil_printf("%c", RxBuffer[i]);
    	}
    }
    cleanup_platform();
    return XST_SUCCESS;
}

