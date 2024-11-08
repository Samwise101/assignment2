// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xfilter_image.h"

extern XFilter_image_Config XFilter_image_ConfigTable[];

XFilter_image_Config *XFilter_image_LookupConfig(u16 DeviceId) {
	XFilter_image_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XFILTER_IMAGE_NUM_INSTANCES; Index++) {
		if (XFilter_image_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XFilter_image_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XFilter_image_Initialize(XFilter_image *InstancePtr, u16 DeviceId) {
	XFilter_image_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XFilter_image_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XFilter_image_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

