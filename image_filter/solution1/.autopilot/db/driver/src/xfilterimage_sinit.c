// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xfilterimage.h"

extern XFilterimage_Config XFilterimage_ConfigTable[];

XFilterimage_Config *XFilterimage_LookupConfig(u16 DeviceId) {
	XFilterimage_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XFILTERIMAGE_NUM_INSTANCES; Index++) {
		if (XFilterimage_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XFilterimage_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XFilterimage_Initialize(XFilterimage *InstancePtr, u16 DeviceId) {
	XFilterimage_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XFilterimage_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XFilterimage_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

