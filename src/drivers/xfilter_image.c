// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xfilter_image.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XFilter_image_CfgInitialize(XFilter_image *InstancePtr, XFilter_image_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axi_cpu_BaseAddress = ConfigPtr->Axi_cpu_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XFilter_image_Start(XFilter_image *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL) & 0x80;
    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL, Data | 0x01);
}

u32 XFilter_image_IsDone(XFilter_image *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XFilter_image_IsIdle(XFilter_image *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XFilter_image_IsReady(XFilter_image *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XFilter_image_EnableAutoRestart(XFilter_image *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL, 0x80);
}

void XFilter_image_DisableAutoRestart(XFilter_image *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL, 0);
}

u32 XFilter_image_Get_rowBelow_BaseAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE);
}

u32 XFilter_image_Get_rowBelow_HighAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH);
}

u32 XFilter_image_Get_rowBelow_TotalBytes(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1);
}

u32 XFilter_image_Get_rowBelow_BitWidth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_WIDTH_ROWBELOW;
}

u32 XFilter_image_Get_rowBelow_Depth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_DEPTH_ROWBELOW;
}

u32 XFilter_image_Write_rowBelow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowBelow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilter_image_Write_rowBelow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowBelow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + offset + i);
    }
    return length;
}

u32 XFilter_image_Get_rowCenter_BaseAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE);
}

u32 XFilter_image_Get_rowCenter_HighAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH);
}

u32 XFilter_image_Get_rowCenter_TotalBytes(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1);
}

u32 XFilter_image_Get_rowCenter_BitWidth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_WIDTH_ROWCENTER;
}

u32 XFilter_image_Get_rowCenter_Depth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_DEPTH_ROWCENTER;
}

u32 XFilter_image_Write_rowCenter_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowCenter_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilter_image_Write_rowCenter_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowCenter_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + offset + i);
    }
    return length;
}

u32 XFilter_image_Get_rowAbove_BaseAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE);
}

u32 XFilter_image_Get_rowAbove_HighAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH);
}

u32 XFilter_image_Get_rowAbove_TotalBytes(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1);
}

u32 XFilter_image_Get_rowAbove_BitWidth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_WIDTH_ROWABOVE;
}

u32 XFilter_image_Get_rowAbove_Depth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_DEPTH_ROWABOVE;
}

u32 XFilter_image_Write_rowAbove_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowAbove_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilter_image_Write_rowAbove_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_rowAbove_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + offset + i);
    }
    return length;
}

u32 XFilter_image_Get_outputRow_BaseAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE);
}

u32 XFilter_image_Get_outputRow_HighAddress(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH);
}

u32 XFilter_image_Get_outputRow_TotalBytes(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1);
}

u32 XFilter_image_Get_outputRow_BitWidth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_WIDTH_OUTPUTROW;
}

u32 XFilter_image_Get_outputRow_Depth(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTER_IMAGE_AXI_CPU_DEPTH_OUTPUTROW;
}

u32 XFilter_image_Write_outputRow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_outputRow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilter_image_Write_outputRow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilter_image_Read_outputRow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + offset + i);
    }
    return length;
}

void XFilter_image_InterruptGlobalEnable(XFilter_image *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_GIE, 1);
}

void XFilter_image_InterruptGlobalDisable(XFilter_image *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_GIE, 0);
}

void XFilter_image_InterruptEnable(XFilter_image *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_IER);
    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_IER, Register | Mask);
}

void XFilter_image_InterruptDisable(XFilter_image *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_IER);
    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_IER, Register & (~Mask));
}

void XFilter_image_InterruptClear(XFilter_image *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilter_image_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_ISR, Mask);
}

u32 XFilter_image_InterruptGetEnabled(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_IER);
}

u32 XFilter_image_InterruptGetStatus(XFilter_image *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFilter_image_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTER_IMAGE_AXI_CPU_ADDR_ISR);
}

