// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xfilterimage.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XFilterimage_CfgInitialize(XFilterimage *InstancePtr, XFilterimage_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axi_cpu_BaseAddress = ConfigPtr->Axi_cpu_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XFilterimage_Start(XFilterimage *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL) & 0x80;
    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL, Data | 0x01);
}

u32 XFilterimage_IsDone(XFilterimage *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XFilterimage_IsIdle(XFilterimage *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XFilterimage_IsReady(XFilterimage *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XFilterimage_EnableAutoRestart(XFilterimage *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL, 0x80);
}

void XFilterimage_DisableAutoRestart(XFilterimage *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_AP_CTRL, 0);
}

u32 XFilterimage_Get_rowBelow_BaseAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE);
}

u32 XFilterimage_Get_rowBelow_HighAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH);
}

u32 XFilterimage_Get_rowBelow_TotalBytes(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1);
}

u32 XFilterimage_Get_rowBelow_BitWidth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_WIDTH_ROWBELOW;
}

u32 XFilterimage_Get_rowBelow_Depth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_DEPTH_ROWBELOW;
}

u32 XFilterimage_Write_rowBelow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowBelow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilterimage_Write_rowBelow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowBelow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWBELOW_BASE + offset + i);
    }
    return length;
}

u32 XFilterimage_Get_rowCenter_BaseAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE);
}

u32 XFilterimage_Get_rowCenter_HighAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH);
}

u32 XFilterimage_Get_rowCenter_TotalBytes(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1);
}

u32 XFilterimage_Get_rowCenter_BitWidth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_WIDTH_ROWCENTER;
}

u32 XFilterimage_Get_rowCenter_Depth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_DEPTH_ROWCENTER;
}

u32 XFilterimage_Write_rowCenter_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowCenter_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilterimage_Write_rowCenter_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowCenter_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWCENTER_BASE + offset + i);
    }
    return length;
}

u32 XFilterimage_Get_rowAbove_BaseAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE);
}

u32 XFilterimage_Get_rowAbove_HighAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH);
}

u32 XFilterimage_Get_rowAbove_TotalBytes(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1);
}

u32 XFilterimage_Get_rowAbove_BitWidth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_WIDTH_ROWABOVE;
}

u32 XFilterimage_Get_rowAbove_Depth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_DEPTH_ROWABOVE;
}

u32 XFilterimage_Write_rowAbove_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowAbove_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilterimage_Write_rowAbove_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_rowAbove_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_ROWABOVE_BASE + offset + i);
    }
    return length;
}

u32 XFilterimage_Get_outputRow_BaseAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE);
}

u32 XFilterimage_Get_outputRow_HighAddress(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH);
}

u32 XFilterimage_Get_outputRow_TotalBytes(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1);
}

u32 XFilterimage_Get_outputRow_BitWidth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_WIDTH_OUTPUTROW;
}

u32 XFilterimage_Get_outputRow_Depth(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFILTERIMAGE_AXI_CPU_DEPTH_OUTPUTROW;
}

u32 XFilterimage_Write_outputRow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_outputRow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + (offset + i)*4);
    }
    return length;
}

u32 XFilterimage_Write_outputRow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XFilterimage_Read_outputRow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH - XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XFILTERIMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE + offset + i);
    }
    return length;
}

void XFilterimage_InterruptGlobalEnable(XFilterimage *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_GIE, 1);
}

void XFilterimage_InterruptGlobalDisable(XFilterimage *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_GIE, 0);
}

void XFilterimage_InterruptEnable(XFilterimage *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_IER);
    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_IER, Register | Mask);
}

void XFilterimage_InterruptDisable(XFilterimage *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_IER);
    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_IER, Register & (~Mask));
}

void XFilterimage_InterruptClear(XFilterimage *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFilterimage_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_ISR, Mask);
}

u32 XFilterimage_InterruptGetEnabled(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_IER);
}

u32 XFilterimage_InterruptGetStatus(XFilterimage *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFilterimage_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XFILTERIMAGE_AXI_CPU_ADDR_ISR);
}

