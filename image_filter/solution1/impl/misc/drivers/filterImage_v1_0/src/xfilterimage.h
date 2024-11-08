// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XFILTERIMAGE_H
#define XFILTERIMAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xfilterimage_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Axi_cpu_BaseAddress;
} XFilterimage_Config;
#endif

typedef struct {
    u64 Axi_cpu_BaseAddress;
    u32 IsReady;
} XFilterimage;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XFilterimage_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XFilterimage_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XFilterimage_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XFilterimage_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XFilterimage_Initialize(XFilterimage *InstancePtr, u16 DeviceId);
XFilterimage_Config* XFilterimage_LookupConfig(u16 DeviceId);
int XFilterimage_CfgInitialize(XFilterimage *InstancePtr, XFilterimage_Config *ConfigPtr);
#else
int XFilterimage_Initialize(XFilterimage *InstancePtr, const char* InstanceName);
int XFilterimage_Release(XFilterimage *InstancePtr);
#endif

void XFilterimage_Start(XFilterimage *InstancePtr);
u32 XFilterimage_IsDone(XFilterimage *InstancePtr);
u32 XFilterimage_IsIdle(XFilterimage *InstancePtr);
u32 XFilterimage_IsReady(XFilterimage *InstancePtr);
void XFilterimage_EnableAutoRestart(XFilterimage *InstancePtr);
void XFilterimage_DisableAutoRestart(XFilterimage *InstancePtr);

u32 XFilterimage_Get_rowBelow_BaseAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowBelow_HighAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowBelow_TotalBytes(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowBelow_BitWidth(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowBelow_Depth(XFilterimage *InstancePtr);
u32 XFilterimage_Write_rowBelow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Read_rowBelow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Write_rowBelow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Read_rowBelow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Get_rowCenter_BaseAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowCenter_HighAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowCenter_TotalBytes(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowCenter_BitWidth(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowCenter_Depth(XFilterimage *InstancePtr);
u32 XFilterimage_Write_rowCenter_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Read_rowCenter_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Write_rowCenter_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Read_rowCenter_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Get_rowAbove_BaseAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowAbove_HighAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowAbove_TotalBytes(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowAbove_BitWidth(XFilterimage *InstancePtr);
u32 XFilterimage_Get_rowAbove_Depth(XFilterimage *InstancePtr);
u32 XFilterimage_Write_rowAbove_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Read_rowAbove_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Write_rowAbove_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Read_rowAbove_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Get_outputRow_BaseAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_outputRow_HighAddress(XFilterimage *InstancePtr);
u32 XFilterimage_Get_outputRow_TotalBytes(XFilterimage *InstancePtr);
u32 XFilterimage_Get_outputRow_BitWidth(XFilterimage *InstancePtr);
u32 XFilterimage_Get_outputRow_Depth(XFilterimage *InstancePtr);
u32 XFilterimage_Write_outputRow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Read_outputRow_Words(XFilterimage *InstancePtr, int offset, word_type *data, int length);
u32 XFilterimage_Write_outputRow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);
u32 XFilterimage_Read_outputRow_Bytes(XFilterimage *InstancePtr, int offset, char *data, int length);

void XFilterimage_InterruptGlobalEnable(XFilterimage *InstancePtr);
void XFilterimage_InterruptGlobalDisable(XFilterimage *InstancePtr);
void XFilterimage_InterruptEnable(XFilterimage *InstancePtr, u32 Mask);
void XFilterimage_InterruptDisable(XFilterimage *InstancePtr, u32 Mask);
void XFilterimage_InterruptClear(XFilterimage *InstancePtr, u32 Mask);
u32 XFilterimage_InterruptGetEnabled(XFilterimage *InstancePtr);
u32 XFilterimage_InterruptGetStatus(XFilterimage *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
