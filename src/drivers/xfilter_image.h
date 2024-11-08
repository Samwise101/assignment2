// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XFILTER_IMAGE_H
#define XFILTER_IMAGE_H

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
#include "xfilter_image_hw.h"

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
} XFilter_image_Config;
#endif

typedef struct {
    u64 Axi_cpu_BaseAddress;
    u32 IsReady;
} XFilter_image;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XFilter_image_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XFilter_image_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XFilter_image_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XFilter_image_ReadReg(BaseAddress, RegOffset) \
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
int XFilter_image_Initialize(XFilter_image *InstancePtr, u16 DeviceId);
XFilter_image_Config* XFilter_image_LookupConfig(u16 DeviceId);
int XFilter_image_CfgInitialize(XFilter_image *InstancePtr, XFilter_image_Config *ConfigPtr);
#else
int XFilter_image_Initialize(XFilter_image *InstancePtr, const char* InstanceName);
int XFilter_image_Release(XFilter_image *InstancePtr);
#endif

void XFilter_image_Start(XFilter_image *InstancePtr);
u32 XFilter_image_IsDone(XFilter_image *InstancePtr);
u32 XFilter_image_IsIdle(XFilter_image *InstancePtr);
u32 XFilter_image_IsReady(XFilter_image *InstancePtr);
void XFilter_image_EnableAutoRestart(XFilter_image *InstancePtr);
void XFilter_image_DisableAutoRestart(XFilter_image *InstancePtr);

u32 XFilter_image_Get_rowBelow_BaseAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowBelow_HighAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowBelow_TotalBytes(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowBelow_BitWidth(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowBelow_Depth(XFilter_image *InstancePtr);
u32 XFilter_image_Write_rowBelow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Read_rowBelow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Write_rowBelow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Read_rowBelow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Get_rowCenter_BaseAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowCenter_HighAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowCenter_TotalBytes(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowCenter_BitWidth(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowCenter_Depth(XFilter_image *InstancePtr);
u32 XFilter_image_Write_rowCenter_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Read_rowCenter_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Write_rowCenter_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Read_rowCenter_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Get_rowAbove_BaseAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowAbove_HighAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowAbove_TotalBytes(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowAbove_BitWidth(XFilter_image *InstancePtr);
u32 XFilter_image_Get_rowAbove_Depth(XFilter_image *InstancePtr);
u32 XFilter_image_Write_rowAbove_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Read_rowAbove_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Write_rowAbove_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Read_rowAbove_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Get_outputRow_BaseAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_outputRow_HighAddress(XFilter_image *InstancePtr);
u32 XFilter_image_Get_outputRow_TotalBytes(XFilter_image *InstancePtr);
u32 XFilter_image_Get_outputRow_BitWidth(XFilter_image *InstancePtr);
u32 XFilter_image_Get_outputRow_Depth(XFilter_image *InstancePtr);
u32 XFilter_image_Write_outputRow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Read_outputRow_Words(XFilter_image *InstancePtr, int offset, word_type *data, int length);
u32 XFilter_image_Write_outputRow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);
u32 XFilter_image_Read_outputRow_Bytes(XFilter_image *InstancePtr, int offset, char *data, int length);

void XFilter_image_InterruptGlobalEnable(XFilter_image *InstancePtr);
void XFilter_image_InterruptGlobalDisable(XFilter_image *InstancePtr);
void XFilter_image_InterruptEnable(XFilter_image *InstancePtr, u32 Mask);
void XFilter_image_InterruptDisable(XFilter_image *InstancePtr, u32 Mask);
void XFilter_image_InterruptClear(XFilter_image *InstancePtr, u32 Mask);
u32 XFilter_image_InterruptGetEnabled(XFilter_image *InstancePtr);
u32 XFilter_image_InterruptGetStatus(XFilter_image *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
