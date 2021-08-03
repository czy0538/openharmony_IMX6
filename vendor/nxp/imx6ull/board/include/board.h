/*
 * Copyright (c) 2020 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "menuconfig.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __cplusplus */

/* physical memory base and size */
#define DDR_MEM_ADDR 0x80000000 //内存基地址与内存大小
#define DDR_MEM_SIZE 0x18000000 /* reseved 128M for ramfs and framebuffer */

#define DDR_RAMFS_ADDR (DDR_MEM_ADDR + DDR_MEM_SIZE)
#define DDR_RAMFS_SIZE 0x7C00000 /* 64+60M */

#define LCD_FB_BASE (DDR_RAMFS_ADDR + DDR_RAMFS_SIZE)
#define LCD_FB_SIZE 0x400000 /* 4M */

/* Peripheral register address base and size */
#define PERIPH_PMM_BASE 0x00a00000 //gic基地址
#define PERIPH_PMM_SIZE 0x02300000 //gic大小

#define KERNEL_VADDR_BASE 0x40000000
#define KERNEL_VADDR_SIZE DDR_MEM_SIZE

#define SYS_MEM_BASE DDR_MEM_ADDR
#define SYS_MEM_SIZE_DEFAULT 0x2000000
#define SYS_MEM_END (SYS_MEM_BASE + SYS_MEM_SIZE_DEFAULT)

#define EXC_INTERACT_MEM_SIZE 0x100000

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif
