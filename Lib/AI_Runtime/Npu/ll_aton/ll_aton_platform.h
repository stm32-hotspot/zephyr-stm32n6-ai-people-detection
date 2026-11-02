/**
 ******************************************************************************
 * @file    ll_aton_platform.h
 * @author  STMicroelectronics
 * @brief   Header file for defining platform dependencies
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021-2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef __LL_ATON_PLATFORM_H
#define __LL_ATON_PLATFORM_H

#include <stdint.h>

#include "ll_aton_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Default timeout is 10s, but can be overridden */
#ifndef ATON_EPOCH_TIMEOUT_MS
#define ATON_EPOCH_TIMEOUT_MS 10000
#endif

/*** Platform dependent definitions & includes ***/

/* Bare metal Cortex-M NCSIM simulator platform*/
#if (LL_ATON_PLATFORM == LL_ATON_PLAT_NCSIM)
#define ATON_BASE             0xA0000000
#define SYSMEM1_BASE          0xA0080000
#define SYSMEM2_BASE          0xA00A0000
#define SYSMEM3_BASE          0xA00C0000
#define SYSMEM4_BASE          0xA00E0000
#define SYSMEM_LENGTH         0x00020000
#define DUMMY_MEM_BASE        0x40000000
#define DUMP_CONFIG_BASE_ADDR 0xA0108000
#define CDMA_BASE_ADDR        0xA0100000
#define OSPI2_BASE_ADDR       0x3D710000
#define OSPI1_BASE_ADDR       0x3D700000
#define OSPIIOM_BASE_ADDR     0x3D720000
#define OSPI_HSEL_SEL         0x3D83100C
#define OSPI1_MEM             0x60000000
#define OSPI2_MEM             0x80000000
#include "cm4ikmcu.h"

/* Imaging simulation environment */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_IMAGING_SIM)
#define ATON_BASE             0xA0000000
#define SYSMEM1_BASE          0xB0000000
#define SYSMEM2_BASE          0xB0040000
#define SYSMEM3_BASE          0xB0080000
#define SYSMEM_LENGTH         0x00040000
#define DUMMY_MEM_BASE        0x40000000
#define DUMP_CONFIG_BASE_ADDR 0xA0108000
#define CDMA_BASE_ADDR        0xA0100000
#define OSPI2_BASE_ADDR       0x3D710000
#define OSPI1_BASE_ADDR       0x3D700000
#define OSPIIOM_BASE_ADDR     0x3D720000
#define OSPI_HSEL_SEL         0x3D83100C
#define OSPI1_MEM             0x60000000
#define OSPI2_MEM             0x80000000
#include "cm4ikmcu.h"

#define ATON_STRENG_CID_CACHE_SET_ALLOC(a, b)     0
#define ATON_STRENG_CID_CACHE_SET_CACHEABLE(a, b) 0
#define ATON_STRENG_CID_CACHE_SET_CID(a, b)       0
#define ATON_STRENG_CID_CACHE_SET_LINESIZE(a, b)  0
#define ATON_STRENG_CID_CACHE_SET_PFETCH(a, b)    0

/* Bare metal Cortex-M STICE4 FPGA platform */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_STICE4)
#define ATON_PLAT_HAS_FFLUSH (1)
#define ATON_BASE            0x51100000

typedef int32_t IRQn_Type;

#include "stm32h7_map.h"

#include <core_cm7.h>

#define CDNN0_IRQn (123)

/* Linux based Xilinx ZC706 FPGA platform */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_ZC706) || (LL_ATON_PLATFORM == LL_ATON_PLAT_IWAVE) ||                          \
    (LL_ATON_PLATFORM == LL_ATON_PLAT_BITTWARE)
/* Data synchronization barrier */
#ifdef __ARM_ARCH
#define __FPU_PRESENT 1
#include <cmsis_compiler.h>
#define LL_ATON_DSB() __DSB()
#else
#include <immintrin.h>
#define LL_ATON_DSB() _mm_lfence()
#endif

#define ATON_PLAT_HAS_FFLUSH (1)
extern uint8_t *get_zynq_aton_base(void);
#define NVIC_EnableIRQ(x)
#define NVIC_DisableIRQ(x)
#define ATON_BASE          (uintptr_t)(get_zynq_aton_base())
/* Timer clock is 100MHz. Compute timeout accordingly. */
#define ATON_EPOCH_TIMEOUT (ATON_EPOCH_TIMEOUT_MS * 100 * 1000)

/* PC based Orlando Simulator platform */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_SWEMUL)
#ifndef ATON_BASE
#define ATON_BASE (0x00010000UL)
#endif
#define ATON_PLAT_HAS_FFLUSH (1)

#define __WFE()
#define __DSB()
#define NVIC_EnableIRQ(x)
#define NVIC_DisableIRQ(x)

/* Bare metal Cortex-M TLM simulator platform as used by MCD */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_TLM_MCD)
#define ATON_PLAT_HAS_FFLUSH (1)

#include "cnn1_top_mapping.h"

#define ATON_BASE             CNN1_TOP_BASEADDR
#define SYSMEM1_BASE          AXI_MEM0
#define SYSMEM2_BASE          AXI_MEM1
#define SYSMEM3_BASE          AXI_MEM2
#define SYSMEM4_BASE          AXI_MEM3
#define SYSMEM_LENGTH         0x00020000
#define DUMMY_MEM_BASE        AXI_FLASH
#define DUMP_CONFIG_BASE_ADDR ATON_BASE + 0x108000
#define CDMA_BASE_ADDR        ATON_BASE + 0x100000

#define __WFE() esw_sleep()
#define __DSB()
#define NVIC_EnableIRQ(x)
#define NVIC_DisableIRQ(x)

/* PC based Test Explorer application platform */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_TSTXPL)
#define ATON_PLAT_HAS_FFLUSH (1)

#define __WFE() (complete_dmas())
#define __DSB()
#define NVIC_EnableIRQ(x)
#define NVIC_DisableIRQ(x)
uintptr_t get_aton_base(void);
void complete_dmas(void);
#define ATON_BASE (get_aton_base())

#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_EC_TRACE)
#include "ll_aton_ec_trace.h"

#define ATON_PLAT_HAS_FFLUSH (1)

#define ATON_BASE (ec_trace_get_aton_base())

#define __WFE()
#define __DSB()
#define NVIC_EnableIRQ(x)
#define NVIC_DisableIRQ(x)

#define ATON_HIDE_REG_MACROS       1
#define ATON_GET_REG32(ADDR)       (ec_trace_unsupported(), 0)
#define ATON_SET_REG32(ADDR, DATA) ec_trace_write((uintptr_t)(ADDR), DATA)

#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_STM32N6)
#include "stm32n6xx.h"

#define CDNN0_IRQHandler NPU0_IRQHandler
#define CDNN1_IRQHandler NPU1_IRQHandler
#define CDNN2_IRQHandler NPU2_IRQHandler
#define CDNN3_IRQHandler NPU3_IRQHandler

#define CDNN0_IRQn NPU0_IRQn
#define CDNN1_IRQn NPU1_IRQn
#define CDNN2_IRQn NPU2_IRQn
#define CDNN3_IRQn NPU3_IRQn

#if defined(CPU_IN_SECURE_STATE)
#define ATON_BASE NPU_BASE_S
#else
#define ATON_BASE NPU_BASE_NS
#endif

#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_STM32H7P)
#include "stm32h7pxx.h"

#define CDNN0_IRQHandler NPU_IT0_IRQHandler
#define CDNN1_IRQHandler NPU_IT1_IRQHandler
#define CDNN2_IRQHandler NPU_IT2_IRQHandler
#define CDNN3_IRQHandler NPU_IT2_IRQHandler /* Only three interrupts here */

#define CDNN0_IRQn NPU_IT0_IRQn
#define CDNN1_IRQn NPU_IT1_IRQn
#define CDNN2_IRQn NPU_IT2_IRQn
#define CDNN3_IRQn NPU_IT2_IRQn
#define ATON_BASE  0x520E0000 /* Not yet defined in HAL */

/* Stellar P3E support: TODO use correct header files, when available */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_SR6P3E)
#include "irq.h"
#include "sr6p3e.h"

#define CDNN0_IRQn IRQ_NPU_0_0_INT_NUMBER /* To R52 Kite Cluster */
#define CDNN1_IRQn IRQ_NPU_0_1_INT_NUMBER /* To R52 Kite Cluster */
#define CDNN2_IRQn IRQ_NPU_0_2_INT_NUMBER /* To Cortex-M4 */
#define CDNN3_IRQn IRQ_NPU_0_3_INT_NUMBER /* To Cortex-M4 */

#define ATON_BASE NPU_BASE

#if defined(__GNUC__) && !defined(__ICCARM__)
#define __WFE() __asm volatile("wfe" ::: "memory")
#define __DSB() __asm volatile("dsb sy" ::: "memory")
#elif defined(__ICCARM__)
#define __WFE() __asm("wfe")
#define __DSB() __asm("dsb sy")
#else
#error "Compiler not supported for __WFE() and __DSB definitions"
#endif

#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_SR6P5E)

#warning "SR6P5E platform support is not yet implemented."

/* Unique stm32a98 ll_aton platform for both m85 and m55 cores */
#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_STM32A98)
#include "stm32a9xx.h"

#define CDNN0_IRQHandler NPU_IT0_IRQ_Handler
#define CDNN1_IRQHandler NPU_IT1_IRQ_Handler
#define CDNN2_IRQHandler NPU_IT2_IRQ_Handler
#define CDNN3_IRQHandler NPU_IT3_IRQ_Handler

#define CDNN0_IRQn NPU_IT0_IRQn
#define CDNN1_IRQn NPU_IT1_IRQn
#define CDNN2_IRQn NPU_IT2_IRQn
#define CDNN3_IRQn NPU_IT2_IRQn

#define ATON_BASE ((uintptr_t)NPU) /* NPU macro is defined in the stm32a98.h*/

#elif (LL_ATON_PLATFORM == LL_ATON_PLAT_USER_IMPL)
#include "ll_aton_user_platform_impl.h" /* file to be provided together with an implementation for the custom platform by the user */

#else
#error No target platform is specified. Please define macro `LL_ATON_PLATFORM`
#endif

/* Default polling timeout */
#ifndef ATON_EPOCH_TIMEOUT
#define ATON_EPOCH_TIMEOUT (ATON_EPOCH_TIMEOUT_MS * 1000)
#endif // !ATON_EPOCH_TIMEOUT

/* Default value for plstform supporting `fflush()` */
#ifndef ATON_PLAT_HAS_FFLUSH
#define ATON_PLAT_HAS_FFLUSH (0)
#endif // !ATON_PLAT_HAS_FFLUSH

/* Data synchronization barrier default definition */
#ifndef LL_ATON_DSB
#define LL_ATON_DSB() __DSB()
#endif

// may be included just here because of the platform dependent definitions (above all `ATON_BASE`)
#include "ATON.h"

/* Default macro for physical to virtual address translation (direct mapping) */
#ifndef ATON_LIB_PHYSICAL_TO_VIRTUAL_ADDR
#define ATON_LIB_PHYSICAL_TO_VIRTUAL_ADDR(address) (address)
#endif // !ATON_LIB_PHYSICAL_TO_VIRTUAL_ADDR

/* Default macro for virtual to physical address translation (direct mapping) */
#ifndef ATON_LIB_VIRTUAL_TO_PHYSICAL_ADDR
#define ATON_LIB_VIRTUAL_TO_PHYSICAL_ADDR(address) (address)
#endif // !ATON_LIB_VIRTUAL_TO_PHYSICAL_ADDR

/* Interrupt line(s) to use */
/* ATON_STD_IRQ_LINE MUST be within the range of available ATON interrupt lines, i.e. within [0 - 3] */
#ifndef ATON_STD_IRQ_LINE
#define ATON_STD_IRQ_LINE 0
#endif

#if (ATON_STD_IRQ_LINE != 0) && (ATON_STD_IRQ_LINE != 1) && (ATON_STD_IRQ_LINE != 2) && (ATON_STD_IRQ_LINE != 3)
#error invalid interrupt line number `ATON_STD_IRQ_LINE` (must be in range [0 - 3])
#endif // ATON_STD_IRQ_LINE

// Beyond macros may be defined just here
#define ATON_STD_IRQHandler                  LL_ATON_CONCAT3(CDNN, ATON_STD_IRQ_LINE, _IRQHandler)
#define ATON_STD_IRQn                        LL_ATON_CONCAT3(CDNN, ATON_STD_IRQ_LINE, _IRQn)
#define ATON_STD_INTANDMSK                   LL_ATON_CONCAT(INTANDMSK, ATON_STD_IRQ_LINE)
#define ATON_STD_INTORMSK                    LL_ATON_CONCAT(INTORMSK, ATON_STD_IRQ_LINE)
#define ATON_INTCTRL_STD_INTANDMSK_SET(DATA) ATON_INTCTRL_INTANDMSK_SET(0, ATON_STD_IRQ_LINE, DATA)
#define ATON_INTCTRL_STD_INTANDMSK_GET       ATON_INTCTRL_INTANDMSK_GET(0, ATON_STD_IRQ_LINE)
#define ATON_INTCTRL_STD_INTORMSK_SET(DATA)  ATON_INTCTRL_INTORMSK_SET(0, ATON_STD_IRQ_LINE, DATA)
#define ATON_INTCTRL_STD_INTORMSK_GET        ATON_INTCTRL_INTORMSK_GET(0, ATON_STD_IRQ_LINE)
#if (ATON_INTCTRL_INTS(0) > 32)
#define ATON_INTCTRL_STD_INTORMSK_H_SET(DATA)  ATON_INTCTRL_INTORMSK_H_SET(0, ATON_STD_IRQ_LINE, DATA)
#define ATON_INTCTRL_STD_INTANDMSK_H_SET(DATA) ATON_INTCTRL_INTANDMSK_H_SET(0, ATON_STD_IRQ_LINE, DATA)
#endif // (ATON_INTCTRL_INTS(0) > 32)

#ifdef __cplusplus
}
#endif

#endif // __LL_ATON_PLATFORM_H
