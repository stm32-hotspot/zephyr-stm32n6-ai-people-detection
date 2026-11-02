/**
 ******************************************************************************
 * @file    ai_io_buffers_ATON.c
 * @author  STMicroelectronics
 * @brief   AI Validation application (entry points) - IO buffers allocation
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 - 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "ai_io_buffers_ATON.h"
#include "ll_aton_NN_interface.h"
#include "ll_aton_platform.h"           // To include core_cmxx.h->armvxm_cachel1.h
#include "network.h"

#include "ll_aton_rt_user_api.h"


#if !defined(UNUSED)
#define UNUSED(X_) (void)(X_)      /* To avoid gcc/g++ warnings */
#endif /* UNUSED */

/* ------ Alignment macros ------ */
// Macro to ensure each declared array has a size that fills D$-lines
//      (to prevent issues when doing cache maintenance -namely: invalidate only- on those buffers)
#define ROUND_TO_DCACHE_SIZE(sz)  ( ((sz) + ((__SCB_DCACHE_LINE_SIZE) - 1)) / (__SCB_DCACHE_LINE_SIZE) * (__SCB_DCACHE_LINE_SIZE) )

/* ------ Allocated inputs ------ */

#if (defined(LL_ATON_NETWORK_USER_ALLOCATED_INPUTS) && (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS > 0))
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 1)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 1 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 2)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 2 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 3)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 3 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 4)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 4 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 5)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 5 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 6)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
static int8_t data_in_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_6_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 6 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 7)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
static int8_t data_in_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_6_ALIGNMENT));
static int8_t data_in_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_7_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 7 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 8)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
static int8_t data_in_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_6_ALIGNMENT));
static int8_t data_in_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_7_ALIGNMENT));
static int8_t data_in_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_8_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 8 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 9)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
static int8_t data_in_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_6_ALIGNMENT));
static int8_t data_in_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_7_ALIGNMENT));
static int8_t data_in_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_8_ALIGNMENT));
static int8_t data_in_9[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_9_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_9_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 9 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 10)
static int8_t data_in_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_1_ALIGNMENT));
static int8_t data_in_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_2_ALIGNMENT));
static int8_t data_in_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_3_ALIGNMENT));
static int8_t data_in_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_4_ALIGNMENT));
static int8_t data_in_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_5_ALIGNMENT));
static int8_t data_in_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_6_ALIGNMENT));
static int8_t data_in_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_7_ALIGNMENT));
static int8_t data_in_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_8_ALIGNMENT));
static int8_t data_in_9[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_9_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_9_ALIGNMENT));
static int8_t data_in_10[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_IN_10_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_IN_10_ALIGNMENT));
#endif   /* USER_ALLOCATED_INPUTS == 10 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS > 10)
#error "LL_ATON_NETWORK_USER_ALLOCATED_INPUTS is too large"
#endif
#endif   /* LL_ATON_NETWORK_USER_ALLOCATED_INPUTS */

/* ------ Allocated outputs ------ */

#if (defined(LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS) && (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS > 0))
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 1)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 1 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 2)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 2 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 3)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 3 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 4)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 4 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 5)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 5 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 6)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
static int8_t data_out_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_6_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 6 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 7)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
static int8_t data_out_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_6_ALIGNMENT));
static int8_t data_out_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_7_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 7 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 8)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
static int8_t data_out_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_6_ALIGNMENT));
static int8_t data_out_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_7_ALIGNMENT));
static int8_t data_out_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_8_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 8 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 9)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
static int8_t data_out_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_6_ALIGNMENT));
static int8_t data_out_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_7_ALIGNMENT));
static int8_t data_out_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_8_ALIGNMENT));
static int8_t data_out_9[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_9_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_9_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 9 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 10)
static int8_t data_out_1[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_1_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_1_ALIGNMENT));
static int8_t data_out_2[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_2_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_2_ALIGNMENT));
static int8_t data_out_3[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_3_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_3_ALIGNMENT));
static int8_t data_out_4[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_4_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_4_ALIGNMENT));
static int8_t data_out_5[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_5_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_5_ALIGNMENT));
static int8_t data_out_6[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_6_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_6_ALIGNMENT));
static int8_t data_out_7[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_7_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_7_ALIGNMENT));
static int8_t data_out_8[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_8_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_8_ALIGNMENT));
static int8_t data_out_9[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_9_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_9_ALIGNMENT));
static int8_t data_out_10[ROUND_TO_DCACHE_SIZE(LL_ATON_NETWORK_OUT_10_SIZE_BYTES)] __attribute__((aligned LL_ATON_NETWORK_OUT_10_ALIGNMENT));
#endif   /* USER_ALLOCATED_OUTPUTS == 10 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS > 10)
#error "LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS is too large"
#endif
#endif   /* LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS */

                
/* ------ Utilities ------ */
                

void connect_input_buffers(const NN_Instance_TypeDef *nn_instance)
{
#if (defined(LL_ATON_NETWORK_USER_ALLOCATED_INPUTS) && (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS > 0))
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 1)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 1 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 2)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 2 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 3)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 3 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 4)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 4 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 5)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 5 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 6)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 5, data_in_6, LL_ATON_NETWORK_IN_6_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 6 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 7)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 5, data_in_6, LL_ATON_NETWORK_IN_6_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 6, data_in_7, LL_ATON_NETWORK_IN_7_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 7 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 8)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 5, data_in_6, LL_ATON_NETWORK_IN_6_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 6, data_in_7, LL_ATON_NETWORK_IN_7_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 7, data_in_8, LL_ATON_NETWORK_IN_8_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 8 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 9)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 5, data_in_6, LL_ATON_NETWORK_IN_6_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 6, data_in_7, LL_ATON_NETWORK_IN_7_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 7, data_in_8, LL_ATON_NETWORK_IN_8_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 8, data_in_9, LL_ATON_NETWORK_IN_9_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 9 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_INPUTS == 10)
LL_ATON_Set_User_Input_Buffer(nn_instance, 0, data_in_1, LL_ATON_NETWORK_IN_1_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 1, data_in_2, LL_ATON_NETWORK_IN_2_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 2, data_in_3, LL_ATON_NETWORK_IN_3_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 3, data_in_4, LL_ATON_NETWORK_IN_4_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 4, data_in_5, LL_ATON_NETWORK_IN_5_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 5, data_in_6, LL_ATON_NETWORK_IN_6_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 6, data_in_7, LL_ATON_NETWORK_IN_7_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 7, data_in_8, LL_ATON_NETWORK_IN_8_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 8, data_in_9, LL_ATON_NETWORK_IN_9_SIZE_BYTES);
LL_ATON_Set_User_Input_Buffer(nn_instance, 9, data_in_10, LL_ATON_NETWORK_IN_10_SIZE_BYTES);
#endif   /* USER_ALLOCATED_INPUTS == 10 */
#else   /* LL_ATON_NETWORK_USER_ALLOCATED_INPUTS */
UNUSED(nn_instance);
#endif
}

void connect_output_buffers(const NN_Instance_TypeDef *nn_instance)
{
#if (defined(LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS) && (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS > 0))
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 1)
LL_ATON_Set_User_Output_Buffer(nn_instance , 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 1 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 2)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 2 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 3)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 3 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 4)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 4 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 5)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 5 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 6)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 5, data_out_6, LL_ATON_NETWORK_OUT_6_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 6 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 7)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 5, data_out_6, LL_ATON_NETWORK_OUT_6_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 6, data_out_7, LL_ATON_NETWORK_OUT_7_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 7 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 8)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 5, data_out_6, LL_ATON_NETWORK_OUT_6_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 6, data_out_7, LL_ATON_NETWORK_OUT_7_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 7, data_out_8, LL_ATON_NETWORK_OUT_8_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 8 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 9)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 5, data_out_6, LL_ATON_NETWORK_OUT_6_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 6, data_out_7, LL_ATON_NETWORK_OUT_7_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 7, data_out_8, LL_ATON_NETWORK_OUT_8_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 8, data_out_9, LL_ATON_NETWORK_OUT_9_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 9 */
#if (LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS == 10)
LL_ATON_Set_User_Output_Buffer(nn_instance, 0, data_out_1, LL_ATON_NETWORK_OUT_1_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 1, data_out_2, LL_ATON_NETWORK_OUT_2_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 2, data_out_3, LL_ATON_NETWORK_OUT_3_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 3, data_out_4, LL_ATON_NETWORK_OUT_4_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 4, data_out_5, LL_ATON_NETWORK_OUT_5_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 5, data_out_6, LL_ATON_NETWORK_OUT_6_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 6, data_out_7, LL_ATON_NETWORK_OUT_7_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 7, data_out_8, LL_ATON_NETWORK_OUT_8_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 8, data_out_9, LL_ATON_NETWORK_OUT_9_SIZE_BYTES);
LL_ATON_Set_User_Output_Buffer(nn_instance, 9, data_out_10, LL_ATON_NETWORK_OUT_10_SIZE_BYTES);
#endif   /* USER_ALLOCATED_OUTPUTS == 10 */
#else   /* LL_ATON_NETWORK_USER_ALLOCATED_OUTPUTS */
UNUSED(nn_instance);
#endif
}