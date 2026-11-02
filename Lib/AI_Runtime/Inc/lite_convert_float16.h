/**
  ******************************************************************************
  * @file    lite_convert_float16.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform lite convert kernel datatypes
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef LITE_CONVERT_FLOAT16_H
#define LITE_CONVERT_FLOAT16_H


#include "ai_lite_interface.h"

/******************************************************************************/
/*  Forward Functions Section                                                 */
/******************************************************************************/
#ifdef AI_FLOAT16_TYPE_SUPPORTED

LITE_API_ENTRY
void forward_lite_node_convert_if16of16(
  const ai_f16 *p_in,
  ai_f16 *p_out,
  const ai_u32 size);


 LITE_API_ENTRY
void forward_lite_node_convert_is1of16(
  const ai_pbits *p_in,
  ai_f16 *p_out,
  const ai_i32 n_channels,
  const ai_i32 n_pixels,
  const ai_f16 *n_values);


LITE_API_ENTRY
void forward_lite_node_convert_if16os1(
  const ai_f16 *p_in,
  ai_pbits *p_out,
  const ai_i32 n_channels,
  const ai_i32 n_pixels,
  const ai_i8 zp,
  const ai_i8 pad);


LITE_API_ENTRY
void forward_lite_node_convert_integer_if16os8(
  const ai_f16 *p_in,
  ai_i8 *p_out,
  const ai_u32 size,
  const ai_float out_scale,
  const ai_i8 out_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_integer_if16ou8(
  const ai_f16 *p_in,
  ai_u8 *p_out,
  const ai_u32 size,
  const ai_float out_scale,
  const ai_u8 out_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_integer_is8of16(
  const ai_i8 *p_in,
  ai_f16 *p_out,
  const ai_u32 size,
  const ai_float in_scale,
  const ai_i8 in_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_integer_iu8of16(
  const ai_u8 *p_in,
  ai_f16 *p_out,
  const ai_u32 size,
  const ai_float in_scale,
  const ai_u8 in_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_if16os16(
  const ai_f16 *p_in,
  ai_i16 *p_out,
  const ai_u32 size,
  const ai_float out_scale,
  const ai_i16 out_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_if16ou16(
  const ai_f16 *p_in,
  ai_u16 *p_out,
  const ai_u32 size,
  const ai_float out_scale,
  const ai_u16 out_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_is16of16(
  const ai_i16 *p_in,
  ai_f16 *p_out,
  const ai_u32 size,
  const ai_float in_scale,
  const ai_i16 in_zeropoint);


LITE_API_ENTRY
void forward_lite_node_convert_iu16of16(
  const ai_u16 *p_in,
  ai_f16 *p_out,
  const ai_u32 size,
  const ai_float in_scale,
  const ai_u16 in_zeropoint);

LITE_API_ENTRY
void forward_lite_node_convert_if32of16(
  const ai_float *p_in,
  ai_f16 *p_out,
  const ai_u32 size);

LITE_API_ENTRY
void forward_lite_node_convert_if16of32(
  const ai_f16 *p_in,
  ai_float *p_out,
  const ai_u32 size);

#endif   /* AI_FLOAT16_TYPE_SUPPORTED */

#endif    /*LITE_CONVERT_FLOAT16_H*/
