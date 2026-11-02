/**
 ******************************************************************************
 * @file    ll_aton_debug.h
 * @author  STMicroelectronics
 * @brief   Header file of ATON LL low level lib debug module.
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

#ifndef __LL_ATON_DEBUG_H
#define __LL_ATON_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "ll_aton_NN_interface.h"
#include "ll_aton_attributes.h"
#include "ll_aton_platform.h"

  /**
   * @brief enumerator type for dump_xxx functions
   */
  enum
  {
    MODE_RAW = 0x1,     // Dump buffer with data as is present in memory (i.e. flattened)
    MODE_ONNX = 0x2,    // Dump buffer considering shape info and in ONNX canonical form (consider batch info)
    MODE_INBITS = 0x4,  // Dump elements as raw bits (as integers)
    MODE_INFLOAT = 0x8, // Dump elements interpreting Qmn format info (as floats)

    MODE_RAW_INBITS = MODE_RAW | MODE_INBITS,
    MODE_ONNX_INBITS = MODE_ONNX | MODE_INBITS,
    MODE_RAW_INFLOAT = MODE_RAW | MODE_INFLOAT,
    MODE_ONNX_INFLOAT = MODE_ONNX | MODE_INFLOAT,

    MODE_8BIT = MODE_RAW_INBITS,  // Deprecated defines
    MODE_16BIT = MODE_RAW_INBITS, // Deprecated defines
    MODE_QMN = MODE_ONNX_INFLOAT, // Deprecated defines
  };

  /**
   * @brief enumerator type identifying buffer group (in,out,internal) for dump_xxx functions
   */
  enum
  {
    BUFF_OUT = 1,
    BUFF_IN = 2,
    BUFF_INT = 4,
  };

  /**
   * @brief  convert
   * @param  addr
   * @retval pointer
   */
  void *LL_ATON_physical_to_virtual(uintptr_t addr);

  /**
   * @brief  retrieve pointer to a given graph in/output buffer
   * @param  bufname tensor name
   * @param  in buffer group identifier
   * @param  _len pointer to integer to be set with buffer length
   * @param  _bits pointer to integer to be set with buffer bitwidth
   * @param  nn_interface pointer to neural network interface structure
   * @retval pointer to buffer or null of not found
   */
  void *get_buffer(const char *bufname, int in, unsigned int *_len, unsigned int *_bits,
                   const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  retrieve buffer info structure for a given graph in/output buffer
   * @param  bufname tensor name
   * @param  in buffer group identifier
   * @param  ret pointer to LL_Buffer_InfoTypeDef to be used to store buffer info
   * @param  nn_interface pointer to neural network interface structure
   * @retval 1 if buffer is found 0 otherwise
   */
  int get_buffer_info(const char *bufname, int in, LL_Buffer_InfoTypeDef *ret,
                      const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  fill all buffers according to buffer group identifier with a given value
   * @param  in group identifier
   * @param  val filler value
   * @param  nn_interface pointer to neural network interface structure
   * @retval none
   */
  void set_all_buffers(int in, unsigned int val, const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  dump a buffer with a given mode
   * @param  mode dump mode specifier
   * @param  bufname tensor name
   * @param  in buffer group identifier
   * @param  nn_interface pointer to neural network interface structure
   * @retval none
   */
  void dump_buffer(int mode, const char *bufname, int in, const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  dump all buffers according to buffer group identifier with a given mode
   * @param  mode dump mode specifier
   * @param  in buffer groups identifiers
   * @param  nn_interface pointer to neural network interface structure
   * @retval none
   */
  void dump_all_buffers(int mode, int in, const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  dump all internal buffers whose epoch matches (to be used in epoch hooks, or if scheduling epoch
   * explicitly)
   * @param  mode dump mode specifier
   * @param  epoch epoch number
   * @param  nn_interface pointer to neural network interface structure
   * @retval none
   */
  void dump_epoch_buffers(int mode, int epoch, const NN_Interface_TypeDef *nn_interface);

  /**
   * @brief  Return the total length of network parameters
   * @param  nn_interface pointer to neural network interface structure
   * @retval number of buffer info structure available
   */
  unsigned int get_params_len(const NN_Interface_TypeDef *nn_interface);

  /**
   * Deprecated methods, implemented as wrappers around new methods
   *
   * @{
   */

  LL_ATON_DEPRECATED("use dump_buffer method")
  static inline void dump_tensor_Qmn(const char *bufname, int in, const NN_Interface_TypeDef *nn_interface)
  {
    dump_buffer(MODE_ONNX_INFLOAT, bufname, in, nn_interface);
  }

  LL_ATON_DEPRECATED("use dump_buffer method")
  static inline void dump_output_16bit(const char *bufname, int in, const NN_Interface_TypeDef *nn_interface)
  {
    dump_buffer(MODE_RAW_INBITS, bufname, in, nn_interface);
  }

  LL_ATON_DEPRECATED("use dump_buffer method")
  static inline void dump_output_8bit(const char *bufname, int in, const NN_Interface_TypeDef *nn_interface)
  {
    dump_buffer(MODE_RAW_INBITS, bufname, in, nn_interface);
  }

  LL_ATON_DEPRECATED("use dump_epoch_buffers method")
  static inline void dump_all_epoch_buffers(int mode, int epoch, int in, const NN_Interface_TypeDef *nn_interface)
  {
    LL_ATON_LIB_UNUSED(in);
    dump_epoch_buffers(mode, epoch, nn_interface);
  }

  // @}

#ifdef __cplusplus
}
#endif

#endif
