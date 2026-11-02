/**
 ******************************************************************************
 * @file    ai_io_buffers_ATON.h
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

#ifndef AI_IO_BUFFERS_ATON_H
#define AI_IO_BUFFERS_ATON_H

#include "ll_aton_rt_user_api.h"

/* ------ Utilities prototypes ------ */
void connect_input_buffers(const NN_Instance_TypeDef *nn_instance);
void connect_output_buffers(const NN_Instance_TypeDef *nn_instance);

#endif  /* AI_IO_BUFFERS_ATON_H */