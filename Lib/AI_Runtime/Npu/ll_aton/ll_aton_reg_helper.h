/**
 ******************************************************************************
 * @file    ll_aton_reg_helper.h
 * @author  STMicroelectronics
 * @brief   Header file of ATON LL register helper macros.
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
#ifndef __LL_ATON_REG_HELPER_H
#define __LL_ATON_REG_HELPER_H

#include "ll_aton_config.h"

#include "ll_aton_platform.h"

// converts signed shift (negative value are left shift) to HW encoding Range 39:0. No shift 16
#ifndef ATON_SHIFT
#define ATON_SHIFT(x) ((x) + 16)
#endif // ATON_SHIFT

// Generic boilerplate for disabling and stopping pending transactions (aka "clearing") of an ATON unit
#define ATON_DISABLE_CLR(unitname, id)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    t = ATON_##unitname##_CTRL_DT;                                                                                     \
    t = ATON_##unitname##_CTRL_SET_EN(t, 0);                                                                           \
    t = ATON_##unitname##_CTRL_SET_CLR(t, 1);                                                                          \
    ATON_##unitname##_CTRL_SET(id, t);                                                                                 \
    /* wait for unit to terminate pipeline registers clearing  */                                                      \
    ATON_REG_POLL(unitname, id, CTRL, CLR, 0);                                                                         \
  } while (0)

// Generic boilerplate for disabling, stopping pending transactions (aka "clearing"), and clearing the configuration of
// an ATON unit
#define ATON_DISABLE_CONFCLR(unitname, id)                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    t = ATON_##unitname##_CTRL_DT;                                                                                     \
    t = ATON_##unitname##_CTRL_SET_CONFCLR(t, 1);                                                                      \
    ATON_##unitname##_CTRL_SET(id, t);                                                                                 \
    /* wait for unit to terminate clearing of configuration registers */                                               \
    ATON_REG_POLL(unitname, id, CTRL, CONFCLR, 0);                                                                     \
  } while (0)

#if (LL_ATON_PLATFORM != LL_ATON_PLAT_EC_TRACE)

#define ATON_REG_WRITE_RELOC(regaddr, base, offset)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    *(volatile uint32_t *)(uintptr_t)(regaddr) = (base) + (offset);                                                    \
  } while (0)

#define ATON_REG_WRITE(regaddr, val)                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    *(volatile uint32_t *)(uintptr_t)(regaddr) = (val);                                                                \
  } while (0)

#define ATON_REG_WRITE_FIELD_RANGE(unitname, id, reg, start, fsize, val)                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    uint32_t t = ATON_##unitname##_##reg##_GET(id);                                                                    \
    t = ATON_SET_FIELD32(t, start, fsize, val);                                                                        \
    ATON_##unitname##_##reg##_SET(id, t);                                                                              \
  } while (0)

#define ATON_REG_WRITE_FIELD(unitname, id, reg, field, val)                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    uint32_t t = ATON_##unitname##_##reg##_GET(id);                                                                    \
    t = ATON_##unitname##_##reg##_SET_##field(t, val);                                                                 \
    ATON_##unitname##_##reg##_SET(id, t);                                                                              \
  } while (0)

#define ATON_REG_POLL(unitname, id, reg, field, val)                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    while (ATON_##unitname##_##reg##_GET_##field(ATON_##unitname##_##reg##_GET(id)) != val)                            \
    {                                                                                                                  \
    }                                                                                                                  \
  } while (0)

#else // (LL_ATON_PLATFORM != LL_ATON_PLAT_EC_TRACE)

#define ATON_REG_WRITE_RELOC(regaddr, base, offset)                                                                    \
  ec_trace_write_reloc((uintptr_t)regaddr, (uint32_t)base, (uint32_t)offset)
#define ATON_REG_WRITE(regaddr, val) ec_trace_write((uintptr_t)regaddr, (uint32_t)val)
#define ATON_REG_WRITE_FIELD_RANGE(unitname, id, reg, start, fsize, val)                                               \
  ec_trace_reg_writefield(ec_trace_get_IP_id(ATON_##unitname##_BASE(id)),                                              \
                          ec_trace_get_REG_id(ATON_##unitname##_##reg##_OFFSET), start, fsize, val)
#define ATON_REG_WRITE_FIELD(unitname, id, reg, field, val)                                                            \
  ATON_REG_WRITE_FIELD_RANGE(unitname, id, reg, ATON_##unitname##_##reg##_##field##_LSB,                               \
                             ATON_##unitname##_##reg##_##field##_W, val)
#define ATON_REG_POLL(unitname, id, reg, field, val)                                                                   \
  ec_trace_reg_poll(ec_trace_get_IP_id(ATON_##unitname##_BASE(id)),                                                    \
                    ec_trace_get_REG_id(ATON_##unitname##_##reg##_OFFSET), ATON_##unitname##_##reg##_##field##_LSB,    \
                    ATON_##unitname##_##reg##_##field##_W, (uint32_t)val)

#endif //(LL_ATON_PLATFORM != LL_ATON_PLAT_EC_TRACE)

#endif //__LL_ATON_REG_HELPER_H
