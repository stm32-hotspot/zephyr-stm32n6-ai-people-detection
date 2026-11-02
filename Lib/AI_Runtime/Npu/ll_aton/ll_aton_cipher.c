/**
 ******************************************************************************
 * @file    ll_aton_cipher.c
 * @author  STMicroelectronics
 * @brief   ATON LL module driver.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024-2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "ll_aton_cipher.h"
#include "ll_aton.h"
#include "ll_aton_platform.h"

#ifdef ATON_STRENG_NUM

#include "ll_aton_runtime.h"

/**
 * @brief  Handles Dma/Cypher data transfer
 * @param  cypherInfo: transfer parameters (addresses, keys, ... )
 * @retval Error code (0: og - -1: ko)
 * @note   WARNING: this is an utility function and its usage is not compatible with the ATON runtime
 * @note   The function uses stream engines 0 and 1
 * @note   It uses default (0) values for encryption id and round (12)
 */
int LL_DmaCypherInit(LL_Cypher_InitTypeDef *cypherInfo)
{
  uint32_t limit;
  uint32_t lastAdd;

  if (cypherInfo->cypherCacheMask != 0 && cypherInfo->len > CYPHER_CACHE_SIZE)
  {
    return (-1);
  }

  LL_ATON_Init();

  /* Use stream engine 0 as source channel */

  lastAdd = cypherInfo->srcAdd + cypherInfo->len;
  limit = (uint32_t)cypherInfo->len + (8 - (lastAdd & 7));
  limit += (8 - (limit & 7));

  LL_Streng_TensorInitTypeDef dma_in = {.dir = 0,
                                        .addr_base.i = (uint32_t)cypherInfo->srcAdd,
                                        .offset_start = 0,
                                        .offset_end = (uint32_t)cypherInfo->len,
                                        .offset_limit = limit,
                                        .raw = 1,
                                        .frame_tot_cnt = 1,
                                        .nbits_in = 8,
                                        .nbits_out = 8,
                                        .nbits_unsigned = 0};

  /* Use stream engine 0 as destination channel */

  lastAdd = cypherInfo->dstAdd + cypherInfo->len;
  limit = (uint32_t)cypherInfo->len + (8 - (lastAdd & 7));
  limit += (8 - (limit & 7));

  LL_Streng_TensorInitTypeDef dma_out = {.dir = 1,
                                         .addr_base.i = (uint32_t)cypherInfo->dstAdd,
                                         .offset_start = 0,
                                         .offset_end = (uint32_t)cypherInfo->len,
                                         .offset_limit = limit,
                                         .raw = 1,
                                         .frame_tot_cnt = 1,
                                         .nbits_in = 8,
                                         .nbits_out = 8,
                                         .nbits_unsigned = 0};

  if (0 != (cypherInfo->cypherCacheMask & CYPHER_CACHE_SRC))
  {
    dma_in.cacheable = 1;
    dma_in.cache_allocate = 1;
  }
  else
  {
    dma_in.cacheable = 0;
    dma_in.cache_allocate = 0;
  }

  if (0 != (cypherInfo->cypherCacheMask & CYPHER_CACHE_DST))
  {
    dma_out.cacheable = 1;
    dma_out.cache_allocate = 1;
  }
  else
  {
    dma_out.cacheable = 0;
    dma_out.cache_allocate = 0;
  }

  /* Configure stream switch */

  const LL_Switch_InitTypeDef __ll_switch_init = {LL_Switch_Init_Dest() = ATONN_DSTPORT(STRSWITCH, 0, STRENG, 1, 0),
                                                  LL_Switch_Init_Source(0) = ATONN_SRCPORT(STRSWITCH, 0, STRENG, 0, 0),
                                                  LL_Switch_Init_Context(0) = 1, LL_Switch_Init_Frames(0) = 0};

  /* Initialize stream engines */

  const LL_ATON_EnableUnits_InitTypeDef dma_units[] = {{{STRENG, 1}}, {{STRENG, 0}}};

  if (cypherInfo->cypherEnableMask != 0)
  {
    if ((cypherInfo->cypherEnableMask & CYPHER_SRC_MASK) != 0)
    {
      /* Enable encryption on input stream engine */
      dma_in.cipher_en = 1;
    }
    else if ((cypherInfo->cypherEnableMask & CYPHER_DST_MASK) != 0)
    {
      /* Enable encryption on output stream engine */

      dma_out.cipher_en = 1;
    }
    else
    {
      return (-1);
    }

    /* Set encryption keys into Bus Interface */

    LL_Busif_SetKeys(0, 0, cypherInfo->busIfKeyLsb, cypherInfo->busIfKeyMsb);
  }

  LL_Streng_TensorInit(CYPHER_SRC_STRENG_ID, &dma_in, 1);
  LL_Streng_TensorInit(CYPHER_DST_STRENG_ID, &dma_out, 1);

  /* Enable stream switch */

  LL_Switch_Init(&__ll_switch_init, 1);

  /* Start */

  LL_ATON_EnableUnits_Init(dma_units, 2);

  /* Wait for end of transfer */

  const uint32_t dma_wait_mask = 1 << CYPHER_DST_STRENG_ID; // Wait for destination stream engine

  LL_Streng_Wait(dma_wait_mask);

  /* Disable and deinit */

  LL_ATON_DisableUnits_Init(dma_units, 1);

  LL_Switch_Deinit(&__ll_switch_init, 1);

  return (0);
}

/**
 * @brief  Handles Dma/Cypher data transfer with extended parameters
 * @param  extCypherInfo: transfer parameters (addresses, keys, ... )
 * @retval Error code (0: ok - -1: ko)
 * @note   WARNING: this is an utility function and its usage is not compatible with the ATON runtime
 * @note   The function uses stream engines 0 and 1
 */

int LL_ExtDmaCypherInit(LL_ExtendedCypher_InitTypeDef *extCypherInfo)
{
  if (extCypherInfo->cypherCacheMask != 0 && extCypherInfo->transferSize > CYPHER_CACHE_SIZE)
  {
    return (-1);
  } /* endif */

  /* Add frames check */

  uint32_t frameNumber;
  uint32_t frameSize;
  uint32_t limit;
  uint32_t lastAdd;

  if (extCypherInfo->incrementStep != 0)
  {
    if ((extCypherInfo->transferSize % extCypherInfo->frameSize) != 0)
    {
      return (-1);
    } /* endif */

    frameSize = extCypherInfo->frameSize;
    frameNumber = extCypherInfo->transferSize / extCypherInfo->frameSize;
  }
  else
  {
    frameSize = extCypherInfo->transferSize;
    frameNumber = 1;
  } /* endif */

  lastAdd = extCypherInfo->srcAdd + extCypherInfo->transferSize;
  limit = (uint32_t)extCypherInfo->transferSize + (8 - (lastAdd & 7));
  limit += (8 - (limit & 7));

  /* Use stream engine 0 as source channel */

  LL_Streng_TensorInitTypeDef dma_in = {.dir = 0,
                                        .addr_base.i = (uint32_t)extCypherInfo->srcAdd,
                                        .offset_start = 0,
                                        .offset_end = frameSize,
                                        .offset_limit = limit,
                                        .raw = 1,
                                        .frame_tot_cnt = frameNumber,
                                        .frame_offset = frameSize,
                                        .nbits_in = 8,
                                        .nbits_out = 8,
                                        .nbits_unsigned = 0};

  /* Use stream engine 1 as destination channel:
     same configuration except the direction, the address and the limit */

  LL_Streng_TensorInitTypeDef dma_out = dma_in;

  lastAdd = extCypherInfo->dstAdd + extCypherInfo->transferSize;
  limit = (uint32_t)extCypherInfo->transferSize + (8 - (lastAdd & 7));
  limit += (8 - (limit & 7));

  dma_out.dir = 1;
  dma_out.addr_base.i = (uint32_t)extCypherInfo->dstAdd;
  dma_out.offset_limit = limit;

  if (0 != (extCypherInfo->cypherCacheMask & CYPHER_CACHE_SRC))
  {
    dma_in.cacheable = 1;
    dma_in.cache_allocate = 1;
  }
  else
  {
    dma_in.cacheable = 0;
    dma_in.cache_allocate = 0;
  } /* endif */

  if (0 != (extCypherInfo->cypherCacheMask & CYPHER_CACHE_DST))
  {
    dma_out.cacheable = 1;
    dma_out.cache_allocate = 1;
  }
  else
  {
    dma_out.cacheable = 0;
    dma_out.cache_allocate = 0;
  } /* endif */

  /* Configure stream switch */

  const LL_Switch_InitTypeDef __ll_switch_init = {LL_Switch_Init_Dest() = ATONN_DSTPORT(STRSWITCH, 0, STRENG, 1, 0),
                                                  LL_Switch_Init_Source(0) = ATONN_SRCPORT(STRSWITCH, 0, STRENG, 0, 0),
                                                  LL_Switch_Init_Context(0) = 1, LL_Switch_Init_Frames(0) = 0};

  /* Initialize stream engines */

  const LL_ATON_EnableUnits_InitTypeDef dma_units[] = {{{STRENG, 1}}, {{STRENG, 0}}};

  if (extCypherInfo->cypherEnableMask != 0)
  {
    if ((extCypherInfo->cypherEnableMask & CYPHER_SRC_MASK) != 0)
    {
      /* Enable encryption on input stream engine */
      dma_in.cipher_en = 1;
    }
    else if ((extCypherInfo->cypherEnableMask & CYPHER_DST_MASK) != 0)
    {
      /* Enable encryption on output stream engine */

      dma_out.cipher_en = 1;
    }
    else
    {
      return (-1);
    } /* endif */

    /* Check parameters */

    if (extCypherInfo->keySel > 1 || extCypherInfo->roundNumber > 1)
    {
      return (-1);
    } /* endif */

  } /* endif */

  /* Stream engines configuration */

  LL_Streng_TensorInit(CYPHER_SRC_STRENG_ID, &dma_in, 1);
  LL_Streng_TensorInit(CYPHER_DST_STRENG_ID, &dma_out, 1);

  /* Encryption setup */

  if (dma_in.cipher_en == 1 || dma_out.cipher_en == 1)
  {
    LL_Streng_EncryptionTypedef dmaCypherInfo;

    dmaCypherInfo.enable = 1;
    dmaCypherInfo.encryption_id = extCypherInfo->encryptionId;
    dmaCypherInfo.key_sel = extCypherInfo->keySel;
    dmaCypherInfo.rounds = extCypherInfo->roundNumber;
    dmaCypherInfo.increment = extCypherInfo->incrementStep;

    if (dma_in.cipher_en == 1)
    {
      LL_Streng_EncryptionInit(CYPHER_SRC_STRENG_ID, &dmaCypherInfo);
    }
    else
    {
      LL_Streng_EncryptionInit(CYPHER_DST_STRENG_ID, &dmaCypherInfo);
    } /* endif */

  } /* endif */

  /* Enable stream switch */

  LL_Switch_Init(&__ll_switch_init, 1);

  /* Start */

  LL_ATON_EnableUnits_Init(dma_units, 2);

  /* Set wait mask for destination engine */

  const uint32_t dma_wait_mask = 1 << CYPHER_DST_STRENG_ID;

  /* Wait for end of transfer */

  LL_Streng_Wait(dma_wait_mask);

  /* Clear interrupt at stream engine level */

  uint32_t strengIrqs = ATON_STRENG_IRQ_GET(CYPHER_DST_STRENG_ID);
  ATON_STRENG_IRQ_SET(
      CYPHER_DST_STRENG_ID,
      strengIrqs); /* Acknowledge ATON interrupt source (i.e. stream engine #i) - could be more fine grain */

  /* Data Synchronization Barrier */

  LL_ATON_DSB();

  /* Clear interrupt at interrupt controller level */

  ATON_INTCTRL_INTCLR_SET(0, 1 << CYPHER_DST_STRENG_ID);

  LL_ATON_DSB();

  /* Disable and deinit */

  LL_ATON_DisableUnits_Init(dma_units, 2);

  LL_Switch_Deinit(&__ll_switch_init, 1);

  return (0);
}

/**
 * @brief  Handles Dma/Cypher data transfer with extended parameters
 * @param  encr_params = encryption parameters
 * @param  dst = transfer destination address
 * @param  src = transfer source address
 * @param  size = bytes to transfer
 * @retval None
 * @note   WARNING: this is an utility function and its usage is not compatible with the ATON runtime
 */

void LL_ATON_DecryptAndCopy(const LL_Streng_EncryptionTypedef *encr_params, uint64_t *dst, const uint64_t *src,
                            uint32_t size)
{
  LL_ExtendedCypher_InitTypeDef extCypherInfo;

  /* Fill the data structure */

  extCypherInfo.srcAdd = (uint32_t)(uintptr_t)src;
  extCypherInfo.dstAdd = (uint32_t)(uintptr_t)dst;
  extCypherInfo.transferSize = size;
  extCypherInfo.cypherCacheMask = CYPHER_CACHE_NONE; /* Do not use cache */
  extCypherInfo.cypherEnableMask = CYPHER_SRC_MASK;  /* Decrypt using source address */

  /* Set default values */

  extCypherInfo.encryptionId = encr_params->encryption_id;
  extCypherInfo.frameSize = size;
  extCypherInfo.keySel = encr_params->key_sel;
  extCypherInfo.roundNumber = encr_params->rounds;
  extCypherInfo.incrementStep = encr_params->increment;

  /* Decrypt and copy in a fake instance environment */

  __ll_set_aton_owner((NN_Instance_TypeDef *)0xFFFFFFFF);
  LL_ExtDmaCypherInit(&extCypherInfo);
  __ll_clear_aton_owner((NN_Instance_TypeDef *)0xFFFFFFFF, false);
}

#endif // ATON_STRENG_NUM
