/**
 ******************************************************************************
 * @file    ll_aton_cipher.h
 * @author  STMicroelectronics
 * @brief   Header file of ATON LL module.
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

#ifndef __LL_ATON_CIPHER_H
#define __LL_ATON_CIPHER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  struct LL_Streng_EncryptionTypedef;

  typedef enum
  {
    CYPHER_CACHE_NONE = 0,
    CYPHER_CACHE_SRC,
    CYPHER_CACHE_DST,
  } CypherCacheSourceMask;

  typedef enum
  {
    CYPHER_DISABLE_MASK = 0,
    CYPHER_SRC_MASK,
    CYPHER_DST_MASK,
  } CypherEnableMask;

  /**
   * @brief Cyphering configuration structure for DmaCypher function
   */

  typedef struct
  {
    uint32_t srcAdd;                       /**< Transfer source address */
    uint32_t dstAdd;                       /**< Transfer destination address */
    uint32_t len;                          /**< Transfer size */
    CypherCacheSourceMask cypherCacheMask; /**< Cache usage mask:
                                            *     0-no cache
                                            *     1-cache source
                                            *     2-cache destination */
    CypherEnableMask cypherEnableMask;     /**< Cyphering channel mask:
                                            *     0-no cypher
                                            *     1-cypher source
                                            *     2-cypher destination */
    uint64_t busIfKeyLsb;                  /**< Bus interface LSB Key */
    uint64_t busIfKeyMsb;                  /**< Bus interface MSB Key */
  } LL_Cypher_InitTypeDef;

#define CYPHER_SRC_STRENG_ID 0       /**< Stream engine used for source data in Dma/Cypher function */
#define CYPHER_DST_STRENG_ID 1       /**< Stream engine used for destination data in Dma/Cypher function */
#define CYPHER_CACHE_SIZE    0x40000 /**< N6 cache size */

  int LL_DmaCypherInit(LL_Cypher_InitTypeDef *cypherInfo);

  /**
   * @brief Extended Cyphering configuration structure for DmaCypher function
   */

  typedef struct
  {
    uint32_t srcAdd;                       /**< Transfer source address */
    uint32_t dstAdd;                       /**< Transfer destination address */
    uint32_t transferSize;                 /**< Transfer size */
    CypherCacheSourceMask cypherCacheMask; /**< Cache usage mask:
                                            *     0-no cache
                                            *     1-cache source
                                            *     2-cache destination */
    CypherEnableMask cypherEnableMask;     /**< Cyphering channel mask:
                                            *     0-no cypher
                                            *     1-cypher source
                                            *     2-cypher destination */
    uint64_t encryptionId;

    uint32_t frameSize;

    uint8_t keySel;
    uint8_t roundNumber;
    uint16_t incrementStep;

  } LL_ExtendedCypher_InitTypeDef;

  void LL_ATON_DecryptAndCopy(const struct LL_Streng_EncryptionTypedef *encr_params, uint64_t *dst, const uint64_t *src,
                              uint32_t size);

#ifdef __cplusplus
}
#endif

#endif //__LL_ATON_CIPHER_H
