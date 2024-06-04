/*
 * [ PROJECT   ]  5_stm32g431_UART
 * [ License   ]  SAMJIN ELECTRONICS.,Co.Ltd
 * [ Author    ]  Copyright 2024 By HAG-SEONG KANG
 * [ E-MAIL    ]  hl5ugc@nate.com (82)10- 3841-9706
 * [ C  P  U   ]
 * [ Compller  ]  
 * [ Filename  ]  word.h
 * [ Version   ]  1.0
 * [ Created   ]  2024[12:53:33 PM ]
 * --------------------------------------------------------------------------
 * Revision History :
 * ------------------------------------------------------------------
 *
 *
 *  *.The use of this source code shall be deemed to have been 
 *    tacitly agreed by the user.
 *  *.This source code is freely available for personal learning 
 *    and research and development.
 *  *.In the case of secondary authoring or redistribution using this source, 
 *    it is essential The company name of SAMJIN ELECTRONICS must be specified.
 *  *.Do not sell or for-profit this source code.
 *  *.This clause and the original author mark are prohibited from being
 *    modified or deleted.
 *  
 *------------------------------------------------------------------
 * --------------------------------------------------------------------------
 * Author         Date       Comments on this revision
 * --------------------------------------------------------------------------
 * Hagseong Kang  May 30, 2024    First release of this file
 * --------------------------------------------------------------------------
 * Additional Notes:
 * **************************************************************************
 */
 /**
 * @brief 
 * @param  
 * @return  
 */


/* Mutiple includes protection */
#ifndef COMMON_CORE_WORD_H_
#define COMMON_CORE_WORD_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes */
#include "def.h"


 
// ---------------------------------------------------------------------------
// Define Global define
// --------------------------------------------------------------------------- 
// 

// ---------------------------------------------------------------------------
// Define typedef
// --------------------------------------------------------------------------- 
// 
// ---------------------------------------------------------------------------
// Define  Global Function prototypes.
// --------------------------------------------------------------------------- 
//
/** @ingroup util_word
    Return low word of a 32-bit integer.

    @param uint32_t ww (0x00000000..0xFFFFFFFF)
    @return low word of input (0x0000..0xFFFF)
*/
static inline uint16_t lowWord(uint32_t ww)
{
  return (uint16_t) ((ww) & 0xFFFFU);
}


/** @ingroup util_word
    Return high word of a 32-bit integer.

    @param uint32_t ww (0x00000000..0xFFFFFFFF)
    @return high word of input (0x0000..0xFFFF)
*/
static inline uint16_t highWord(uint32_t ww)
{
  return (uint16_t) ((ww) >> 16U);
}


/* *************************************************/
static inline uint8_t lowByte(uint16_t ww)
{
  return (uint8_t) ((ww) & 0x00FFU);
}


static inline uint8_t highByte(uint16_t ww)
{
  return (uint8_t) ((ww) >> 8U);
}

static inline uint16_t word(uint8_t H_Byte,uint8_t L_Byte)
{
  uint16_t word;
  word = (uint16_t)(H_Byte<<8U);
  word = word + L_Byte;
  return word;
}

#define bitSet(value, bit)  ((value) |= (1UL << (bit)))
#define bitClear(value, bit)  ((value) &= ~(1UL << (bit)))

#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitRead(value, bit)  (((value) >> (bit)) & 0x01)

#ifdef __cplusplus
}
#endif
#endif /* word.h End Of File !! Well Done */
