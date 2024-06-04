/*
 * [ PROJECT   ]  stm32g431_fw
 * [ License   ]  SAMJIN ELECTRONICS.,Co.Ltd
 * [ Author    ]  Copyright 2024 By HAG-SEONG KANG
 * [ E-MAIL    ]  hl5ugc@nate.com (82)10- 3841-9706
 * [ C  P  U   ]
 * [ Compller  ]  
 * [ Filename  ]  qbuffer.h
 * [ Version   ]  1.0
 * [ Created   ]  2024[4:38:14 PM ]
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
 *	*.This clause and the original author mark are prohibited from being 
 *	  modified or deleted. 
 *  
 *------------------------------------------------------------------
 * --------------------------------------------------------------------------
 * Author         Date       Comments on this revision
 * --------------------------------------------------------------------------
 * Hagseong Kang  Apr 25, 2024    First release of this file
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
#ifndef COMMON_CORE_QBUFFER_H_
#define COMMON_CORE_QBUFFER_H_

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
typedef struct
{
  uint32_t in;
  uint32_t out;
  uint32_t len;
  uint32_t size;

  uint8_t *p_buf;
} qbuffer_t;
// ---------------------------------------------------------------------------
// Define  Global Function prototypes.
// --------------------------------------------------------------------------- 
//
void     qbufferInit(void);
bool     qbufferCreate(qbuffer_t *p_node, uint8_t *p_buf, uint32_t length);
bool     qbufferCreateBySize(qbuffer_t *p_node, uint8_t *p_buf, uint32_t size, uint32_t length);
bool     qbufferWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length);
bool     qbufferRead(qbuffer_t *p_node, uint8_t * const p_data, uint32_t length);
uint8_t *qbufferPeekWrite(qbuffer_t *p_node);
uint8_t *qbufferPeekRead(qbuffer_t *p_node);
uint32_t qbufferAvailable(qbuffer_t *p_node);
uint32_t qbufferWriteAvailable(qbuffer_t *p_node);
void     qbufferFlush(qbuffer_t *p_node);

#endif 
#ifdef __cplusplus
}

#endif /* qbuffer.h End Of File !! Well Done */
