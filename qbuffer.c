/*
 * [ PROJECT   ]  stm32g431_fw
 * [ License   ]  SAMJIN ELECTRONICS.,Co.Ltd
 * [ Author    ]  Copyright 2024 By HAG-SEONG KANG
 * [ E-MAIL    ]  hl5ugc@nate.com (82)10- 3841-9706
 * [ C  P  U   ]
 * [ Compller  ]  
 * [ Filename  ]  qbuffer.c
 * [ Version   ]  1.0
 * [ Created   ]  2024[4:39:26 PM ]
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



/* Define Includes */
#include "qbuffer.h"

// ---------------------------------------------------------------------------
//  Define Macros .
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
//  Define Constants.
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
//  Define Private variables.
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
// Define Private function prototype.
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
// Define Public function definitions. 
// ---------------------------------------------------------------------------
//
void qbufferInit(void)
{

}

bool qbufferCreate(qbuffer_t *p_node, uint8_t *p_buf, uint32_t length)
{
  bool ret = true;

  p_node->in    = 0U;
  p_node->out   = 0U;
  p_node->len   = length;
  p_node->size  = 1U;
  p_node->p_buf = p_buf;

  return ret;
}


bool qbufferWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length)
{
  bool bRet = true;
  uint32_t next_in = 0UL ;

  for(uint32_t i=0; i<length; i++)
  {
    // Over Write Check
    next_in = (p_node->in + 1U) % p_node->len ;
    if(next_in != p_node->out)
    {
      p_node->p_buf[p_node->in] = p_data[i];
      p_node->in = (p_node->in + 1U) % p_node->len ;
    }
    else
    {
      bRet = false ;
    }
  }
  /*
  bool bRet = true;
  uint32_t next_in = 0x00U;
  uint8_t * p_buf;

  for (int i=0; i<length; i++)
  {
    next_in = (p_node->in + 1) % p_node->len;

    if (next_in != p_node->out)
    {
      if (p_node->p_buf != NULL && p_data != NULL)
      {
        //uint8_t *p_buf;
        p_buf = &p_node->p_buf[p_node->in*p_node->size];
        for (int j=0; j<p_node->size; j++)
        {
          p_buf[j] = p_data[j];
        }
        p_data += p_node->size;
      }
      p_node->in = next_in;
    }
    else
    {
      bRet = false;
      break;
    }
  }
   */
  return bRet;
}

bool qbufferRead(qbuffer_t *p_node, uint8_t * const p_data, uint32_t length)
{
  bool bRet = false;

  if (p_node->p_buf != NULL && p_data != NULL)
  {
    for (uint32_t i=0; i<length; i++)
    {
      if(p_node->in != p_node->out)
      {
        p_data[i] = p_node->p_buf[p_node->out];
        p_node->out = (p_node->out + 1U) % p_node->len ;
      }
      else
      {
        bRet = false;
        break ;
      }
    }
    bRet = true ;
  }
  /*
  uint8_t * p_buf;

  for (int i=0; i<length; i++)
  {
    if (p_node->p_buf != NULL && p_data != NULL)
    {
      //uint8_t * const p_buf;
      p_buf = &p_node->p_buf[p_node->out*p_node->size];
      for (int j=0; j<p_node->size; j++)
      {
        p_data[j] = p_buf[j];
      }
      p_data += p_node->size;
    }

    if (p_node->out != p_node->in)
    {
      p_node->out = (p_node->out + 1) % p_node->len;
    }
    else
    {
      bRet = false;
      break;
    }
  }
  */
  return bRet;
}
uint8_t *qbufferPeekWrite(qbuffer_t *p_node)
{
  return &p_node->p_buf[p_node->in*p_node->size];
}

uint8_t *qbufferPeekRead(qbuffer_t *p_node)
{
  return &p_node->p_buf[p_node->out*p_node->size];
}


uint32_t qbufferAvailable(qbuffer_t *p_node)
{
  uint32_t u32Ret = 0x00U;


  u32Ret = (p_node->len + p_node->in - p_node->out) % p_node->len;

  return u32Ret;
}

uint32_t qbufferWriteAvailable(qbuffer_t *p_node)
{
  uint32_t u32Ret = 0x00U;

  u32Ret = p_node->len - qbufferAvailable(p_node);

  return u32Ret;
}
void qbufferFlush(qbuffer_t *p_node)
{
  p_node->in  = 0U;
  p_node->out = 0U;
}
// ---------------------------------------------------------------------------
// Define private function definitions.  
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
//  Define Callbacks definitions.
// ---------------------------------------------------------------------------
//
 
 
/* qbuffer.c End Of File !! Well Done */
