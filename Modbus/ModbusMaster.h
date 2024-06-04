/*
 * [ PROJECT   ]  6_stm32g431_UART_DMA
 * [ License   ]  SAMJIN ELECTRONICS.,Co.Ltd
 * [ Author    ]  Copyright 2024 By HAG-SEONG KANG
 * [ E-MAIL    ]  hl5ugc@nate.com (82)10- 3841-9706
 * [ C  P  U   ]
 * [ Compller  ]  
 * [ Filename  ]  ModbusMaster.h
 * [ Version   ]  1.0
 * [ Created   ]  2024[4:02:28 PM ]
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
#ifndef COMMON_CORE_MODBUS_MODBUSMASTER_H_
#define COMMON_CORE_MODBUS_MODBUSMASTER_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes */
#include "def.h"


 
// ---------------------------------------------------------------------------
// Define Global define
// --------------------------------------------------------------------------- 
// 
#define MBS_MAX_CH            1U // USE_MBS_MAX_CH
// ---------------------------------------------------------------------------
// Define typedef
// --------------------------------------------------------------------------- 
// 
/* 1     ~ 10000   coil                Read-Write
 * 10001 ~ 20000   Inputs              Read Only
 * 30001 ~ 40000   Inputs Ananlog Reg  Read Only
 * 40001 ~ 50000   Holding Reg         Read-Write
 */

#define MBIllegalFunction           0x01U
#define MBIllegalDataAddress        0x02U
#define MBIllegalDataValue          0x03U
#define MBSlaveDeviceFailure        0x04U

#define MBSuccess                   0x00U
#define MBFalse                     0xFFU
#define MBInvalidSlaveID            0xE0U
#define MBInvalidFunction           0xE1U
#define MBResponseTimedOut          0xE2U
#define MBInvalidCRC                0xE3U
 // Modbus function codes for bit access
#define MBReadCoils                  0x01U ///< Modbus function 0x01 Read Coils
#define MBReadDiscreteInputs         0x02U ///< Modbus function 0x02 Read Discrete Inputs
#define MBWriteSingleCoil            0x05U ///< Modbus function 0x05 Write Single Coil
#define MBWriteMultipleCoils         0x0FU ///< Modbus function 0x0F Write Multiple Coils

// Modbus function codes for 16 bit access
#define MBReadHoldingRegisters       0x03U ///< Modbus function 0x03 Read Holding Registers
#define MBReadInputRegisters         0x04U ///< Modbus function 0x04 Read Input Registers
#define MBWriteSingleRegister        0x06U ///< Modbus function 0x06 Write Single Register
#define MBWriteMultipleRegisters     0x10U ///< Modbus function 0x10 Write Multiple Registers
#define MBMaskWriteRegister          0x16U ///< Modbus function 0x16 Mask Write Register
#define MBReadWriteMultipleRegisters 0x17U ///< Modbus function 0x17 Read Write Multiple Registers
//
typedef enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1,

}mb_com_state_t;

// ---------------------------------------------------------------------------
// Define  Global Function prototypes.
// --------------------------------------------------------------------------- 
//
bool MBMInit(void);
bool MBMDeInit(void);
bool MBMIsInit(void);
bool MBMOpen(uint8_t u8Ch,uint8_t u8SlaveID);
bool IsMBMOpen(uint8_t u8Ch);
bool MBM_Memory_MAP_Init(uint8_t u8Ch,uint16_t *pCoilBuff,uint16_t *pInputBuff,
                        uint16_t *pInputRbuff, uint16_t *pInputHRbuff);
//
uint8_t MBMreadCoils(uint8_t u8Ch,uint16_t u16ReadAddress, uint16_t u16BitQty);
uint8_t MBreadDiscreteInputs(uint8_t u8Ch,uint16_t u16ReadAddress, uint16_t u16BitQty);
uint8_t MBMreadInputRegisters(uint8_t u8Ch,uint16_t u16ReadAddress,uint8_t u16ReadQty);
uint8_t MBMreadHoldingRegisters(uint8_t u8Ch,uint16_t u16ReadAddress,uint16_t u16ReadQty);
//
uint8_t MBMwriteCoil(uint8_t u8Ch,uint16_t u16WriteAddress, uint8_t u8State);
uint8_t MBMwriteRegister(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16WriteValue);
uint8_t MBMwriteCoils(uint8_t u8Ch,uint16_t u16WriteAddress, uint16_t u16BitQty);
uint8_t MBMwriteRegisters(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16WriteQty);
uint8_t MBMmaskWriteRegister(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16AndMask, uint16_t u16OrMask);
uint8_t MBMreadWritegisters(uint8_t u8Ch,uint16_t u16ReadAddress,uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty);


//
uint8_t MBM_Response_Check(const uint8_t * pu8Buff,uint8_t u8Len);
#ifdef __cplusplus
}
#endif
#endif /* ModbusMaster.h End Of File !! Well Done */
