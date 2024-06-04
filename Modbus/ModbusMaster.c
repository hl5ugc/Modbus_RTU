/*
 * [ PROJECT   ]  6_stm32g431_UART_DMA
 * [ License   ]  SAMJIN ELECTRONICS.,Co.Ltd
 * [ Author    ]  Copyright 2024 By HAG-SEONG KANG
 * [ E-MAIL    ]  hl5ugc@nate.com (82)10- 3841-9706
 * [ C  P  U   ]
 * [ Compller  ]  
 * [ Filename  ]  ModbusMaster.c
 * [ Version   ]  1.0
 * [ Created   ]  2024[4:02:45 PM ]
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



/* Define Includes */
#include "ModbusMaster.h"
#include "ModbusCrc.h"
#include "qbuffer.h"

#include "UART.h" // imsi
// ---------------------------------------------------------------------------
//  Define Macros .
// ---------------------------------------------------------------------------
//
#define MDM_QUERY_BUF_LENGTH    32U
// ---------------------------------------------------------------------------
//  Define TypeDef & Constants.
// ---------------------------------------------------------------------------
//
/*
typedef struct _modbus_mapping_t {
    int nb_bits;
    int start_bits;
    int nb_input_bits;
    int start_input_bits;
    int nb_input_registers;
    int start_input_registers;
    int nb_registers;
    int start_registers;
    uint8_t *tab_bits;
    uint8_t *tab_input_bits;
    uint16_t *tab_input_registers;
    uint16_t *tab_registers;
} modbus_mapping_t;
*/
typedef struct
{
  uint8_t  rdptr ;
  uint8_t  wrptr ;
  uint8_t  len   ;
  uint16_t u16Buff[MDM_QUERY_BUF_LENGTH];
} MBM_Q_t;

typedef struct
{
  bool     isOpen ;
  uint8_t  u8SlaveID ;
  uint8_t  u8Function ;
  uint16_t u16ReadAdd  ;
  uint16_t u16ReadQty ;
  uint16_t u16WriteAdd ;
  uint16_t u16WriteQty ;
  MBM_Q_t  u16TXBuff;
  uint8_t  u8Count ;
  //  PLC Memroy
  uint16_t *  CoilBuff ;
  uint16_t    CoilOffset ;
  uint16_t *  InputBuff ;
  uint16_t    InputOffset ;
  uint16_t *  InputRbuff ;
  uint16_t    InputROffset ;
  uint16_t *  InputHRbuff ;
  uint16_t    InputHROffset ;
  //
  uint8_t  u8Status ;
} MBM_t;

// ---------------------------------------------------------------------------
//  Define Private variables.
// ---------------------------------------------------------------------------
/*
static uint16_t CoilBuff[5];      // Output Bit 5 X 16 = 30 Bits Read-Write
static uint16_t DInputBuff[5];    // INput Bit 5 X 16 = 30 Bits  Read Only
static uint16_t ResBuff[50];      // Analog Input (30001 ~ )       Read Only
static uint16_t HResBuff[50];     // Holding Reg (40001 ~)       Read-Write
*/

static bool isInit = false  ;
static MBM_t MBM_table[MBS_MAX_CH];

/*
uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers
uint16_t _u16ReadAddress;                                    ///< slave register from which to read
uint16_t _u16ReadQty;                                        ///< quantity of words to read
uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
uint16_t _u16WriteAddress;                                   ///< slave register to which to write
uint16_t _u16WriteQty;                                       ///< quantity of words to write
uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
uint16_t* txBuffer; // from Wire.h -- need to clean this up Rx
uint8_t _u8TransmitBufferIndex;
uint16_t u16TransmitBufferLength;
uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
uint8_t _u8ResponseBufferIndex;
uint8_t _u8ResponseBufferLength;
*/
// ---------------------------------------------------------------------------
// Define Private function prototype.
// ---------------------------------------------------------------------------
//
static uint8_t MBM_QReadSend(MBM_t *pMB_node);
static uint8_t MBM_FWriteSend(MBM_t *pMB_node);
// ---------------------------------------------------------------------------
// Define Callback function prototype.
// ---------------------------------------------------------------------------
//

// ---------------------------------------------------------------------------
// Define Public function definitions. 
// ---------------------------------------------------------------------------
//
bool MBMInit(void)
{
  bool bRet = true ;

  for(int i=0; i<MBS_MAX_CH; i++)
  {
    MBM_table[i].isOpen       = false ;
    MBM_table[i].u8SlaveID    = 0x00U ;
    MBM_table[i].u8Function   = 0U ;
    MBM_table[i].u16ReadAdd   = 0U ;
    MBM_table[i].u16ReadQty   = 0U ;
    MBM_table[i].u16WriteAdd  = 0U ;
    MBM_table[i].u16WriteQty  = 0U ;
    MBM_table[i].u8Count      = 0U ;
    MBM_table[i].u8Status     = COM_WAITING ;
    MBM_table[i].CoilBuff     = NULL ;
    MBM_table[i].CoilOffset   = 0U ;
    MBM_table[i].InputBuff    = NULL ;
    MBM_table[i].InputOffset  = 0U ;
    MBM_table[i].InputRbuff   = NULL ;
    MBM_table[i].InputROffset = 0U ;
    MBM_table[i].InputHRbuff  = NULL ;
    MBM_table[i].InputHROffset = 0U ;
  }

  isInit = true ;

  return (bRet);
}

bool MBMDeInit(void)
{
  return true;
}

bool MBMIsInit(void)
{
  return isInit;
}
//
bool MBMOpen(uint8_t u8Ch,uint8_t u8SlaveID)
{
  bool bRet = false ;

  if(u8Ch < MBS_MAX_CH)
  {
    if(MBM_table[u8Ch].isOpen == false)
    {
      MBM_table[u8Ch].u8SlaveID = u8SlaveID ;
      //
      MBM_table[u8Ch].u16TXBuff.rdptr = 0U;
      MBM_table[u8Ch].u16TXBuff.wrptr = 0U;
      MBM_table[u8Ch].u16TXBuff.len   = MDM_QUERY_BUF_LENGTH;
      // .......
      /*
      MBM_table[u8Ch].CoilBuff    = &CoilBuff[0] ;
      MBM_table[u8Ch].InputBuff   = &DInputBuff[0] ;
      MBM_table[u8Ch].InputHRbuff = &HResBuff[0] ;
      MBM_table[u8Ch].InputRbuff  = &ResBuff[0] ;
      */
      MBM_table[u8Ch].u8Status    = COM_IDLE ;
      MBM_table[u8Ch].isOpen      = true ;
      //
    }
  }
  return (bRet);
}
bool IsMBMOpen(uint8_t u8Ch)
{
  return (MBM_table[u8Ch].isOpen);
}
bool MBM_Memory_MAP_Init(uint8_t u8Ch,uint16_t *pCoilBuff,uint16_t *pInputBuff,
                         uint16_t *pInputRbuff,uint16_t *pInputHRbuff)
{
  bool bRet = false ;
  //
  if(MBM_table[u8Ch].isOpen == true)
  {
    MBM_table[u8Ch].CoilBuff    = pCoilBuff;
    MBM_table[u8Ch].InputBuff   = pInputBuff;
    MBM_table[u8Ch].InputRbuff  = pInputRbuff ;
    MBM_table[u8Ch].InputHRbuff = pInputHRbuff ;
    //
    bRet = true ;
  }
  return (bRet);
}
/**
    Modbus function 0x01 Read Coils.

    This function code is used to read from 1 to 2000 contiguous status of
    coils in a remote device. The request specifies the starting address,
    i.e. the address of the first coil specified, and the number of coils.
    Coils are addressed starting at zero.

    The coils in the response buffer are packed as one coil per bit of the
    data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
    data word contains the output addressed in the query. The other coils
    follow toward the high order end of this word and from low order to high
    order in subsequent words.

    If the returned quantity is not a multiple of sixteen, the remaining
    bits in the final data word will be padded with zeros (toward the high
    order end of the word).

    @param u16ReadAddress address of first coil (1 ~ 10000)
    @param u16BitQty quantity of coils to read (1..2000, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup discrete
*/
uint8_t MBMreadCoils(uint8_t u8Ch,uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBReadCoils ;
    MBM_table[u8Ch].u16ReadAdd = u16ReadAddress ;
    MBM_table[u8Ch].u16ReadQty = u16BitQty ;
    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}
/**
    Modbus function 0x02 Read Discrete Inputs.

    This function code is used to read from 1 to 2000 contiguous status of
    discrete inputs in a remote device. The request specifies the starting
    address, i.e. the address of the first input specified, and the number
    of inputs. Discrete inputs are addressed starting at zero.

    The discrete inputs in the response buffer are packed as one input per
    bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
    the first data word contains the input addressed in the query. The other
    inputs follow toward the high order end of this word, and from low order
    to high order in subsequent words.

    If the returned quantity is not a multiple of sixteen, the remaining
    bits in the final data word will be padded with zeros (toward the high
    order end of the word).

    @param u16ReadAddress address of first discrete input (10001 ~ 20000)
    @param u16BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup discrete
*/
uint8_t MBreadDiscreteInputs(uint8_t u8Ch,uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBReadDiscreteInputs ;
    MBM_table[u8Ch].u16ReadAdd = u16ReadAddress ;
    MBM_table[u8Ch].u16ReadQty = u16BitQty ;
    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}

/**
    Modbus function 0x04 Read Input Registers.

    This function code is used to read from 1 to 125 contiguous input
    registers in a remote device. The request specifies the starting
    register address and the number of registers. Registers are addressed
    starting at zero.

    The register data in the response buffer is packed as one word per
    register.

    @param u16ReadAddress address of the first input register (20001 ~ 30000)
    @param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMreadInputRegisters(uint8_t u8Ch,uint16_t u16ReadAddress,uint8_t u16ReadQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBReadInputRegisters ;
    MBM_table[u8Ch].u16ReadAdd = u16ReadAddress ;
    MBM_table[u8Ch].u16ReadQty = u16ReadQty ;
    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}

/**
    Modbus function 0x03 Read Holding Registers.

    This function code is used to read the contents of a contiguous block of
    holding registers in a remote device. The request specifies the starting
    register address and the number of registers. Registers are addressed
    starting at zero.

    The register data in the response buffer is packed as one word per
    register.

    @param u16ReadAddress address of the first holding register (40001 ~ 50000)
    @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMreadHoldingRegisters(uint8_t u8Ch,uint16_t u16ReadAddress,uint16_t u16ReadQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBReadHoldingRegisters ;
    MBM_table[u8Ch].u16ReadAdd = u16ReadAddress ;
    MBM_table[u8Ch].u16ReadQty = u16ReadQty ;
    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}
/**
    Modbus function 0x05 Write Single Coil.

    This function code is used to write a single output to either ON or OFF
    in a remote device. The requested ON/OFF state is specified by a
    constant in the state field. A non-zero value requests the output to be
    ON and a value of 0 requests it to be OFF. The request specifies the
    address of the coil to be forced. Coils are addressed starting at zero.

    @param u16WriteAddress address of the coil (0x0000..0xFFFF)
    @param u8State 0=OFF, non-zero=ON (0x00..0xFF)
    @return 0 on success; exception number on failure
    @ingroup discrete
    */
uint8_t MBMwriteCoil(uint8_t u8Ch,uint16_t u16WriteAddress, uint8_t u8State)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBWriteSingleCoil ;
    MBM_table[u8Ch].u16ReadAdd = u16WriteAddress ;
    if(u8State == 0x00) { MBM_table[u8Ch].u16ReadQty = 0x0000U ; }
    else { MBM_table[u8Ch].u16ReadQty = 0xFF00U ; }

    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]);        // Protocol Format same read & write
  }
  return (u8Ret);
}


/**
    Modbus function 0x06 Write Single Register.

    This function code is used to write a single holding register in a
    remote device. The request specifies the address of the register to be
    written. Registers are addressed starting at zero.

    @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
    @param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMwriteRegister(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16WriteValue)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBWriteSingleRegister ;
    MBM_table[u8Ch].u16ReadAdd = u16WriteAddress ;
    MBM_table[u8Ch].u16ReadQty = u16WriteValue ;
    if(u16WriteValue == 0x00) { MBM_table[u8Ch].u16ReadQty = 0x0000U ; }
    else { MBM_table[u8Ch].u16ReadQty = 0xFF00U ; }

    u8Ret = MBM_QReadSend(&MBM_table[u8Ch]); // Protocol Format same read & write
  }
  return (u8Ret);
}


/**
    Modbus function 0x0F Write Multiple Coils.

    This function code is used to force each coil in a sequence of coils to
    either ON or OFF in a remote device. The request specifies the coil
    references to be forced. Coils are addressed starting at zero.

    The requested ON/OFF states are specified by contents of the transmit
    buffer. A logical '1' in a bit position of the buffer requests the
    corresponding output to be ON. A logical '0' requests it to be OFF.

    @param u16WriteAddress address of the first coil (0x0000..0xFFFF)
    @param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup discrete
*/
uint8_t MBMwriteCoils(uint8_t u8Ch,uint16_t u16WriteAddress, uint16_t u16BitQty)
{
  uint8_t u8Ret = MBFalse ;
  //MBM_table[u8Ch].u8Status = COM_IDLE;
  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBWriteMultipleCoils ;
    MBM_table[u8Ch].u16WriteAdd = u16WriteAddress ;
    MBM_table[u8Ch].u16WriteQty = u16BitQty ;

    u8Ret = MBM_FWriteSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);

}
/**
    Modbus function 0x10 Write Multiple Registers.

    This function code is used to write a block of contiguous registers (1
    to 123 registers) in a remote device.

    The requested written values are specified in the transmit buffer. Data
    is packed as one word per register.

    @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
    @param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMwriteRegisters(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16WriteQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBWriteMultipleRegisters ;
    MBM_table[u8Ch].u16WriteAdd = u16WriteAddress ;
    MBM_table[u8Ch].u16WriteQty = u16WriteQty ;

    u8Ret = MBM_FWriteSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}

/**
    Modbus function 0x16 Mask Write Register.

    This function code is used to modify the contents of a specified holding
    register using a combination of an AND mask, an OR mask, and the
    register's current contents. The function can be used to set or clear
    individual bits in the register.

    The request specifies the holding register to be written, the data to be
    used as the AND mask, and the data to be used as the OR mask. Registers
    are addressed starting at zero.

    The function's algorithm is:

    Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))

    @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
    @param u16AndMask AND mask (0x0000..0xFFFF)
    @param u16OrMask OR mask (0x0000..0xFFFF)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMmaskWriteRegister(uint8_t u8Ch,uint16_t u16WriteAddress,uint16_t u16AndMask, uint16_t u16OrMask)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBMaskWriteRegister ;
    MBM_table[u8Ch].u16WriteAdd = u16WriteAddress ;
    MBM_table[u8Ch].u16TXBuff.u16Buff[0] = u16AndMask ;
    MBM_table[u8Ch].u16TXBuff.u16Buff[0] = u16OrMask ;
    u8Ret = MBM_FWriteSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}


/**
    Modbus function 0x17 Read Write Multiple Registers.

    This function code performs a combination of one read operation and one
    write operation in a single MODBUS transaction. The write operation is
    performed before the read. Holding registers are addressed starting at
    zero.

    The request specifies the starting address and number of holding
    registers to be read as well as the starting address, and the number of
    holding registers. The data to be written is specified in the transmit
    buffer.

    @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
    @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
    @param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
    @param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
    @return 0 on success; exception number on failure
    @ingroup register
*/
uint8_t MBMreadWriteRegisters(uint8_t u8Ch,uint16_t u16ReadAddress,
  uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
  uint8_t u8Ret = MBFalse ;

  if((u8Ch < MBS_MAX_CH) && (MBM_table[u8Ch].u8Status == COM_IDLE))
  {
    MBM_table[u8Ch].u8Function = MBReadWriteMultipleRegisters ;
    MBM_table[u8Ch].u16ReadAdd  = u16ReadAddress ;
    MBM_table[u8Ch].u16ReadQty  = u16ReadQty ;
    MBM_table[u8Ch].u16WriteAdd = u16WriteAddress ;
    MBM_table[u8Ch].u16WriteQty = u16WriteQty ;

    u8Ret = MBM_FWriteSend(&MBM_table[u8Ch]);
  }
  return (u8Ret);
}
// ---------------------------------------------------------------------------
// Define private function definitions.  
// ---------------------------------------------------------------------------
//
// assemble Modbus Request Application Data Unit (ADU),
uint8_t MBM_QReadSend(MBM_t *pMB_node)
{
  uint8_t u8Ret = MBSuccess ;

  uint8_t  ModbusADU[12] = {0x00U,};
  uint8_t  ADU_ptr        = 0x00U ;
  uint16_t u16CRC         = 0xFFFFU ;
  //
  ModbusADU[ADU_ptr++] = pMB_node->u8SlaveID ;
  ModbusADU[ADU_ptr++] = pMB_node->u8Function ;
  ModbusADU[ADU_ptr++] = highByte(pMB_node->u16ReadAdd);
  ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16ReadAdd);
  //
  switch(pMB_node->u8Function)
  {
    case MBReadCoils:
    case MBReadDiscreteInputs:
    case MBReadInputRegisters:
    case MBReadHoldingRegisters:
    case MBReadWriteMultipleRegisters:
      ModbusADU[ADU_ptr++] = highByte(pMB_node->u16ReadQty);
      ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16ReadQty);
      break;
    default:
      u8Ret = MBFalse ;
  }
  //
  if(u8Ret == MBSuccess)
  { // make crc
    u16CRC = MODBUS_CRC16_v3((const uint8_t *)ModbusADU,ADU_ptr);
    ModbusADU[ADU_ptr++] =  lowByte(u16CRC);
    ModbusADU[ADU_ptr++] = highByte(u16CRC);

    uartWriteBuff(UART1, ModbusADU, ADU_ptr);
    pMB_node->u8Status = COM_WAITING ;
  }

  return (u8Ret);
}
//
//
//

// assemble Modbus Request Application Data Unit (ADU),
uint8_t MBM_FWriteSend(MBM_t *pMB_node)
{
  uint8_t  u8Ret          = MBSuccess ;
  uint8_t  u8i            = 0U;

  uint8_t  u8RegsNo       = 0U;
  uint8_t  u8BytesNo      = 0U;

  uint8_t  ModbusADU[128]  = {0x00U,};
  uint8_t  ADU_ptr        = 0x00U ;
  uint16_t u16CRC         = 0xFFFFU ;
  uint16_t *pu16SrcBuff ;


  //
  ModbusADU[ADU_ptr++] = pMB_node->u8SlaveID ;
  ModbusADU[ADU_ptr++] = pMB_node->u8Function ;
  //
  if(pMB_node->u8Function == MBReadWriteMultipleRegisters)
  {
    ModbusADU[ADU_ptr++] = highByte(pMB_node->u16ReadAdd);
    ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16ReadAdd);
    ModbusADU[ADU_ptr++] = highByte(pMB_node->u16ReadQty);
    ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16ReadQty);
  }
  //
  ModbusADU[ADU_ptr++] = highByte(pMB_node->u16WriteAdd);
  ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16WriteAdd);

  switch(pMB_node->u8Function)
  {
    case MBWriteSingleCoil:
    case MBWriteMultipleCoils:
      //uint8_t u8StartBuff = pMB_node->u16WriteAdd / 16U ;
      pu16SrcBuff = &pMB_node->CoilBuff[pMB_node->u16WriteAdd / 16U ] ;
      break ;
    case MBWriteSingleRegister:
    case MBWriteMultipleRegisters:
    case MBReadWriteMultipleRegisters:
      pu16SrcBuff = &pMB_node->InputHRbuff[pMB_node->u16WriteAdd]  ;
      break ;
  }
  switch(pMB_node->u8Function)
  {
    case MBWriteMultipleCoils:
      ModbusADU[ADU_ptr++] = highByte(pMB_node->u16WriteQty);
      ModbusADU[ADU_ptr++] =  lowByte(pMB_node->u16WriteQty);

      u8RegsNo  = pMB_node->u16WriteQty / 16U;
      u8BytesNo = u8RegsNo * 2 ;
      //
      if((pMB_node->u16WriteQty % 16U) != 0U)
      {
        u8RegsNo++ ;
        u8BytesNo++ ;
      }
      ModbusADU[ADU_ptr++] = u8BytesNo ;
      //

      //
      for(u8i=0; u8i<u8BytesNo; u8i++)
      {
        if(u8i % 2U)
        {
          ModbusADU[ADU_ptr] = lowByte(pu16SrcBuff[u8i/2U]);
        }
        else
        {
          ModbusADU[ADU_ptr] = highByte(pu16SrcBuff[u8i/2U]);
        }
        ADU_ptr++ ;
      }
      break;

    case MBMaskWriteRegister:
      ModbusADU[ADU_ptr++]  = highByte(pu16SrcBuff[0]);
      ModbusADU[ADU_ptr++]  =  lowByte(pu16SrcBuff[0]);
      break ;
    case MBReadWriteMultipleRegisters:
    case MBWriteMultipleRegisters:
      ModbusADU[ADU_ptr++]  = highByte(pMB_node->u16WriteQty);
      ModbusADU[ADU_ptr++]  =  lowByte(pMB_node->u16WriteQty);
      ModbusADU[ADU_ptr++]  =(uint8_t)(pMB_node->u16WriteQty<<1U);
      //
      for(u8i=0; u8i<lowByte(pMB_node->u16WriteQty); u8i++)
      {
        ModbusADU[ADU_ptr++] = highByte(pu16SrcBuff[u8i]);
        ModbusADU[ADU_ptr++] =  lowByte(pu16SrcBuff[u8i]);
      }
      break;
    default:
      u8Ret = MBFalse ;
  }
  //
  if(u8Ret == MBSuccess)
  { // make crc
    u16CRC = MODBUS_CRC16_v3((const uint8_t *)ModbusADU,ADU_ptr);
    ModbusADU[ADU_ptr++] =  lowByte(u16CRC);
    ModbusADU[ADU_ptr++] =  highByte(u16CRC);

    uartWriteBuff(UART1, ModbusADU, ADU_ptr);
    pMB_node->u8Status = COM_WAITING ;
  }

  return (u8Ret);
}

uint8_t MBM_Response_Check(const uint8_t * pu8Buff,uint8_t u8Len)
{
  uint8_t u8Ret   = MBFalse ;
  uint8_t u8i     = 0U ;
  uint8_t u8Ch    = 0U ;
  uint8_t u8BWR_ptr         = 0U ;
  uint8_t u8BBWR_ptr        = 0U ;
  uint8_t u8WritetAllCount  = 0U ;

  uint16_t *pu16DesBuff ;
  uint16_t u16Respose[32] ;
  uint8_t  u16Respose_ptr = 0 ;
  //
  for(u8Ch=0; u8Ch<MBS_MAX_CH; u8Ch++)
  {
    if(MBM_table[u8Ch].u8SlaveID == pu8Buff[0]) // Check SlaveID
    {
      u8Ret = MBSuccess ;
      break ;
    }
    else
    {
      u8Ret = MBInvalidSlaveID ;
    }
  }
  //
  if(u8Ret == MBSuccess)
  {
    if(MBM_table[u8Ch].u8Function != pu8Buff[1]) // Check MBM Function
    {
      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(pu8Buff[1], 7U))
      {
        u8Ret = pu8Buff[2]; // return Modbus Exception Code
      }
      else { u8Ret = MBInvalidFunction ;}
    }
    else
    {
      switch(pu8Buff[1])
      {
        case MBReadCoils:
          pu16DesBuff = MBM_table[u8Ch].CoilBuff ;
          break ;
        case MBReadDiscreteInputs:
          pu16DesBuff = MBM_table[u8Ch].InputBuff ;
          break ;
        case MBReadHoldingRegisters:
          pu16DesBuff = MBM_table[u8Ch].InputHRbuff ;
          break ;
        case MBReadInputRegisters:
          pu16DesBuff = MBM_table[u8Ch].InputRbuff ;
          break ;
          //
        case MBWriteSingleCoil:
        case MBWriteMultipleCoils:
        case MBWriteSingleRegister:
        case MBWriteMultipleRegisters:
          MBM_table[u8Ch].u8Status = COM_IDLE ;
          break;
        case MBReadWriteMultipleRegisters:
          pu16DesBuff = MBM_table[u8Ch].InputHRbuff ;
          break;

      }

      // evaluate returned Modbus function code
      switch(pu8Buff[1])
      {
        /**
         * This method processes functions 1 & 2 (for master)
         * This method puts the slave answer into master data buffer
         *
         */
        case MBReadCoils:
        case MBReadDiscreteInputs:

        // load bytes into word; response bytes are ordered L, H, L, H, ...
          for(u8i=0; u8i<(pu8Buff[2] >> 1U); u8i++)
          {
            if(u8i < 32)
            {
              u16Respose[u8i] = word(pu8Buff[2*u8i + 4],pu8Buff[2*u8i + 3]);
            }
            u16Respose_ptr = u8i ;
          }
          //
          if(pu8Buff[2] % 2)
          {
            if(u8i < 32)
            {
              u16Respose[u8i] = word(0x00U,pu8Buff[2*u8i + 3]);
            }
            u16Respose_ptr = u8i + 1U ;
          }
          //  program modify
          u8BWR_ptr         = (uint8_t)(MBM_table[u8Ch].u16ReadAdd / 16U)  ;
          u8BBWR_ptr        = (uint8_t)(MBM_table[u8Ch].u16ReadAdd % 16U)  ;
          u8WritetAllCount  = (uint8_t)MBM_table[u8Ch].u16ReadQty ;

          for(u8i=0; u8i<u8WritetAllCount; u8i++)
          {
            if(bitRead(u16Respose[u8i/16U],u8i%16U) == 0x01)
            {
              //bitWrite((MBM_table[u8Ch].CoilBuff[u8BWR_ptr + (u8BBWR_ptr/16U)]),u8BBWR_ptr%16U,1U);
              bitWrite((pu16DesBuff[u8BWR_ptr + (u8BBWR_ptr/16U)]),u8BBWR_ptr%16U,1U);
            }
            else {
              //bitWrite((MBM_table[u8Ch].CoilBuff[u8BWR_ptr + (u8BBWR_ptr/16U)]),u8BBWR_ptr%16U,0U);
              bitWrite((pu16DesBuff[u8BWR_ptr + (u8BBWR_ptr/16U)]),u8BBWR_ptr%16U,0U);
            }
            u8BBWR_ptr = u8BBWR_ptr + 1U ;
          }
          MBM_table[u8Ch].u8Status = COM_IDLE ;
          break;
        /**
         * This method processes functions 3 & 4 (for master)
         * This method puts the slave answer into master data buffer
         *
         * @ingroup register
         */
        case MBReadHoldingRegisters:
        case MBReadInputRegisters:
        case MBReadWriteMultipleRegisters:
          u8BWR_ptr = (uint8_t)(MBM_table[u8Ch].u16ReadAdd )  ;
          for(u8i=0; u8i<(pu8Buff[2] >> 1U); u8i++)
          {
            pu16DesBuff[u8BWR_ptr + u8i] = word(pu8Buff[2*u8i + 3],pu8Buff[2*u8i + 4]);
          }
          //
          MBM_table[u8Ch].u8Status = COM_IDLE ;
          break ;
      }
    }
  }

  return (u8Ret);
}
/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */

// ---------------------------------------------------------------------------
//  Define Callbacks definitions.
// ---------------------------------------------------------------------------
//
 
// ---------------------------------------------------------------------------
//  Define CLI definitions.
// ---------------------------------------------------------------------------
//
 /* ModbusMaster.c End Of File !! Well Done */
