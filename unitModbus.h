/**
 * @file 	plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author 	pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
*/


#include <hydroModbusCommon.h>


// specific read holding registers to unit
//#define FUTURE CUSTOM_HR_START_READ
#define B1N1_T1_LSH_001_MB CUSTOM_HR_START_READ 
#define B1N1_T1_LSL_001_MB B1N1_T1_LSH_001_MB + 1
#define B1N1_T1_ZSC_003_MB B1N1_T1_LSL_001_MB + 1 
#define B1N1_T1_ZSC_001_MB B1N1_T1_ZSC_003_MB + 1
#define B1N1_T1_ZSC_002_MB B1N1_T1_ZSC_001_MB + 1 
#define B1N1_T1_ZSC_004_MB B1N1_T1_ZSC_002_MB + 1

// specific write holding registers to unit
#define FUTURES CUSTOM_HR_START_WRITE


// 
// write analogs/sp specific to units
//  HOLDING_REGISTER_WRITE_OFFSET + LAST #DEFINE IN THE LIST ON TOP.
//  IF YOU ADD MORE ENSURE THE CHANGE IS MADE HERE 



#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + B1N1_T1_ZSC_004_MB + 1
uint16_t modbusRegisters[MODBUS_REG_COUNT];

#define MB_SPEED 			    19200
#define MB_SLAVE_ID				1
#define MB_SERIAL_PORT			0
#define MB_MAX485_PIN			6  // set to zero for RS-232



Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT,MB_MAX485_PIN); 

