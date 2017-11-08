


/**
*  @file    hydroNutrientTank.ino
*  @author  peter c
*  @date    11/07/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Nutrient Tank Controllers Code based treatment controller
** @section HISTORY
** 2017Oct25 - created
*/
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // for RTC
#include <Streaming.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <Adafruit_MCP23008.h>

#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <DA_NonBlockingDelay.h>
#include <flowmeter.h>

#include "unitModbus.h"
// comment out to  include terminal processing for debugging
 //#define PROCESS_TERMINAL
//#define TRACE_FLOW_SENSOR
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
 //#define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
#define ALARM_REFRESH_INTERVAL 10 // ms


#define B1N1_T1_LSH_001_GPIO 3 // GPIO for MCP ports 0 is labelled DRAIN but not used
#define B1N1_T1_LSL_001_GPIO 1
#define B1N1_T1_ZSC_003_GPIO 5
#define B1N1_T1_ZSC_001_GPIO 2
#define B1N1_T1_ZSC_002_GPIO 0
#define B1N1_T1_ZSC_004_GPIO 6

bool GPIOStatus[8];

#define NUTRIENT_TANK_MCP_ADDRESS 0
Adafruit_MCP23008 nutrientTank;
// One Wire - Hydroponic temperatures
// 
#define TEMPERATURE1 7 // pin
#define TEMPERATURE2 12 // pin

#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWireBus1(TEMPERATURE1);
OneWire oneWireBus2(TEMPERATURE2);

DallasTemperature B1N1_T1_TT_003(& oneWireBus1);
DallasTemperature B1N1_T1_TT_004(& oneWireBus2);

DA_DiscreteOutput B1N1_T1_XY_005 = DA_DiscreteOutput(9, LOW); // V1
DA_DiscreteOutput B1N1_T1_XY_003 = DA_DiscreteOutput(3, LOW); // V2
DA_DiscreteOutput B1N1_T1_XY_001 = DA_DiscreteOutput(10, LOW); // V3
DA_DiscreteOutput B1N1_T1_XY_002 = DA_DiscreteOutput(11, LOW); // V4
DA_AnalogInput B1N1_T1_PT_001 = DA_AnalogInput(A1, 0.0, 1023.); // min max
DA_AnalogInput B1N1_T1_PT_002 = DA_AnalogInput(A2, 0.0, 1023.); // min max
DA_AnalogInput B1N1_T1_PT_003 = DA_AnalogInput(A6, 0.0, 1023.); // min max
DA_AnalogInput B1N1_LT_001 = DA_AnalogInput(A7, 0.0, 1023.); // min max

//Flow meter 
#define B1N1_T1_FT_004_PIN  A3
#define B1N1_T1_FT_003_PIN  3
//#define FT_004_PIN  D4
#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period

#define ENABLE_FLOW3_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(B1N1_T1_FT_003_PIN), onB1N1_T1_FT_003_PulseIn, RISING)
#define DISABLE_FLOW3_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(B1N1_T1_FT_003_PIN))
#define ENABLE_FLOW4_SENSOR_INTERRUPTS attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(B1N1_T1_FT_004_PIN), onB1N1_T1_FT_004_PulseIn, RISING);
#define DISABLE_FLOW4_SENSOR_INTERRUPTS detachPinChangeInterrupt(digitalPinToInterrupt(B1N1_T1_FT_003_PIN))


FlowMeter B1N1_T1_FT_004(B1N1_T1_FT_004_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt pin, calculation period in seconds
FlowMeter B1N1_T1_FT_003(B1N1_T1_FT_003_PIN, FLOW_CALC_PERIOD_SECONDS);

// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( POLL_CYCLE_SECONDS*1000, &doOnPoll);
DA_NonBlockingDelay flowRateTimer = DA_NonBlockingDelay( FLOW_CALC_PERIOD_SECONDS*1000, &doOnCalcFlowRate);


// HEARTBEAT
unsigned int heartBeat = 0;



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial;
#endif

void onB1N1_T1_FT_004_PulseIn()
{
  B1N1_T1_FT_004.handleFlowDetection();
}

void onB1N1_T1_FT_003_PulseIn()
{
  B1N1_T1_FT_003.handleFlowDetection();
}


void printOneWireAddress(HardwareSerial *tracePort, DeviceAddress aDeviceAddress, bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *tracePort << '0';
    tracePort->print(aDeviceAddress[i], HEX);
  }
  if (aCR)
    *tracePort << endl;
}

void init1WireTemperatureSensor(DallasTemperature * sensor, int idx)
{
  DeviceAddress address;
  sensor->begin();
  if (sensor->getAddress(address, 0))
  {

#ifdef TRACE_1WIRE
    *tracePort << "Channel " << idx << " 1Wire Temperature initialized. Address =  ";
    printOneWireAddress(tracePort, address, true);
#endif

    sensor->setResolution(address, ONE_TEMPERATURE_PRECISION);
  }
  else
  {

#ifdef TRACE_1WIRE
    *tracePort << "Unable to find address for 1Wire Temperature Device @ " << idx << endl;
#endif

  }
}

void initHydroponicOneWireTemps()
{
  init1WireTemperatureSensor(& B1N1_T1_TT_003, 1);
  init1WireTemperatureSensor(& B1N1_T1_TT_004, 2);
  
}

void setupMCP()
{
    nutrientTank.begin(NUTRIENT_TANK_MCP_ADDRESS);
    for(int i=0;i<8;i++) {
      nutrientTank.pinMode(i,INPUT);
      nutrientTank.pullUp(i, HIGH);
    }

}
void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  randomSeed(analogRead(3));

  initHydroponicOneWireTemps();
  setupMCP();
  ENABLE_FLOW4_SENSOR_INTERRUPTS;
  ENABLE_FLOW3_SENSOR_INTERRUPTS;
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
pollTimer.refresh();
  flowRateTimer.refresh();
  //*tracePort << "looping" << endl;

}

void processFlowMeter4()
{
    DISABLE_FLOW4_SENSOR_INTERRUPTS;
  B1N1_T1_FT_004.end();

  #ifdef TRACE_FLOW_SENSOR
  *tracePort << "FLOW 4:";
    B1N1_T1_FT_004.serialize( tracePort, true);
  #endif

  B1N1_T1_FT_004.begin();
  ENABLE_FLOW4_SENSOR_INTERRUPTS;
}

void processFlowMeter3()
{
    DISABLE_FLOW3_SENSOR_INTERRUPTS;
  B1N1_T1_FT_003.end();
  
  #ifdef TRACE_FLOW_SENSOR
  *tracePort << "FLOW 3:";
    B1N1_T1_FT_003.serialize( tracePort, true);


  #endif

  B1N1_T1_FT_003.begin();
  ENABLE_FLOW3_SENSOR_INTERRUPTS;
}

void doOnCalcFlowRate()
{
processFlowMeter4();
processFlowMeter3();


  // resetTotalizers();
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{

  doReadAnalogs();

  doProcess1WireTemperatures();
  for( int i=0;i<8;i++)
  {

    GPIOStatus[i] = nutrientTank.digitalRead(i);
  }

  heartBeat++;

}

void doPoll1WireTemperature(DallasTemperature * sensor, int idx)
{
  sensor->requestTemperatures();

#ifdef TRACE_1WIRE
  *tracePort << "Temperature " << idx << " = " << sensor->getTempCByIndex(0) << " C" << endl;
#endif

}

void doProcess1WireTemperatures()
{
  doPoll1WireTemperature(& B1N1_T1_TT_003, 0);
  doPoll1WireTemperature(& B1N1_T1_TT_004, 1);
  
}

void doReadAnalogs()
{
  B1N1_T1_PT_001.refresh();
  B1N1_T1_PT_002.refresh();
  B1N1_T1_PT_003.refresh();


#ifdef TRACE_3NALOGS
  B1N1_T1_PT_001.serialize(tracePort, true);
  B1N1_T1_PT_002.serialize(tracePort, true);
  B1N1_T1_PT_003.serialize(tracePort, true);
 
#endif

}



// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{

  modbusRegisters[HR_TEMPERATURE1] = B1N1_T1_TT_003.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE2] = B1N1_T1_TT_004.getTempCByIndex(0) * 100;
  
  modbusRegisters[HR_PRESSURE1] = B1N1_T1_PT_001.getRawSample();
  modbusRegisters[HR_PRESSURE2] = B1N1_T1_PT_002.getRawSample();
  modbusRegisters[HR_PRESSURE3] = B1N1_T1_PT_003.getRawSample();
  modbusRegisters[HR_FLOW1] = B1N1_T1_FT_004.getCurrentPulses();
  modbusRegisters[HR_FLOW2] = B1N1_T1_FT_003.getCurrentPulses();  
  modbusRegisters[HR_HEARTBEAT] = heartBeat;

  modbusRegisters[B1N1_T1_LSH_001_MB] = GPIOStatus[B1N1_T1_LSH_001_GPIO];
  modbusRegisters[B1N1_T1_LSL_001_MB]= GPIOStatus[B1N1_T1_LSL_001_GPIO];
  modbusRegisters[B1N1_T1_ZSC_003_MB]= GPIOStatus[B1N1_T1_ZSC_003_GPIO];
  modbusRegisters[B1N1_T1_ZSC_001_MB]= GPIOStatus[B1N1_T1_ZSC_001_GPIO];
  modbusRegisters[B1N1_T1_ZSC_002_MB]= GPIOStatus[B1N1_T1_ZSC_002_GPIO];
  modbusRegisters[B1N1_T1_ZSC_004_MB]= GPIOStatus[B1N1_T1_ZSC_004_GPIO];
  
}


bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkAndActivateDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->activate();

  #ifdef TRACE_MODBUS
    *tracePort << "Activate DO:";
    aDO->serialize(tracePort, true);
    LED.activate();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (!getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->reset();

  #ifdef TRACE_MODBUS
    *tracePort << "Reset DO:";
    aDO->serialize(tracePort, true);
    LED.reset();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void processValveCommands()
{
  checkAndActivateDO(VALVE1_OPEN_CLOSE, & B1N1_T1_XY_003);
  checkAndResetDO(VALVE1_OPEN_CLOSE, & B1N1_T1_XY_003);

  checkAndActivateDO(VALVE2_OPEN_CLOSE, & B1N1_T1_XY_002);
  checkAndResetDO(VALVE2_OPEN_CLOSE, & B1N1_T1_XY_002);

  checkAndActivateDO(VALVE3_OPEN_CLOSE, & B1N1_T1_XY_001);
  checkAndResetDO(VALVE3_OPEN_CLOSE, & B1N1_T1_XY_001);

  checkAndActivateDO(VALVE4_OPEN_CLOSE, & B1N1_T1_XY_005);
  checkAndResetDO(VALVE4_OPEN_CLOSE, & B1N1_T1_XY_005);
}

void processModbusCommands()
{
  processValveCommands();
}

#endif
