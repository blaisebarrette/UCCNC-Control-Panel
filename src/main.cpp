#include <Arduino.h>
#include <ModbusRTU.h>
#define SLAVE_ID 1

#define Transmit_enable_pin 4 /*___________RTU Enable pin                                  */
#define rxdPin 16             /*_______________________Modbus Serial RX pin                            */
#define txdPin 17             /*_______________________Modbus Serial TX pin                            */
ModbusRTU mb;                 /* Modbus Initialisation */
void setup()
{
  Serial1.begin(115200, SERIAL_8N1, rxdPin, txdPin);
  mb.begin(&Serial1, Transmit_enable_pin);
  mb.slave(SLAVE_ID);

  mb.addHreg(0);
  mb.addHreg(1);
  //mb.Hreg(0, 123);
}

void loop()
{
  mb.task();
  yield();
}