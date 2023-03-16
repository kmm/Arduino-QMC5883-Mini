# Arduino-QMC5883-Mini (KM_QMC5883)
> Wire/SoftWire QMC5883L digital compass/magnetometer mini driver

## Overview 

Minimalistic header-only Arduino driver for QMC5883L magnetic sensors, templated for compatibility with Wire and SoftWire I2C libraries.
Does not currently implement compass functionality such as heading calculation, and is functional but not extensively tested. 

## SoftWire Example
> (Arduino boilerplate not included)
```cpp
#include <SoftWire.h>
#include "qmc5883l.hpp"
#define PIN_BUS_SDA 3
#define PIN_BUS_SCL 4
 
SoftWire sw = SoftWire(PIN_BUS_SDA, PIN_BUS_SCL);
 
// Initialize sensor (replace SoftWire with Wire for hardware I2C)
KM_QMC5883<SoftWire> kqmc = KM_QMC5883<SoftWire>(&sw, 13);
bool initstatus = kqmc.begin(); // true if device ACKed
kqmc.config(kqmc.MODE_CONT, kqmc.ODR_10HZ, kqmc.RANGE_2G);
 
// Take a reading, returns gaussish* value for XYZ 
// in float[3] array via passed pointer.
// (* I don't have a gaussmeter so I'm just trusting the datasheet.)
 
// Allocate storage for read values
float xyz[3];
 
// Perform reading, returning QMC status register contents
uint8_t readstatus = kqmc.readXYZ((float *)xyz);
Serial.printf("X: %f, Y: %f, Z: %f\r\n", xyz[0], xyz[1], xyz[2]);
 
if(readstatus & kqmc.STATUS_OVL_MASK) {
	Serial.println("Measurement out of range!");
}
  
// Put sensor in standby mode
kqmc.standby();
```

## References
Roughly based on my MicroPython driver fork: https://github.com/kmm/micropython-QMC5883L
With some referencing of the Adafruit HMC driver for Wire I2C comms: https://github.com/adafruit/Adafruit_HMC5883_Unified