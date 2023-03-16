
/*********************************************************************
 * @file  qmc5883l.hpp
 * 
 * @brief Wire/SoftWire QMC5883L digital compass/magnetometer mini driver
 * 
 * Minimalistic header-only Arduino driver for QMC5883L magnetic sensors,
 * templated for compatibility with Wire and SoftWire I2C libraries.
 * Does not currently implement compass functionality such as heading
 * calculation, and is not extensively tested. Roughly based on my
 * MicroPython driver fork:
 *  https://github.com/kmm/micropython-QMC5883L
 *  
 * SoftWire Example (Arduino boilerplate not included):
 *  #include "qmc5883l.hpp"
 *  #define PIN_BUS_SDA 3
 *  #define PIN_BUS_SCL 4
 *  
 *  SoftWire sw = SoftWire(PIN_BUS_SDA, PIN_BUS_SCL);
 *  
 *  // Initialize sensor (replace SoftWire with Wire for hardware I2C)
 *  KM_QMC5883<SoftWire> kqmc = KM_QMC5883<SoftWire>(&sw, 13);
 *  bool initstatus = kqmc.begin(); // true if device ACKed
 *  kqmc.config(kqmc.MODE_CONT, kqmc.ODR_10HZ, kqmc.RANGE_2G);
 *  
 *  // Take a reading, returns gaussish* value for XYZ 
 *  // in float[3] array via passed pointer.
 *  // (* I don't have a gaussmeter so I'm just trusting the datasheet.)
 *  
 *  // Allocate storage for read values
 *  float xyz[3];
 *  
 *  // Perform reading, returning QMC status register contents
 *  uint8_t readstatus = kqmc.readXYZ((float *)xyz);
 *  Serial.printf("X: %f, Y: %f, Z: %f\r\n", xyz[0], xyz[1], xyz[2]);
 *  
 *  if(readstatus & kqmc.STATUS_OVL_MASK) {
 *      Serial.println("Measurement out of range!");
 *  }
 *  
 *  // Put sensor in standby mode
 *  kqmc.standby();
 *  
 * @copyright 
 * Copyright 2023 kmm
 * MIT License
 *********************************************************************/
#include <Arduino.h>
template <class TWire>
class KM_QMC5883 
{
  public:
    static const uint8_t REG_STATUS = 0x06;
    static const uint8_t REG_TEMPL = 0x07;
    static const uint8_t REG_TEMPH = 0x07;
    static const uint8_t REG_CR1 = 0x09;
    static const uint8_t REG_CR2 = 0x0A;
    static const uint8_t REG_SR = 0x0B;
    static const uint8_t REG_CHIPID = 0x0D;

    static const uint8_t MODE_STBY = 0;
    static const uint8_t MODE_CONT = 1;
    static const uint8_t ODR_10HZ = 0;
    static const uint8_t ODR_50HZ = 1;
    static const uint8_t ODR_100HZ = 2;
    static const uint8_t ODR_200HZ = 3;
    static const uint8_t RANGE_2G = 0;
    static const uint8_t RANGE_8G = 1;
    static const uint8_t OSR_512 = 0;
    static const uint8_t OSR_256 = 1;
    static const uint8_t OSR_128 = 2;
    static const uint8_t OSR_64 = 3;
    // Status register bit masks
    static const uint8_t STATUS_DRDY_MASK = 1;  // Data ready flag
    static const uint8_t STATUS_OVL_MASK = 2;   // Overrange flag
    static const uint8_t STATUS_DOR_MASK = 4;   // Data skip flag

    TWire *_wirelib;
    uint8_t _address = 0x0D;
    float _scale = 12000.0F;

    KM_QMC5883<TWire>(TWire *wirelib, uint8_t address = 0x0D) {
        this->_wirelib = wirelib;
        this->_address = address;
    }

    bool begin(void) {
        _wirelib->begin();
        _wirelib->endTransmission(true);
        _wirelib->beginTransmission(this->_address);
        delay(5);
        uint8_t result = _wirelib->endTransmission(true);
        if(result == 0) return true;
        else return false;
    }

    void writeReg8(uint8_t reg, uint8_t value) {
        _wirelib->beginTransmission(this->_address);
        _wirelib->write(reg);
        _wirelib->write(value);
        _wirelib->endTransmission();
    }

    uint8_t readReg8(uint8_t reg) {
        _wirelib->beginTransmission(this->_address);
        _wirelib->write(reg);
        _wirelib->endTransmission();
        _wirelib->requestFrom(this->_address, 1);
        uint8_t value = _wirelib->read();
        _wirelib->endTransmission();
        return value;
    }

    void reset(void) {
        writeReg8(REG_CR2, 0x80);
    }

    void standby(void) {
        writeReg8(REG_CR1, MODE_STBY);
    }

    void config( uint8_t mode = MODE_CONT, uint8_t odr = ODR_10HZ,
                uint8_t range = RANGE_2G, uint8_t osr = OSR_512,
                uint8_t roll_en = 1, uint8_t int_en = 0) 
    {
        reset();
        delay(10);
        switch(range) {
            case RANGE_2G:
                this->_scale = 12000.0F;
                break;
            case RANGE_8G:
                this->_scale = 3000.0F;
                break;
            default:
                range = RANGE_2G;
                this->_scale = 12000.0F;
        }
        uint8_t cr1_byte = (osr << 6) | (range << 4) | (odr << 2) | mode;
        uint8_t cr2_byte = (roll_en << 6) | int_en;
        writeReg8(REG_SR, 0x01);
        writeReg8(REG_CR1, cr1_byte);
        writeReg8(REG_CR2, cr2_byte);
    }

    void readRaw(uint8_t *data) {
        for(uint8_t i = 0; i < 6; i++) {
            data[i] = readReg8(i);
        }
    }

    uint8_t readXYZ(float *xyz) {
        uint8_t data[6];
        uint8_t status = readReg8(REG_STATUS);
        readRaw(data);
        int16_t rx = (data[1] << 8) | data[0];
        int16_t ry = (data[3] << 8) | data[2];
        int16_t rz = (data[5] << 8) | data[4];
        xyz[0] = rx / this->_scale;
        xyz[1] = ry / this->_scale;
        xyz[2] = rz / this->_scale;
        return status;
    }
};