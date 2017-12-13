/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"

#ifndef ACCELEROMETER_ADXL345_H
#define ACCELEROMETER_ADXL345_H

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Device I2C address. */
#define ACCELEROMETER_ADXL345_ADDRESS   0x53

/* ----------------------------------------------------------------
 * CLASSES
 * -------------------------------------------------------------- */

class AccelerometerAdxl345 {
public:
    /** The types of events that can be
     * returned by the interrupt handler.
     */
    typedef enum {
        EVENT_NONE,
        EVENT_ACTIVITY,
        EVENT_SINGLE_TAP,
        EVENT_DOUBLE_TAP,
        MAX_NUM_EVENTS
    } EventsBitmap_t;

    /** Constructor.
     */
    AccelerometerAdxl345(void);
    
    /** Destructor.
     */
    ~AccelerometerAdxl345(void);

    /** Initialise and configure the accelerometer chip.
     * @param pI2c a pointer to the I2C instance to use.
     * @param address 7-bit I2C address of the accelerometer.
     * @return true if successful, otherwise false.
     */
    bool init(I2C *pI2c, uint8_t address = ACCELEROMETER_ADXL345_ADDRESS);

    /** Call this to determine if an interrupt has gone off.
     */
    EventsBitmap_t handleInterrupt(void);
    
    /** Read the accelerometer.  All pointers may be NULL.
     * @param pX X axis data.
     * @param pY Y axis data.
     * @param pZ Z axis data.
     * @return true if the accelerometer was read successfully, otherwise false.
     */
    bool read(int16_t *pX, int16_t *pY, int16_t *pZ);
    
    /** Set the activity threshold that should trigger an interrupt.
     * @param threshold  the threshold to set.  The units are 62.5 mg steps.
     * @return true if the threshold was set successfully, otherwise false.
     */
    bool setActivityThreshold(uint8_t threshold);
    
    /** Check if interrupts are enabled.
     * @return true if interrupts are enabled, otherwise false.
     */
    bool areInterruptsEnabled(void);
    
    /** Enable interrupts.
     * @return true if interrupts were switched on successfully, otherwise false.
     */
    bool enableInterrupts(void);
    
    /** Disable interrupts.
     * @return true if interrupts were switched off successfully, otherwise false.
     */
    bool disableInterrupts(void);
    
protected:
    /** Pointer to the I2C interface. */
    I2C *gpI2c;
    /** The address of the device. */
    uint8_t gAddress;
    /** Flag to indicate device is ready */
    bool gReady;

    /** Read from registers.
    * Note: gpI2c should be locked before this is called.
    * @param address the address to start reading from.
    * @param pValues a place to put the returned values.
    * @param numValues the number of values to read.
    * @return true if successful, otherwise false.
    */
    bool getRegisters(char address, char *pValues, int numValues);

    /** For debug.
     * @param i2cAddress 7-bit I2C address of the device to read from.
     * @param reg the register to start reading from read.
     * @param numValues the number of bytes to read.
     * @return true if successful, otherwise false.
     */
    void readDeviceRegisters(char i2cAddress, char reg, int numValues);
};

#endif /* ACCELEROMETER_ADXL345_H */

/* End Of File */