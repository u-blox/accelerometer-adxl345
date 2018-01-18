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

/**
 * @file accelerometer_adxl345.cpp
 * This file defines an API to the Analog Devices ADXL345 accelerometer chip.
 */

// Define this to print debug information
//#define DEBUG_ADXL345

#include <mbed.h>
#include <accelerometer_adxl345.h>

#ifdef DEBUG_ADXL345
# include <stdio.h>
#endif

#include "mbed.h"
#include "accelerometer_adxl345.h"

// ----------------------------------------------------------------
// PROTECTED FUNCTIONS
// ----------------------------------------------------------------

// Debug.
void AccelerometerAdxl345::readDeviceRegisters(char i2cAddress, char reg, int numValues)
{
    char *pData = new char[numValues];

    if (gpI2c != NULL) {
        printf("Device 0x%02x: ", i2cAddress >> 1);
        // Move the address pointer
        if (gpI2c->write(i2cAddress, &reg, 1) == 0) {
           // Read from the address
            if (gpI2c->read(i2cAddress, pData, numValues) == 0) {
                printf("read %d value(s):\n", numValues);
                printf("%02x: ", (int) reg);
                for (int x = 0; x < numValues; x++) {
                    printf("%02x", (int) (pData + x));
                    if ((x + 1) < numValues) {
                        printf(", ");
                    }
                }
                printf(".\n");
            } else {
                printf("unable to read %d value(s) from register 0x%02x.\n", numValues, reg);
            }
        } else {
            printf("unable to write address of register 0x%02x.\n", reg);
        }
    }
    
    free(pData);
}

// Read from a register.
bool AccelerometerAdxl345::getRegisters(char address, char *pValues, int numValues)
{
    bool success = false;

    if (gpI2c != NULL) {
        // Move the address pointer
        if (gpI2c->write(gAddress, &address, 1) == 0) {
            if (pValues != NULL) {
               // Read from the address
                if (gpI2c->read(gAddress, pValues, numValues) == 0) {
                    success = true;
#ifdef DEBUG_ADXL345
                    printf("AccelerometerAdxl345 (I2C 0x%02x): read %d byte(s) from register 0x%02x.\n", gAddress >> 1, numValues, address);
#endif
                }
            } else {
                success = true;
            }
        }
    }

    return success;
}

// ----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------

// Constructor.
AccelerometerAdxl345::AccelerometerAdxl345(void)
{
    gpI2c = NULL;
    gAddress = 0;
    gReady = false;
}

// Destructor.
AccelerometerAdxl345::~AccelerometerAdxl345(void)
{
}

// Initial setup of the accelerometer.
bool AccelerometerAdxl345::init(I2C *pI2c, uint8_t address)
{
    char data[2];

    gpI2c = pI2c;
    gAddress = address << 1;
    gReady = false;
    
    data[0] = 0;
    // Reading register 0x00 should get us back 0xe5
    if (getRegisters(0x00, &(data[0]), 1)) {
        if (data[0] == 0xe5) {
            gReady = true;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345 is connected at I2C address 0x%02x.\n", gAddress >> 1);
#endif
            // Set up the interrupts: activity only
            data[0] = 0x2e;
            data[1] = 0x00;  // Disable all interrupts for the moment
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

            data[0] = 0x24;  // Activity threshold
            data[1] = 0x10;  // Low
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

            data[0] = 0x27;  // Activity/inactivity control
            data[1] = 0xf0;  // Compare changes, all axes participating
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set activity/inactivity register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

            data[0] = 0x2c; // Measurement rate
            data[1] = 0x07; // The lowest rate
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set measurement rate register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

            data[0] = 0x2e;
            data[1] = 0x10;   // Enable the activity interrupt
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set interrupt enable register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

            data[0] = 0x2d; // The power control register
            data[1] = 0x08; // Measurement mode
            if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
                gReady = false;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                       gAddress >> 1, data[0], data[1]);
#endif
            }

        } else {
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, reading value of register 0x00 got 0x%02x but expected 0xe5.\n",
                   gAddress >> 1, data[0]);
#endif
        }
    } else {
#ifdef DEBUG_ADXL345
        printf("AccelerometerAdxl345: I2C address 0x%02x, unable to get the contents of register 0x00.\n", gAddress >> 1);
#endif
    }
    
    return gReady;
}

// Call this to determine if an interrupt has gone off.
AccelerometerAdxl345::EventsBitmap_t AccelerometerAdxl345::handleInterrupt(void)
{
    AccelerometerAdxl345::EventsBitmap_t eventsBitmap = AccelerometerAdxl345::EVENT_NONE;
    char address = 0x30;
    char eventReg;

    // Read what happened
    if (gReady) {
        // Move the address pointer
        if (gpI2c->write(gAddress, &address, 1) == 0) {
           // Read from the address
            if (gpI2c->read(gAddress, &eventReg, 1) == 0) {
                // Activity
                if (eventReg & 0x10) {
                    eventsBitmap = (AccelerometerAdxl345::EventsBitmap_t) (eventsBitmap | AccelerometerAdxl345::EVENT_ACTIVITY);
                }
            
                // Double tap
                if (eventReg & 0x20) {
                    eventsBitmap = (AccelerometerAdxl345::EventsBitmap_t) (eventsBitmap | AccelerometerAdxl345::EVENT_DOUBLE_TAP);
                }
            
                // Single tap
                if (eventReg & 0x40) {
                    eventsBitmap = (AccelerometerAdxl345::EventsBitmap_t) (eventsBitmap | AccelerometerAdxl345::EVENT_SINGLE_TAP);
                }
            }
        }
    }

    return eventsBitmap;
}

// Read the accelerometer.
bool AccelerometerAdxl345::read(int16_t *pX, int16_t *pY, int16_t *pZ)
{
    bool success = false;
    char data[6];
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    if (gReady) {
        data[0] = 0x2d; // The power control register
        data[1] = 0x08; // Measurement mode
        if (gpI2c->write(gAddress, data, 2) == 0) {
            wait_ms(10);
            if (getRegisters(0x32, &(data[0]), sizeof (data))) {
                success = true;
                x = data[0] + ((int16_t) data[1] << 8);
                y = data[2] + ((int16_t) data[3] << 8);
                z = data[4] + ((int16_t) data[5] << 8);
                if (pX != NULL) {
                    *pX = x;
                }
                if (pY != NULL) {
                    *pY = y;
                }
                if (pZ != NULL) {
                    *pZ = z;
                }
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: x %d, y %d, z %d.\n", x, y, z);
#endif
            } else {
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345: I2C address 0x%02x, unable to read from register 0x32.\n", gAddress >> 1);
#endif
            }
        } else {
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }
    }

    return success;
}

// Set the activity threshold for an interrupt to be triggered.
bool AccelerometerAdxl345::setActivityThreshold(uint8_t threshold)
{
    bool success = false;
    char data[2];

    if (gReady) {
        data[0] = 0x24;  // Activity threshold
        data[1] = (char) threshold;
        if (gpI2c->write(gAddress, data, 2) == 0) {
            success = true;
        } else {
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }
    }

    return success;
}

// Check if interrupt are enabled.
bool AccelerometerAdxl345::areInterruptsEnabled(void)
{
    bool areEnabled = false;
    char value = 0;

    if (gReady) {
        // Read register 0x2E
        if (getRegisters(0x2e, &value, sizeof (value)) == 0) {
            if (value != 0x00) {
                areEnabled = true;
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345 I2C address 0x%02x, interrupts are enabled (0x%02x).\n", gAddress >> 1, value);
#endif
            } else {
#ifdef DEBUG_ADXL345
                printf("AccelerometerAdxl345 I2C address 0x%02x, interrupts are disabled.\n", gAddress >> 1);
#endif
            }
        } else {
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to read from register 0x2e.\n", gAddress >> 1);
#endif
        }
    }

    return areEnabled;
}

// Enable interrupts from the accelerometer.
bool AccelerometerAdxl345::enableInterrupts(void)
{
    bool success = true;
    char data[2];

    if (gReady) {
        // Set up the interrupts: activity only
        data[0] = 0x2e;
        data[1] = 0x00;  // Disable all interrupts for the moment
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }

        data[0] = 0x24;  // Activity threshold
        data[1] = 0x10;  // Low
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }

        data[0] = 0x27;  // Activity/inactivity control
        data[1] = 0xf0;  // Compare changes, all axes participating
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set activity/inactivity register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }

        data[0] = 0x2c; // Measurement rate
        data[1] = 0x07; // The lowest rate
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set measurement rate register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }

        // Call this just to clear any interrupts
        handleInterrupt();

        data[0] = 0x2e;
        data[1] = 0x10;   // Enable the activity interrupt
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set interrupt enable register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }

        data[0] = 0x2d; // The power control register
        data[1] = 0x08; // Measurement mode
        if (gpI2c->write(gAddress, data, sizeof(data)) != 0) {
            success = false;
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }
    }
    
    return success;
}

// Disable interrupts from the accelerometer.
bool AccelerometerAdxl345::disableInterrupts(void)
{
    bool success = false;
    char data[2];

     if (gReady) {
        data[0] = 0x2e;
        data[1] = 0x00;  // Disable all interrupts
        if (gpI2c->write(gAddress, data, sizeof(data)) == 0) {
            success = true;
        } else {
#ifdef DEBUG_ADXL345
            printf("AccelerometerAdxl345: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                   gAddress >> 1, data[0], data[1]);
#endif
        }
     }

    return success;
}