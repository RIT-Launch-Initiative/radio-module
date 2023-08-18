/**
 * Device Map For Radio Module
 *
 * @author Aaron Chan
 */

#ifndef RADIO_MODULE_DEVICE_MAP_H
#define RADIO_MODULE_DEVICE_MAP_H

#include "device/DeviceMap.h"
#include "sched/macros.h"
#include "return.h"

#include "device/GPIODevice.h"
#include "device/StreamDevice.h"
#include "device/I2CDevice.h"
#include "device/SPIDevice.h"

#include "device/peripherals/MAXM10S/MAXM10S.h"
#include "device/peripherals/wiznet/wiznet.h"
#include "device/peripherals/RFM9XW/RFM9XW.h"
#include "device/peripherals/W25Q/W25Q.h"
#include "device/peripherals/LED/LED.h"

static const size_t MAP_SIZE = 15;


class RadioModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    /// @brief constructor
    RadioModuleDeviceMap(I2CDevice &i2cDevice, SPIDevice &wiznetSPI, SPIDevice &flashSPI,
                          GPIODevice &wiznetCS, GPIODevice &flashCS, GPIODevice &ledOneGPIO,
                          GPIODevice &ledTwoGPIO, GPIODevice &wiznetLEDGPIO, StreamDevice &uartDebug, StreamDevice &uartGPS)
                          : alloc::DeviceMap<MAP_SIZE>("Radio Module Device Map") {};

    /// @brief initialize the Sensor Module specific map
    RetType init() {

        RetType ret;


        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    MAXM10S maxm10S;
    RFM9XW rfm9xw;
    Wiznet w5500;
    W25Q w25q;
    LED ledOne;
    LED ledTwo;
};

#endif // RADIO_MODULE_DEVICE_MAP_H
