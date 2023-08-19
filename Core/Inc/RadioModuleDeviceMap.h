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
#include "device/peripherals/LED/LED.h"

static const size_t MAP_SIZE = 5;


class RadioModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    /// @brief constructor
    RadioModuleDeviceMap(I2CDevice &gpsI2C, StreamDevice &gpsUART, GPIODevice &gpsReset, GPIODevice &gpsInterrupt,
            SPIDevice &wiznetSPI, GPIODevice &wiznetCS, GPIODevice &wiznetReset, GPIODevice &wiznetLED, NetworkLayer &wiznetUpper, Packet &wiznetPacket,
            SPIDevice &rfmSPI, GPIODevice &rfmCS, GPIODevice &rfmReset, GPIODevice &ledOneGPIO, GPIODevice &ledTwoGPIO)
                          : alloc::DeviceMap<MAP_SIZE>("Radio Module Device Map"), maxm10S(gpsI2C, gpsUART, gpsReset, gpsInterrupt), rfm9xw(rfmSPI, rfmCS, rfmReset),
                            w5500(wiznetSPI, wiznetCS, wiznetReset, wiznetLED, wiznetUpper, wiznetPacket), ledOne(ledOneGPIO), ledTwo(ledTwoGPIO) {};

    /// @brief initialize the Sensor Module specific map
    RetType init() {
        int ret = add("MAXM10S", &maxm10S);
//        ret += add("RFM9XW", &rfm9xw);
        ret += add("W5500", &w5500);
//        ret += add("LED_ONE", &ledOne);
//        ret += add("LED_TWO", &ledTwo);

        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    MAXM10S maxm10S;
    RFM9XW rfm9xw;
    Wiznet w5500;
    LED ledOne;
    LED ledTwo;
};

#endif // RADIO_MODULE_DEVICE_MAP_H
