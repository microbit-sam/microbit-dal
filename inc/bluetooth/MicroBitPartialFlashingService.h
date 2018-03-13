/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef MICROBIT_PARTIAL_FLASH_SERVICE_H
#define MICROBIT_PARTIAL_FLASH_SERVICE_H

#include "MicroBitConfig.h"
#include "MicroBitBLEManager.h"
#include "ble/BLE.h"
#include "MicroBitMemoryMap.h"

#include "MicroBitFlash.h"
#include "MicroBitStorage.h"

#include "MicroBitComponent.h"
#include "MicroBitEvent.h"
#include "MicroBitListener.h"
#include "EventModel.h"

#define PARTIAL_FLASHING_VERSION 0x01

// UUIDs for our service and characteristics
extern const uint8_t  MicroBitPartialFlashingServiceUUID[];
extern const uint8_t  MicroBitPartialFlashingServiceCharacteristicUUID[];

/**
  * Class definition for the custom MicroBit Partial Flash Service.
  * Provides a BLE service to remotely read the memory map and flash the PXT program.
  */
class MicroBitPartialFlashingService
{
    public:
    /**
      * Constructor.
      * Create a representation of the Partial Flash Service
      * @param _ble The instance of a BLE device that we're running on.
      * @param _memoryMap An instance of MicroBiteMemoryMap to interface with.
      */
    MicroBitPartialFlashingService(BLEDevice &_ble, EventModel &_messageBus);

    /**
      * Callback. Invoked when any of our attributes are written via BLE.
      */
    void onDataWritten(const GattWriteCallbackParams *params);

    /**
      * Callback. Invoked when any of our attributes are read via BLE.
      */
    void onDataRead(GattReadAuthCallbackParams *params);

    private:
    // M:B Bluetooth stack and MessageBus
    BLEDevice           &ble;
    EventModel          &messageBus;

    /**
      * Writing to flash inside MicroBitEvent rather than in the ISR
      */
    void partialFlashingEvent(MicroBitEvent e);

    // The base address to write to. Bit masked:  (0xFFFF0000 & region.endAddress) >> 16
    uint8_t baseAddress = 0x3;

    // Handles to access each characteristic when they are held by Soft Device.
    GattAttribute::Handle_t partialFlashCharacteristicHandle;

    /**
      * Process a Partial Flashing data packet
      */
    void flashData(uint8_t *data);

};


#endif
