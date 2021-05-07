#pragma once
#include <cstdint>
class SyncFrame
{
public:
    static constexpr uint8_t syncFrame[] = {0xaa, 0xb4, 0x68}; //101010101011010001101000 every 1st and 21th !!DATA!! frame
    static bool isSyncFrame(uint8_t* data)
    {
        return data[11] == syncFrame[2] &&
               data[10] == syncFrame[1] &&
               data[9] == syncFrame[0];
    }
};
