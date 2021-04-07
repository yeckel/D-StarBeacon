#include "Scrambler.h"
#include <string.h>

namespace
{
    static constexpr uint8_t scrablerData[] = {0x70U, 0x4FU, 0x93U, 0x40U, 0x64U, 0x74U, 0x6DU, 0x30U, 0x2BU, 0x70U, 0x4FU, 0x93U};
    static constexpr uint8_t syncFrame[] = {0xaa, 0xb4, 0x68};//101010101011010001101000 every 1st and 21th !!DATA!! frame
}
bool isSyncFrame(uint8_t* buff)
{
    return buff[9] == syncFrame[0] &&
           buff[10] == syncFrame[1] &&
           buff[11] == syncFrame[2];
}

void getSyncFrame(uint8_t* buff)
{
    memset(buff, 0xFF, 9);
}

uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void scrambleReverseInput(uint8_t* buff, uint size)
{
    if(size > sizeof(scrablerData))
    {
        return;
    }
    for(uint8_t i = 0; i < size; i++)
    {
        uint8_t b_r = reverse(buff[i]);
        buff[i] = b_r ^ scrablerData[i];
    }
}

void scrambleReverseOutput(uint8_t* buff, uint size)
{
    if(size > sizeof(scrablerData))
    {
        return;
    }
    for(uint8_t i = 0; i < size; i++)
    {
        buff[i] = reverse(buff[i] ^ scrablerData[i]);
    }
}


