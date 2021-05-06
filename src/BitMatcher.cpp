#include "BitMatcher.h"
#include <string.h>
#include <cstdlib>

bool BitMatcher::appendAndCheck(bool bit)
{
    for(int i = dataLength - 1; i >= 0; i--)
    {
        bool bitOver = (0b10000000 & tailData[i]) >> 7;
        tailData[i] = tailData[i] << 1 | (bit & 0b00000001);
        bit = bitOver;
    }
    return memcmp(tailData, endSyncData, dataLength) == 0;
}

BitMatcher::BitMatcher(const uint8_t* data, unsigned int count):
    dataLength(count)
{
    tailData = (uint8_t*)malloc(dataLength);
    reset();
    endSyncData = data;
}

void BitMatcher::reset()
{
    memset(tailData, 0x00, dataLength);
}
