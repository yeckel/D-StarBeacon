#pragma once
#include <stdint.h>

class BitMatcher
{
public:
    BitMatcher(const uint8_t* data, unsigned int count);
    void reset();
    /**
     * @brief appendAndCheck appends bit to the mather and performs matching of its shift register
     * @param bit
     * @return true if pattern matches false if not
     */
    bool appendAndCheck(bool bit);
private:
    uint8_t* tailData;
    const uint8_t* endSyncData;
    unsigned int dataLength{0};
};
