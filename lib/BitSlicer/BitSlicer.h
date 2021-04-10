#pragma once
#include <stdint.h>
#include "BitMatcher.h"

using uint = unsigned int;

class BitSlicer
{
public:
    static constexpr uint HEADER_BITSIZE{660};
    static constexpr uint DATA_BITSIZE{96};//96bits == 12 bytes
    static constexpr uint DATA_FRAME_SIZE{DATA_BITSIZE / 8};
    static constexpr uint HEADER_SIZE{(HEADER_BITSIZE + 4) / 8 + 1}; //header 660 + 4 padding to full byte plus viterbi
    static constexpr uint8_t SLOW_AMBE_SIZE{12u};

    BitSlicer();
    void reset();
    void setHeaderBuffer(uint8_t* headerBuff, uint headerBuffSize);
    bool appendBit(bool bit);
    bool haveHeader();
    bool isEvenDataReady();
    bool isOddDataReady();
    uint8_t* getEvenData();
    uint8_t* getOddData();
private:
    BitMatcher tailMatcher;
    bool receivedRFHeader{false};
    uint8_t receivedByte{0};

    uint8_t* m_headerBuff{nullptr};
    uint m_headerBuffSize{0};
    uint8_t m_dataBuffEven[SLOW_AMBE_SIZE];
    uint8_t m_dataBuffOdd[SLOW_AMBE_SIZE];
    uint8_t* m_dataBuff = m_dataBuffEven;
    uint8_t m_dataFrameCounter{0};

    bool m_isOdd{false};
    bool m_evenReady{false};
    bool m_oddReady{false};
    uint m_dataBuffSize{0};

    uint8_t receivedHeaderByteNr{0};
    uint8_t receivedAmbeByteNr{0};
    uint8_t bitInRxByte{7};
    uint16_t dataFrameCount{0};
    uint16_t headerReceivedBits{0};
};
