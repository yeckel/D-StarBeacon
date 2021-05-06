#pragma once
#include <stdint.h>
#include "BitMatcher.h"
#include <FreeRTOS.h>
#include "SyncFrame.h"

using uint = unsigned int;

class BitSlicer
{
public:
    static constexpr uint HEADER_BITSIZE{660};
    static constexpr uint DATA_BITSIZE{96};//96bits == 12 bytes
    static constexpr uint DATA_FRAME_SIZE{DATA_BITSIZE / 8};
    static constexpr uint HEADER_SIZE{(HEADER_BITSIZE + 4) / 8 + 1}; //header 660 + 4 padding to full byte plus viterbi
    static constexpr uint8_t DSTAR_FRAME_SIZE{12u};

    BitSlicer(QueueHandle_t& rxQueue);
    void reset();
    void setHeaderBuffer(uint8_t* headerBuff, uint headerBuffSize);
    bool appendBit(bool bit);
    bool haveHeader();

private:
    BitMatcher tailMatcher;
    QueueHandle_t& rxQueue;
    bool receivedRFHeader{false};
    uint8_t receivedByte{0};

    uint8_t* m_headerBuff{nullptr};
    uint m_headerBuffSize{0};
    uint8_t m_dataBuff[DSTAR_FRAME_SIZE];
    uint8_t m_dataFrameCounter{0};
    uint8_t receivedHeaderByteNr{0};
    uint8_t receivedAmbeByteNr{0};
    uint8_t bitInRxByte{7};
    uint16_t dataFrameCount{0};
    uint16_t headerReceivedBits{0};
    bool processDataFrame();
};
