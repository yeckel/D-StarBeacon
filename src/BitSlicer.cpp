#include "BitSlicer.h"
#include <Streaming.h>
#include <string.h>

namespace
{
    static constexpr uint8_t endSyncData[] = {0xaa, 0xaa, 0xaa, 0xaa, 0x13, 0x5e};
}

bool BitSlicer::processDataFrame()
{
    if(m_dataFrameCounter == 0)
    {
        receivedAmbeByteNr = 0;
        if(!SyncFrame::isSyncFrame(m_dataBuff))
        {
            Serial << "Failed sync!";
            for(uint i = 0; i < DSTAR_FRAME_SIZE; i++)
            {
                Serial << _HEX(m_dataBuff[i]) << ",";
            }
            Serial << endl;
            return true;
        }
        //        Serial << "  SYNC" << endl;
    }
    else
    {
        receivedAmbeByteNr = 0;
    }
    xQueueSendFromISR(rxQueue, m_dataBuff, 0);
    //    Serial << " nr:" << _DEC(m_dataFrameCounter) << " ";
    m_dataFrameCounter = m_dataFrameCounter == 20 ? 0 : m_dataFrameCounter + 1;
    return false;
}

bool IRAM_ATTR BitSlicer::appendBit(bool bit)
{
    auto atEnd = tailMatcher.appendAndCheck(bit);
    if(atEnd)
    {
        //        Serial << "At end";
        return true;
    }
    receivedByte = (receivedByte & ~(1UL << bitInRxByte)) | (bit << bitInRxByte);
    if(!receivedRFHeader)
    {
        headerReceivedBits++;
    }
    if(bitInRxByte == 0 || headerReceivedBits == HEADER_BITSIZE)
    {
        bitInRxByte = 7;
        if(!receivedRFHeader)
        {
            receivedRFHeader = headerReceivedBits == HEADER_BITSIZE;
            //            cout << "bitNr:" << dec << headerReceivedBits << " m_headerBuff[" << dec << uint(receivedHeaderByteNr) << "]=" << hex << uint(receivedByte) << endl;
            m_headerBuff[receivedHeaderByteNr] = receivedByte;
            if(receivedRFHeader)
            {
                headerReceivedBits++;
                m_headerBuff[receivedHeaderByteNr] &= 0xF0;
                return false;
            }
            receivedHeaderByteNr++;
        }
        else
        {
            //            cout << (m_isOdd ? "m_dataBuffOdd[" : "m_dataBuffEven[") << dec << uint(receivedAmbeByteNr) << "]=" << hex << uint(receivedByte) << endl;
            m_dataBuff[receivedAmbeByteNr] = receivedByte;
            //            Serial << "0x" << _HEX(receivedByte) << ",";
            receivedAmbeByteNr++;
            if(receivedAmbeByteNr == 12)
            {
                return processDataFrame();
            }
        }
    }
    else
    {
        bitInRxByte--;
    }
    return false;
}

BitSlicer::BitSlicer(QueueHandle_t& rxQueue)
    : tailMatcher(endSyncData, sizeof(endSyncData)), rxQueue(rxQueue)
{
}

void BitSlicer::reset()
{
    dataFrameCount = 0;
    bitInRxByte = 7;
    receivedHeaderByteNr = 0;
    headerReceivedBits = 0;
    receivedRFHeader = false;
    receivedByte = 0;
    receivedAmbeByteNr = 0;
    m_dataFrameCounter = 0;
    tailMatcher.reset();
}

void BitSlicer::setHeaderBuffer(uint8_t* headerBuff, uint headerBuffSize)
{
    m_headerBuff = headerBuff;
    m_headerBuffSize = headerBuffSize;
}

bool BitSlicer::haveHeader()
{
    return headerReceivedBits >= HEADER_BITSIZE;
}
