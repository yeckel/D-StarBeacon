#include "BitSlicer.h"
#include <Streaming.h>
#include <string.h>

namespace
{
    static constexpr uint8_t endSyncData[] = {0xaa, 0xaa, 0xaa, 0xaa, 0x13, 0x5e};
    static constexpr uint8_t syncFrame[] = {0xaa, 0xb4, 0x68}; //101010101011010001101000 every 1st and 21th !!DATA!! frame
}

bool BitSlicer::appendBit(bool bit)
{
    auto atEnd = tailMatcher.appendAndCheck(bit);
    if(atEnd)
    {
        Serial << "At end";
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
            receivedAmbeByteNr++;
            if(receivedAmbeByteNr == 12)
            {
                if(m_dataFrameCounter == 0)
                {
                    bool isSyncVrame = m_dataBuff[11] == syncFrame[2] &&
                                       m_dataBuff[10] == syncFrame[1] &&
                                       m_dataBuff[9] == syncFrame[0];
                    receivedAmbeByteNr = 0;
                    if(!isSyncVrame)
                    {
                        Serial << endl << "Sync frame" << endl;
                        return true;
                    }
                }
                else
                {
                    m_evenReady |= !m_isOdd;
                    m_oddReady |= m_isOdd;
                    //start storing to the 2nd buffer
                    m_isOdd = !m_isOdd;
                    m_dataBuff = m_isOdd ? m_dataBuffOdd : m_dataBuffEven;
                    receivedAmbeByteNr = 0;
                }
                m_dataFrameCounter = m_dataFrameCounter == 20 ? 0 : m_dataFrameCounter + 1;
            }
        }
    }
    else
    {
        bitInRxByte--;
    }
    return false;
}

BitSlicer::BitSlicer()
    : tailMatcher(endSyncData, sizeof(endSyncData))
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
    m_isOdd = false;
    m_evenReady = false;
    m_oddReady = false;
    m_dataBuff = m_dataBuffEven;
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

bool BitSlicer::isEvenDataReady()
{
    return  m_evenReady;
}

bool BitSlicer::isOddDataReady()
{
    return m_oddReady;
}

uint8_t* BitSlicer::getEvenData()
{
    m_evenReady = false;
    return m_dataBuffEven;
}

uint8_t* BitSlicer::getOddData()
{
    m_oddReady = false;
    return m_dataBuffOdd;
}
