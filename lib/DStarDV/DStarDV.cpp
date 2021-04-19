#include "DStarDV.h"
#include <string.h>
#include <Streaming.h>
#include "Scrambler.h"

namespace
{
    static constexpr uint8_t PKT_LEN_MAP{0x0F};
    static constexpr uint8_t PKT_TYPE_MAP{0xF0};
    static constexpr uint8_t PKT_TYPE_MSG{0x40};//does not have a size
    static constexpr uint8_t PKT_TYPE_GPS{0x30};
    static constexpr uint8_t PKT_TYPE_HEADER{0x50};//repeatet RF header
    static constexpr uint8_t PKT_TYPE_SQUELCH{0x20};
    static constexpr uint8_t PKT_TYPE_FILL{0x60};
    static constexpr uint8_t FILLER_BYTE{0x66};
    static constexpr uint8_t FAST_DATA_SHORT{0x80}; //fast data short
    static constexpr uint8_t FAST_DATA_LONG{0x90}; //fast data long
    static constexpr uint8_t BYTES_PER_PACKET{5}; //2+3
    static constexpr uint32_t syncFrame{0x0068b4aa};//101010101011010001101000 every 1st and 21th !!DATA!! frame
}
uint8_t min(uint8_t a, uint8_t b)
{
    return a <= b ? a : b;
}
void DStarDV::storeHeaderData(uint8_t* buff, bool isFirst)
{
    if(m_haveMsg)
    {
        return;
    }
    uint8_t cnt = isFirst ?
                  2 ://first line max 2 bytes
                  3; //2nd line max 3 bytes
    uint8_t offset = isFirst ? 1 : 0;
    for(uint8_t i = 0; i < cnt; i++)
    {
        dStarMsg[posMSG] = buff[i + offset];
        posMSG++;
        m_haveMsg = posMSG == DSTAR_MSG_SIZE;
    }
}


void DStarDV::sendPlainData(uint8_t* buff, bool isFirst)
{
    uint8_t cnt = isFirst ?
                  min(dataLen, 2) ://first line max 2 bytes
                  min(dataLen, 3); //2nd line max 3 bytes
    uint8_t offset = isFirst ? 1 : 0;
    for(uint8_t i = 0; i < cnt; i++)
    {
        uint8_t ch = buff[i + offset];
        if(m_outputStream)
        {
            m_outputStream->write(ch);
        }
        //        Serial << "0x" << _HEX(ch) << " ";
    }
}

void DStarDV::sendFastData(uint8_t* buff, bool isFirst)
{
    memcpy(m_fastDataPacket[isFirst ? 0 : 1], buff, DSTAR_FRAME_SIZE);
    if(isFirst)
    {
        return;
    }
    auto controlByte = m_fastDataPacket[0][9];
    bool isLong = (controlByte & 0xF0) == FAST_DATA_LONG;
    uint8_t fastDataSize = (m_fastDataPacket[0][9] & 0x0F) + (isLong ? 16 : 0);
    memcpy(m_fastDataBuffer + 0, m_fastDataPacket[0] + 10, 2);
    memcpy(m_fastDataBuffer + 2, m_fastDataPacket[1] + 10, 2);
    memcpy(m_fastDataBuffer + 4, m_fastDataPacket[0] + 0, 4);
    memcpy(m_fastDataBuffer + 8, m_fastDataPacket[0] + 5, 4);
    memcpy(m_fastDataBuffer + 12, m_fastDataPacket[1] + 0, 4);
    memcpy(m_fastDataBuffer + 16, m_fastDataPacket[1] + 5, 4);
    memcpy(m_fastDataBuffer + 20, m_syncFrameData, 4);//skipping 5th byte which is 0x02
    memcpy(m_fastDataBuffer + 24, m_syncFrameData + 5, 4);
    Serial << endl << "s:" << uint(fastDataSize) << " d:";
    for(uint i = 0; i < min(DSTAR_MAX_FASTDATA_SIZE, fastDataSize); i++)
    {
        char ch = m_fastDataBuffer[i];
        Serial << "0x" << _HEX(ch) << ", ";
        if(m_outputStream)
        {
            m_outputStream->write(ch);
        }
    }
    Serial << endl;
}

void DStarDV::pushScrambled(uint32_t data)
{
    uint8_t* p_data{(uint8_t*)& data};
    scrambleReverseOutput(p_data, 3);
    comBuffer.push(data);
}

void DStarDV::setDataOutput(Stream* outputStream)
{
    m_outputStream = outputStream;
}

void DStarDV::receiveData(uint8_t* buff)
{
    //    scrambleReverseInput(buff + 9, 3);
    scrambleReverseInput(buff, DSTAR_FRAME_SIZE);
    Serial << "Data " << (m_isEven ? "e" : "o") << ":";
    for(uint i = 0; i < DSTAR_FRAME_SIZE; i++)
    {
        Serial << "0x" << _HEX(buff[i]) << ", ";
    }
    //    Serial << "   ";
    if(m_isEven)
    {
        dataLen = (buff[9] & PKT_LEN_MAP);
        dataType = (buff[9] & PKT_TYPE_MAP);
        switch(dataType)
        {
        case PKT_TYPE_MSG:
            Serial << "T ";
            storeHeaderData(buff + 9, m_isEven);
            break;
        case PKT_TYPE_GPS:
            Serial << "A ";
            sendPlainData(buff + 9, m_isEven);
            dataLen = dataLen > 2 ? dataLen - 2 : 0; //first frame has max 2 bytes payload
            break;
        case PKT_TYPE_HEADER:
            Serial << "H ";
            break;
        case PKT_TYPE_SQUELCH:
            Serial << "C ";
            break;
        case PKT_TYPE_FILL:
            Serial << "F ";
            break;
        case FAST_DATA_LONG:
            Serial << "E ";
        case FAST_DATA_SHORT:
            Serial << "D ";
            //            scrambleReverseInput(buff, 9);//last 3 bytes already scrambled
            sendFastData(buff, m_isEven);
            break;
        default:
            Serial << "U ";//unknown
            break;
        }
    }
    else
    {
        switch(dataType)
        {
        case PKT_TYPE_MSG:
            Serial << "t ";
            storeHeaderData(buff + 9, m_isEven);
            break;
        case PKT_TYPE_GPS:
            Serial << "a ";
            sendPlainData(buff + 9, m_isEven);
            break;
        case PKT_TYPE_HEADER:
            Serial << "h ";
            break;
        case PKT_TYPE_SQUELCH:
            Serial << "c ";
            break;
        case PKT_TYPE_FILL:
            Serial << "f ";
            break;
        case FAST_DATA_LONG:
            Serial << "e ";
        case FAST_DATA_SHORT:
            Serial << "d ";
            //            scrambleReverseInput(buff, 9);//last 3 bytes already scrambled
            sendFastData(buff, m_isEven);
            break;
        default:
            Serial << "u ";//unknown
            break;
        }
    }
    //    Serial << "  :" << hex << int(buff[0]) << " " << int(buff[1]) << " " << int(buff[2]) << endl;
    Serial << endl;
    m_isEven = !m_isEven;
}

void DStarDV::receiveSyncData(uint8_t* buff)
{
    Serial << "Sync:";
    memcpy(m_syncFrameData, buff, DSTAR_FRAME_SIZE);
    scrambleReverseInput(m_syncFrameData, DSTAR_FRAME_SIZE - 3);
    m_isEven = true; //after sync comms always even frame
    for(uint i = 0; i < DSTAR_FRAME_SIZE - 3; i++)
    {
        Serial << "0x" << _HEX(m_syncFrameData[i]) << ", ";
    }
    Serial << endl;
}

void DStarDV::reset()
{
    memset(dStarMsg, 0x20, sizeof(dStarMsg));//spaces
    posMSG = 0;
    m_isEven = true;//0 is even
    dataCounter = 21;
    m_haveMsg = false;
}

const uint8_t* DStarDV::getDStarMsg()
{
    dStarMsg[sizeof(dStarMsg) - 1] = 0x00;//c-string end
    return dStarMsg;
}

void DStarDV::setMSG(const String& msg)
{
    uint8_t buff[20];
    memset(buff, 0x20, sizeof(buff));
    memcpy(buff, msg.c_str(), min(sizeof(buff), msg.length()));
    uint32_t data{0x6666600u};
    uint8_t* p_data = (uint8_t*)&data;
    for(uint8_t i = 0; i < 4; i++)
    {
        auto offsetInMsg = BYTES_PER_PACKET * i;
        p_data[0] = PKT_TYPE_MSG | (0xFF & i);
        p_data[1] = msg[0 + offsetInMsg];
        p_data[2] = msg[1 + offsetInMsg];
        //        Serial << _HEX(data) << endl;
        pushScrambled(data);
        p_data[0] = msg[2 + offsetInMsg];
        p_data[1] = msg[3 + offsetInMsg];
        p_data[2] = msg[4 + offsetInMsg];
        pushScrambled(data);
        //        Serial << _HEX(data) << endl;
    }
}

void DStarDV::setDPRS(uint8_t* msg, uint size)
{
    uint neededPackets = size / BYTES_PER_PACKET;
    if(size % BYTES_PER_PACKET)
    {
        neededPackets += 1;
    }
    uint32_t data{0x66666600u};
    uint8_t* p_data = (uint8_t*)&data;
    //    Serial << "Needed packets" << neededPackets << endl;
    for(uint i = 0; i < neededPackets; i++)
    {
        auto offsetInMsg = BYTES_PER_PACKET * i;
        uint8_t bytesInPkt = (i + 1 < neededPackets) ?
                             BYTES_PER_PACKET :
                             size - (neededPackets - 1) * BYTES_PER_PACKET ;

        p_data[0] = PKT_TYPE_GPS | (0xFF & bytesInPkt);
        p_data[1] = msg[0 + offsetInMsg];
        if(bytesInPkt >= 2)
        {
            p_data[2] = msg[1 + offsetInMsg];
        }
        pushScrambled(data);
        //        Serial << _HEX(data) << endl;
        data = 0x66666600u;
        if(bytesInPkt >= 3)
        {
            p_data[0] = msg[2 + offsetInMsg];
        }
        if(bytesInPkt >= 4)
        {
            p_data[1] = msg[3 + offsetInMsg];
        }
        if(bytesInPkt == 5)
        {
            p_data[2] = msg[4 + offsetInMsg];
        }
        pushScrambled(data);
        //        Serial << _HEX(data) << endl;
    }
    //    Serial << "Size" << comBuffer.size() << endl;
}

void DStarDV::getNextData(uint32_t& data)
{
    //first and every 21st packet is sync one
    if(dataCounter  == 21)
    {
        dataCounter = 0;
        data = syncFrame;
        //        Serial << "Sync" << endl;
    }
    else
    {
        comBuffer.lockedPop(data);
        //        Serial << "Data" << endl;
    }
    dataCounter++;
}
