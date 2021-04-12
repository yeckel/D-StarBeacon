#include "SlowAmbe.h"
//#include <iostream>
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
    static constexpr uint8_t BYTES_PER_PACKET{5}; //2+3
    static constexpr uint32_t syncFrame{0x0068b4aa};//101010101011010001101000 every 1st and 21th !!DATA!! frame
}
uint8_t min(uint8_t a, uint8_t b)
{
    return a <= b ? a : b;
}
void SlowAmbe::storeHeaderData(uint8_t* buff, bool isFirst)
{
    if(m_haveMsg)
    {
        return;
    }
    uint8_t cnt = isFirst ?
                  min(dataLen, 2) ://first line max 2 bytes
                  min(dataLen, 3); //2nd line max 3 bytes
    uint8_t offset = isFirst ? 1 : 0;
    for(uint8_t i = 0; i < cnt; i++)
    {
        dStarMsg[posMSG] = buff[i + offset];
        posMSG++;
        m_haveMsg = posMSG == DSTAR_MSG_SIZE;
    }
}


void SlowAmbe::sendPlainData(uint8_t* buff, bool isFirst)
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

void SlowAmbe::pushScrambled(uint32_t data)
{
    uint8_t* p_data{(uint8_t*)& data};
    scrambleReverseOutput(p_data, 3);
    comBuffer.push(data);
}

void SlowAmbe::setDataOutput(Stream* outputStream)
{
    m_outputStream = outputStream;
}

void SlowAmbe::receiveData(uint8_t* buff)
{
    scrambleReverseInput(buff, 3);
    if(m_isEven)
    {
        dataLen = (buff[0] & PKT_LEN_MAP);
        dataType = (buff[0] & PKT_TYPE_MAP);
        switch(dataType)
        {
        case PKT_TYPE_MSG:
            dataLen = 5;
            //            std::cout << "T";
            storeHeaderData(buff, m_isEven);
            break;
        case PKT_TYPE_GPS:
            //            std::cout << "A";
            sendPlainData(buff, m_isEven);
            //            for(uint i = 0; i < 3; i++)
            //            {
            //                std::cout << "0x" << std::hex << int (buff[i]) << ", ";
            //            }
            break;
        case PKT_TYPE_HEADER:
            //            std::cout << "H";
            break;
        case PKT_TYPE_SQUELCH:
            //            std::cout << "C";
            break;
        case PKT_TYPE_FILL:
            break;
        default:
            Serial << "U";//unknown
            for(uint i = 0; i < 3; i++)
            {
                Serial << "0x" << _HEX(buff[i]) << ", ";
            }
            break;
        }
    }
    else
    {
        if(dataLen >= 2)
        {
            dataLen -= 2;
            switch(dataType)
            {
            case PKT_TYPE_MSG:
                //            std::cout << "t";
                storeHeaderData(buff, m_isEven);
                break;
            case PKT_TYPE_GPS:
                //            std::cout << "a";
                sendPlainData(buff, m_isEven);
                break;
            case PKT_TYPE_HEADER:
                //            std::cout << "h";
                break;
            case PKT_TYPE_SQUELCH:
                //            std::cout << "c";
                break;
            case PKT_TYPE_FILL:
                break;
            default:
                Serial << "u";//unknown
                for(uint i = 0; i < 3; i++)
                {
                    Serial << "0x" << _HEX(buff[i]) << ", ";
                }
                break;
            }
        }
    }
    //    Serial << "  :" << hex << int(buff[0]) << " " << int(buff[1]) << " " << int(buff[2]) << endl;
    m_isEven = !m_isEven;
}

void SlowAmbe::reset()
{
    memset(dStarMsg, 0x20, sizeof(dStarMsg));//spaces
    posMSG = 0;
    m_isEven = true;//0 is even
    dataCounter = 21;
    m_haveMsg = false;
}

const uint8_t* SlowAmbe::getDStarMsg()
{
    dStarMsg[sizeof(dStarMsg) - 1] = 0x00;//c-string end
    return dStarMsg;
}

void SlowAmbe::setMSG(const String& msg)
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

void SlowAmbe::setDPRS(uint8_t* msg, uint size)
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

void SlowAmbe::getNextData(uint32_t& data)
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
