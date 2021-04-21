#pragma once
#include <Stream.h>
#include <stdint.h>
#include <RingBuf.h>

using uint = unsigned int;

class DStarDV
{
public:
    static constexpr uint8_t DSTAR_FRAME_SIZE{12u};//packet size 9 voice plus 3 data
    static constexpr uint8_t SLOW_AMBE_BITSIZE{DSTAR_FRAME_SIZE * 8};
    static constexpr uint8_t DSTAR_MSG_SIZE{20};//Icom message on the display rotating
    static constexpr uint8_t DSTAR_MAX_FASTDATA_SIZE{28};
    static constexpr uint8_t BYTES_PER_PACKET_SLOW{5};//2+3
    static constexpr uint8_t BYTES_PER_PACKET_FAST{20};//10+10
    struct FrameData
    {
        uint8_t data[DSTAR_FRAME_SIZE];
        void fillEmpty()
        {
            memcpy(data, noVoice, DSTAR_FRAME_SIZE);
        }
    private:
        uint8_t noVoice[DSTAR_FRAME_SIZE] = {0x4d, 0xb2, 0x44, 0x12, 0x03, 0x68, 0x14, 0x64, 0x13, 0x66, 0x66, 0x66};//first 9 from real packet
    };
    void setDataOutput(Stream* outputStream);
    /**
    * @brief receiveData
    * @param buff - three bytes of DV slow data
    */
    void receiveData(uint8_t* buff);
    void receiveSyncData(uint8_t* buff);
    void reset();
    const uint8_t* getDStarMsg();
    void setMSG(const String& msg);
    void setDPRS(uint8_t* msg, uint size, bool useFastDV = false);
    FrameData getNextData();
    bool haveDStarMsg() const
    {
        return m_haveMsg;
    }
    inline bool isBufferFull()
    {
        return comBuffer.size() + 10 > comBuffer.maxSize();
    }
    inline bool isHalfBufferEmpty()
    {
        return comBuffer.size() > comBuffer.maxSize() / 2;
    }
    RingBuf<FrameData, 50> comBuffer;
private:
    Stream* m_outputStream{nullptr};
    uint8_t dStarMsg[DSTAR_MSG_SIZE + 1];
    uint posMSG{0};
    bool m_isEven{true};
    uint8_t dataLen{0};
    uint8_t dataType{0};
    uint dataCounter{21};
    bool m_haveMsg{false};
    void storeHeaderData(uint8_t* buff, bool isFirst);
    void sendPlainData(uint8_t* buff, bool isFirst);
    void sendFastData(uint8_t* buff, bool isFirst);
    void pushScrambled(FrameData data, bool whole = false);
    uint8_t m_fastDataPacket[2][DSTAR_FRAME_SIZE];
    uint8_t m_fastDataBuffer[DSTAR_MAX_FASTDATA_SIZE];
    uint8_t m_syncFrameData[DSTAR_FRAME_SIZE];
};
