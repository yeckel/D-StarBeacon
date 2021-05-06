#pragma once
#include <Stream.h>
#include <stdint.h>
#include <FreeRTOS.h>

using uint = unsigned int;

class DStarDV
{
public:
    static constexpr uint8_t DSTAR_VOICE_SIZE{9u};//packet size 9 voice plus 3 data
    static constexpr uint8_t DSTAR_FRAME_SIZE{12u};//packet size 9 voice plus 3 data
    static constexpr uint8_t SLOW_AMBE_BITSIZE{DSTAR_FRAME_SIZE * 8};
    static constexpr uint8_t DSTAR_MSG_SIZE{20};//Icom message on the display rotating
    static constexpr uint8_t DSTAR_MAX_FASTDATA_SIZE{28};
    static constexpr uint8_t BYTES_PER_PACKET_SLOW{5};//2+3
    static constexpr uint8_t BYTES_PER_PACKET_FAST{20};//10+10
    DStarDV(QueueHandle_t& txQueue): comBuffer(txQueue) {}
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
    * @param buff - three bytes of DV data
    */
    void receiveData(uint8_t* buff);
    void reset();
    const uint8_t* getDStarMsg();
    void setMSG(const String& msg);
    void setDPRS(uint8_t* msg, uint size, bool useFastDV = false);
    FrameData getNextData();
    bool haveDStarMsg() const
    {
        return m_haveMsg;
    }
    bool hasSpaceInBuffer(int queueSize)
    {
        //        m_wasBufferFull
        bool bufferAlmostEmpty = uxQueueMessagesWaiting(comBuffer) < 10;
        if(bufferAlmostEmpty)
        {
            m_bufferReady = true;
        }
        bool bufferAmostFull = uxQueueMessagesWaiting(comBuffer) + 10 > queueSize;
        if(bufferAmostFull)
        {
            m_bufferReady = false;
        }
        return m_bufferReady;
    }
    QueueHandle_t& comBuffer;
private:
    bool m_bufferReady{false};
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
    void splitDataSlow(uint8_t* msg, uint size);
    void splitDataFast(uint8_t* msg, uint size);
    void processDVData(uint8_t* buff);
    void processSyncData(uint8_t* buff);
};
