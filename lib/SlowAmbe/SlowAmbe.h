#pragma once
#include <stdint.h>
#include <RingBuf.h>

using uint = unsigned int;
class SlowAmbe
{
public:
    static constexpr uint8_t SLOW_AMBE_SIZE{12u};//packet size 9 voice plus 3 data
    static constexpr uint8_t SLOW_AMBE_BITSIZE{SLOW_AMBE_SIZE * 8};
    static constexpr uint8_t DSTAR_MSG_SIZE{20};//Icom message on the display rotating
    static constexpr uint8_t DSTAR_GPS_MAX_SIZE{100}; //"$$CRCB3BF,OK1CHP-1>API705,DSTAR*:!4946.70N/01329.42E-/A=001240Playing with ic-705"
    static constexpr uint8_t DSTAR_RF_SIZE{42};
    using AmbeData = uint32_t;//4 bytes will use just 3
    SlowAmbe();
    /**
    * @brief receiveData
    * @param buff - three bytes of DV slow data
    */
    void receiveData(uint8_t* buff);
    void reset();
    uint8_t* getDStarGPSData(uint16_t& size);
    uint8_t* getDStarMsg(uint16_t& size);//always 20
    uint8_t* getDStarRFHeader(uint16_t& size);
    void setMSG(uint8_t* msg);
    void setDPRS(uint8_t* msg, uint size);
    void getNextData(uint32_t& data);

    RingBuf<AmbeData, 80> comBuffer;
private:
    uint8_t dStarMsg[DSTAR_MSG_SIZE];
    uint8_t dStarGPS[DSTAR_GPS_MAX_SIZE];
    uint8_t dStarRFHeader[DSTAR_RF_SIZE];
    uint posMSG{0}, posGPS{0}, posRF{0};
    uint lineNr{0};
    uint8_t dataLen{0};
    uint8_t dataType{0};
    uint dataCounter{0};
    void storeData(uint8_t* buff, bool isFirst, uint8_t* dest, uint& pos);
};
