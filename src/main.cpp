//For T-Beam with AXP202
#include <Arduino.h>
#include <SSD1306Wire.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <time.h>
#include <sys/time.h>
#include <BluetoothSerial.h>
#include <pins_arduino.h>
#include <Streaming.h>
#include <axp20x.h>
#include <TimeLib.h>
#include <SPI.h>
#include <DSTAR.h>
#include <SlowAmbe.h>
#include <Scrambler.h>
#include <BitSlicer.h>


#define LORA_CS 18      // GPIO18 - SX1276 CS
#define LORA_RST 23     // GPIO23 - SX1276 RST
#define LORA_IRQ 26     // GPIO26 - SX1276 IO0
#define LORA_IO0 LORA_IRQ  // alias
#define LORA_IO1 33     // GPIO33 - SX1276 IO1 -> wired on pcb AND connected to header pin LORA1
#define LORA_IO2 32     // GPIO32 - SX1276 IO2 -> wired on pcb AND connected to header pin LORA2

const int offset = 2 * 3600; //CEST

SX1278 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_IO1);
MorseClient morse(&radio);
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_64);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
BluetoothSerial serialBT;
AXP20X_Class axp;
TaskHandle_t BeaconSenderHandler;

void checkLoraState(int state)
{
    if(state != ERR_NONE)
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while(true);
    }
}

uint8_t preambleAndBitSync[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x76, 0x50};
static constexpr uint16_t PREAMBLE_BITSIZE{sizeof(preambleAndBitSync) * 8};
volatile uint preambleBitPos{0};
uint8_t syncWord[] = {0xaa, 0xaa, 0xaa, 0xaa, 0xEC, 0xA0};
uint8_t stoppingFrame[] = {0xaa, 0xaa, 0xaa, 0xaa, 0x13, 0x5e};
static constexpr uint16_t STOPPING_FRAME_BITSIZE{sizeof(stoppingFrame) * 8};
volatile uint stoppingFramePos{0};
uint8_t plainDataBTRXBuff[5];

uint8_t DSTAR_MSG[SlowAmbe::DSTAR_MSG_SIZE] = {'D', '-', 'S', 't', 'a', 'r', ' ', 'b', 'e', 'a', 'c', 'o', 'n', ' ', 'e', 's', 'p', '3', '2', '!'};

uint8_t dStarTxHeaderData[DSTAR::RF_HEADER_SIZE] = {0x0, 0x0, 0x0, //flag 1,2,3
                                                    0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //destination callsign
                                                    0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //repeater callsign
                                                    0x43, 0x51, 0x43, 0x51, 0x43, 0x51, 0x20, 0x20, //companion callsign
                                                    'O', 'K', '1', 'C', 'H', 'P', 0x20, 0x20, //own callsign
                                                    '7', '0', '5', ' ', //rig ID
                                                    0x00, 0x00, //place for CRC
                                                    0x0 //padding
                                                   };
uint8_t dStarRxHeaderData[DSTAR::RF_HEADER_SIZE];

uint8_t headerRXTXBuffer[DSTAR::RF_HEADER_SIZE * 2];
volatile uint headerBitPos{0};
uint8_t history[DSTAR::RF_HEADER_SIZE * 8];  //buffer for viterbi decoding (large buffer need)

uint8_t payloadData[SlowAmbe::SLOW_AMBE_SIZE] = {0x4d, 0xb2, 0x44, 0x12, 0x03, 0x68, 0x14, 0x64, 0x13, 0x66, 0x66, 0x66};//first 9 from real packet

SlowAmbe sa;
DSTAR dStar;
BitSlicer bs;

volatile uint ambeDataBitPos{0};
bool isFirstAmbe{true};
volatile bool isPTTPressed{false};
volatile bool stopTx{false};
bool bluetoothXOFF{false};
volatile bool inSync = false;
volatile bool receivedPacket{false};

float f = 434.800f + 0.00244f;//t-beam sx1278 has XTAL offset

//----------------------------------TX bit routines -----------------------
void sendBit(uint8_t* sendBuff, uint buffBitPos)
{
    auto bytePos = buffBitPos / 8;
    auto bitPos = buffBitPos % 8;
    bool bit = sendBuff[bytePos] & (0b10000000 >> bitPos);
    //    Serial << bit;
    digitalWrite(LORA_IO2, bit);
}

void sendPreambleBit()
{
    //        Serial << "P";
    sendBit(preambleAndBitSync, preambleBitPos);
    preambleBitPos++;
}

void sendHeaderBit()
{
    //        Serial << "H";
    sendBit(headerRXTXBuffer, headerBitPos);
    headerBitPos++;
}

void fetchNextPayloadData()
{
    uint32_t data;
    uint8_t* p_data{(uint8_t*)& data};
    sa.getNextData(data);
    memcpy(payloadData + 9, p_data, 3); //using just last 3 bytes in slowAmbeData and 3 first bytes from data
    ambeDataBitPos = 0;
    isFirstAmbe = false;
}

void sendPayloadBit()
{
    if(ambeDataBitPos == SlowAmbe::SLOW_AMBE_BITSIZE ||
       isFirstAmbe)
    {
        fetchNextPayloadData();
    }
    //        Serial << "A";
    sendBit(payloadData, ambeDataBitPos);
    ambeDataBitPos++;
}

void sendStoppingFrameBit()
{
    //        Serial << "S";
    sendBit(stoppingFrame, stoppingFramePos);
    stoppingFramePos++;
}

void txBit()
{
    if(preambleBitPos < PREAMBLE_BITSIZE)
    {
        sendPreambleBit();
    }
    else if(headerBitPos < DSTAR::RF_HEADER_TRANSFER_BITSIZE)
    {
        sendHeaderBit();
    }
    else if(!sa.comBuffer.isEmpty() ||
            ambeDataBitPos < SlowAmbe::SLOW_AMBE_BITSIZE)
    {
        sendPayloadBit();
    }
    else if(stoppingFramePos < STOPPING_FRAME_BITSIZE)
    {
        sendStoppingFrameBit();
    }
    else
    {
        stopTx = true;
    }
}
//-------------------------------RX bit routines----------------------------------------
void rxBit()
{
    auto receivedBit = digitalRead(LORA_IO2);
    if(!inSync)
    {
        return;
    }
    //    Serial << receivedBit;
    auto commStopped = bs.appendBit(receivedBit);
    if(commStopped)
    {
        Serial << endl << "RX Stopped" << endl;
        inSync = false;
        digitalWrite(BUILTIN_LED, false);
        receivedPacket = true;
    }
}
//--------------------------------Interrupt handlers-------------------------------------
void dataClockInterruptHandler()
{
    if(isPTTPressed)
    {
        txBit();
    }
    else
    {
        rxBit();
    }
}

void receivedSyncWord(void)
{
    inSync = true;
    Serial << __FUNCTION__ << endl;
}
//----------------------------------------------------------------------------------------
void prepareHeader()
{
    dStar.add_crc(dStarTxHeaderData);
    dStar.convolution(dStarTxHeaderData, headerRXTXBuffer);   //source, dest
    dStar.interleave(headerRXTXBuffer);
    dStar.pseudo_random(headerRXTXBuffer, DSTAR::RF_HEADER_TRANSFER_BITSIZE);
}

void decodeHeader(uint8_t* bufferConv)
{
    dStar.pseudo_random(bufferConv, 660);
    dStar.deInterleave(bufferConv);
    dStar.viterbi(bufferConv, history, dStarRxHeaderData); //decode data
    Serial << endl;

    Serial <<  endl <<  "Nb errors:";
    Serial << int(dStar.acc_error[1][0]) << endl;    //print nb errors

    Serial << "Checking crc: " << dStar.check_crc(dStarRxHeaderData) << endl;    //crc testing

    Serial << endl << "RF Header:\"";
    for(uint i = 0; i < sizeof(dStarRxHeaderData); i++)
    {
        if(dStarRxHeaderData[i] >= 0x20 && dStarRxHeaderData[i] <= 0x7F)
        {
            Serial << char(dStarRxHeaderData[i]);
        }
        else
        {
            Serial << "0x" << _HEX(dStarRxHeaderData[i]) << " ";
        }
    }
    Serial << "\"" << endl;
}
void prepareDPRS(const String& data)
{
    auto crc = dStar.calcCCITTCRC((uint8_t*)data.c_str(), 0, data.length());
    String dprs = String("$$CRC" + String(crc, HEX) + "," + data);
    Serial << dprs;
    sa.setDPRS((uint8_t*)dprs.c_str(), dprs.length());
}

void startTX()
{
    isPTTPressed = true;
    prepareHeader();
    sa.setMSG(DSTAR_MSG);
    ambeDataBitPos = SlowAmbe::SLOW_AMBE_BITSIZE;//to run into data fetch immediately
    radio.setDio0Action(nullptr);
    checkLoraState(radio.transmitDirect());
    Serial << __FUNCTION__ << endl;
}
void startRX()
{
    isPTTPressed = false;
    radio.setDio0Action(receivedSyncWord);
    checkLoraState(radio.enableOokBitSychronizer());
    checkLoraState(radio.receiveDirect());
    checkLoraState(radio.setSyncWord(syncWord, sizeof(syncWord)));
    Serial << __FUNCTION__ << endl;
}

void setup()
{
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 12, 15);
    serialBT.begin("D-Star Beacon");
    sa.setDataOutput(&serialBT);
    bs.setHeaderBuffer(headerRXTXBuffer, DSTAR::RF_HEADER_SIZE * 2); //TODO refactore to constrctor
    radio.reset();
    Serial.print(F("[SX1278] Initializing ... "));
    pinMode(LORA_IO2, OUTPUT);
    pinMode(LORA_IO1, INPUT);
    Serial << "Start FSK:" << endl;
    checkLoraState(radio.beginFSK(f, 4.8f, 4.8 * 0.25f, 25.0f, 4, 48, false));
    //    MorseClient morse(&radio);
    //    morse.begin(f);
    //    morse.startSignal();
    //    morse.print("001");
    checkLoraState(radio.setEncoding(RADIOLIB_ENCODING_NRZ));
    checkLoraState(radio.setDataShaping(RADIOLIB_SHAPING_0_5));
    radio.setDio1Action(dataClockInterruptHandler);

    startRX();
}

String formatDPRSString(String callsign, double lat, double lon, int alt, String message)
{
    //OK1CHP-1>API705,DSTAR*:!4946.70N/01329.43E-/A=001198TTGO Esp32 rulez!\r
    int deg_lat = int(abs(lat));
    char buff_lat[3];
    sprintf(buff_lat, "%02d", deg_lat);
    double min_lat = (lat - deg_lat) * 60.0f;
    int deg_lon = int(abs(lon));
    double min_lon = (lon - deg_lon) * 60.0f;
    char buff_long[4];
    sprintf(buff_long, "%03d", deg_lon);

    char buff_alt[7];
    sprintf(buff_alt, "%06d", alt);
    return String(callsign
                  + ">API705,DSTAR*:!"
                  + String(buff_lat) + String(min_lat, 2) + ((lat > 0.0f) ? "N" : "S") + "/"
                  + String(buff_long) + String(min_lon, 2) + ((lon > 0.0f) ? "E" : "W")
                  + "-/A=" + String(buff_alt)
                  + message
                  + "\r");
}

uint8_t lastSecond{0};
uint8_t readBTByte{0};
void resetTXData()
{
    preambleBitPos = 0;
    headerBitPos = 0;
    ambeDataBitPos = 0;
    stoppingFramePos = 0;
    isFirstAmbe = true;
}

void loop()
{
    while(gpsSerial.available() > 0)
    {
        auto ch = gpsSerial.read();
        gps.encode(ch);
    }

    if(!sa.isBufferFull())
    {
        //if at least half the buffer capacity is empty, request more data
        if(isPTTPressed &&
           sa.isHalfBufferEmpty() &&
           bluetoothXOFF)
        {
            bluetoothXOFF = false;
            serialBT.write(0x11);//XON
            //            Serial << "TX buffer is ready:" << sa.comBuffer.size() << endl;
        }
        while(serialBT.available() > 0)
        {
            auto ch = serialBT.read();
            ///Serial << "0x" << _HEX(ch) << ", ";
            plainDataBTRXBuff[readBTByte] = ch;
            readBTByte++;
            if(readBTByte == sizeof(plainDataBTRXBuff))
            {
                //                Serial << "Adding: 5 " << endl;
                sa.setDPRS(plainDataBTRXBuff, sizeof(plainDataBTRXBuff));
                readBTByte = 0;
                break;
            }
        }
        if(readBTByte != 0)
        {
            //            Serial << "Adding: " << uint(readBTByte) << endl;
            sa.setDPRS(plainDataBTRXBuff, readBTByte);
            readBTByte = 0;
        }
    }
    else
    {
        if(!bluetoothXOFF)
        {
            //            Serial << "TX buffer is full!!:" << sa.comBuffer.size() << endl;
            serialBT.write(0x13);//XOFF
            bluetoothXOFF = true;
        }
    }
    //    if(gps.time.isUpdated())
    //    {
    //        Serial << gps.time.value() << endl;
    //        auto newSecond = gps.time.second();
    //        if(newSecond % 20 == 0 && lastSecond != newSecond)
    //        {
    //            lastSecond = newSecond;
    //            prepareDPRS(formatDPRSString("OK1CHP-1",
    //                                         gps.location.lat(), gps.location.lng(), gps.altitude.feet(),
    //                                         "T-BEAM GPS"));
    //        }
    //    }
    if(!sa.comBuffer.isEmpty() && !isPTTPressed)
    {
        startTX();
    }

    if(isPTTPressed && stopTx)
    {
        startRX();
        stopTx = false;
        resetTXData();
        sa.reset();
    }

    if(receivedPacket)
    {
        Serial << "fd: " << radio.getFrequencyError(true) << endl;
        if(bs.haveHeader())
        {
            decodeHeader(headerRXTXBuffer);
        }
        receivedPacket = false;
        bs.reset();
        radio.receiveDirect();//reset IRQ flags
        if(sa.haveDStarMsg())
        {
            auto m = sa.getDStarMsg();
            Serial << "Msg:\"";
            for(uint i = 0; i < 20; i++)
            {
                Serial << char(m[i]);
            }
            Serial << "\"" << endl;
        }
        sa.reset();
    }
    if(bs.isEvenDataReady())
    {
        auto data = bs.getEvenData();
        sa.receiveData(data + 9);
    }
    if(bs.isOddDataReady())
    {
        auto data = bs.getOddData();
        sa.receiveData(data + 9);
    }
}
