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
BluetoothSerial ESP_BT;
AXP20X_Class axp;
TaskHandle_t BeaconSenderHandler;

void checkLoraState(int state)
{
    if(state == ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while(true);
    }
}

uint8_t preambleAndBitSync[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x76, 0x50};
volatile uint preambleBitPos{0};
uint8_t stoppingFrame[] = {0xaa, 0xaa, 0xaa, 0xaa, 0x13, 0x5e};
volatile uint stoppingFramePos{0};

uint8_t DSTAR_MSG[SlowAmbe::DSTAR_MSG_SIZE] = {'D', '-', 'S', 't', 'a', 'r', ' ', 'b', 'e', 'a', 'c', 'o', 'n', ' ', 'e', 's', 'p', '3', '2', '!'};

uint8_t dStarHeader[DSTAR::RF_HEADER_SIZE] = {0x0, 0x0, 0x0, //flag 1,2,3
                                              0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //destination callsign
                                              0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //repeater callsign
                                              0x43, 0x51, 0x43, 0x51, 0x43, 0x51, 0x20, 0x20, //companion callsign
                                              'O', 'K', '1', 'C', 'H', 'P', 0x20, 0x20, //own callsign
                                              '7', '0', '5', ' ', //rig ID
                                              0x00, 0x00, //place for CRC
                                              0x0 //padding
                                             };
uint8_t headerTXBuffer[DSTAR::RF_HEADER_SIZE * 2];
volatile uint headerBitPos{0};

uint8_t slowAmbeData[SlowAmbe::SLOW_AMBE_SIZE] = {0x4d, 0xb2, 0x44, 0x12, 0x03, 0x68, 0x14, 0x64, 0x13, 0x66, 0x66, 0x66};//first 9 from real packet

SlowAmbe sa;
DSTAR dStar;

volatile uint ambeDataBitPos{0};
bool isFirstAmbe{true};

float f = 434.800f + 0.0024f - 0.00091;

void startTX()
{
    ambeDataBitPos = SlowAmbe::SLOW_AMBE_BITSIZE;//to run into data fetch immediately
    checkLoraState(radio.transmitDirect());
    Serial << "Start transmit" << endl;
}

void sendBit(uint8_t* sendBuff, uint buffBitPos)
{
    auto bytePos = buffBitPos / 8;
    auto bitPos = buffBitPos % 8;
    bool bit = sendBuff[bytePos] & (0b10000000 >> bitPos);
    //    Serial << bit;
    digitalWrite(LORA_IO2, bit);
}

void dataClk()
{
    if(preambleBitPos < sizeof(preambleAndBitSync) * 8)
    {
        //        Serial << "P";
        sendBit(preambleAndBitSync, preambleBitPos);
        preambleBitPos++;
    }
    else if(headerBitPos < 660)
    {
        //        Serial << "H";
        sendBit(headerTXBuffer, headerBitPos);
        headerBitPos++;
    }
    else if(!sa.comBuffer.isEmpty() ||
            ambeDataBitPos < SlowAmbe::SLOW_AMBE_BITSIZE)
    {
        if(ambeDataBitPos == SlowAmbe::SLOW_AMBE_BITSIZE ||
           isFirstAmbe)
        {
            //            Serial << endl;
            uint32_t data;
            uint8_t* p_data{(uint8_t*)& data};
            sa.getNextData(data);
            memcpy(slowAmbeData + 9, p_data, 3); //using just last 3 bytes in slowAmbeData and 3 first bytes from data
            ambeDataBitPos = 0;
            isFirstAmbe = false;
        }
        //        Serial << "A";
        sendBit(slowAmbeData, ambeDataBitPos);
        ambeDataBitPos++;
    }
    else if(stoppingFramePos < sizeof(stoppingFrame) * 8)
    {
        //        Serial << "S";
        sendBit(stoppingFrame, stoppingFramePos);
        stoppingFramePos++;
    }
    else
    {
        radio.standby();//TODO receive
        preambleBitPos = 0;
        headerBitPos = 0;
        ambeDataBitPos = 0;
        stoppingFramePos = 0;
        isFirstAmbe = true;
    }
}
void print_data(byte* data, int dataSize)
{
    for(int n = 0; n < dataSize; n++)
    {
        Serial << "0x" << _HEX(data[n]) << ",";
    }
}

void prepareHeader()
{
    dStar.add_crc(dStarHeader);
    dStar.convolution(dStarHeader, headerTXBuffer);   //source, dest
    dStar.interleave(headerTXBuffer);
    dStar.pseudo_random(headerTXBuffer, 660);
}

void prepareDPRS(const String& data)
{
    auto crc = dStar.calcCCITTCRC((uint8_t*)data.c_str(), 0, data.length());
    String dprs = String("$$CRC" + String(crc, HEX) + "," + data);
    Serial << dprs;
    sa.setDPRS((uint8_t*)dprs.c_str(), dprs.length());
}
void setup()
{
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 12, 15);
    radio.reset();
    Serial.print(F("[SX1278] Initializing ... "));
    pinMode(LORA_IO2, OUTPUT);
    pinMode(LORA_IO1, INPUT);

    //    checkLoraState(radio.begin(f, 10.4));
    Serial << "Start FSK:" << endl;
    checkLoraState(radio.beginFSK(f, 4.8f, 4.8 * 0.25f, 25.0f, 4, 48, false));
    //    MorseClient morse(&radio);
    //    morse.begin(f);
    //    morse.startSignal();
    //    morse.print("001");
    checkLoraState(radio.setEncoding(RADIOLIB_ENCODING_NRZ));
    checkLoraState(radio.setDataShaping(RADIOLIB_SHAPING_0_5));
    attachInterrupt(LORA_IO1, dataClk, RISING);

    Serial << "Setup done!" << endl;
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
void loop()
{
    //    beaconSender();
    while(gpsSerial.available() > 0)
    {
        auto ch = gpsSerial.read();
        gps.encode(ch);
    }
    if(gps.time.isUpdated())
    {
        Serial << gps.time.value() << endl;
        auto newSecond = gps.time.second();
        if(newSecond % 20 == 0 && lastSecond != newSecond)
        {
            lastSecond = newSecond;
            prepareHeader();
            sa.setMSG(DSTAR_MSG);
            prepareDPRS(formatDPRSString("OK1CHP-1",
                                         gps.location.lat(), gps.location.lng(), gps.altitude.feet(),
                                         "T-BEAM GPS"));
            startTX();
        }

    }
}
