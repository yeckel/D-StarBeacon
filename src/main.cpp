#include <Arduino.h>
#include <SSD1306Wire.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <BluetoothSerial.h>
#include <pins_arduino.h>
#include <Streaming.h>
#include <axp20x.h>
#include <SPI.h>
#include <DSTAR.h>
#include <DStarDV.h>
#include <Scrambler.h>
#include <BitSlicer.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>

#ifndef LORA_IO1
#define LORA_IO1 LORA_D1 //With board = ttgo-lora32-v21 is the port id different!
#endif
#ifndef LORA_IO2
#define LORA_IO2 LORA_D2
#endif

#if 1
//Old T-Beam with switch
#define GPS_TX 12
#define GPS_RX 15
#else
//new T-Beam with axp20x and soft switch
#define GPS_TX 34
#define GPS_RX 12
#endif
SX1278 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_IO1);
MorseClient morse(&radio);
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_64);
TinyGPSPlus gps;
auto gpsSerial = Serial1;
BluetoothSerial serialBT;
AXP20X_Class axp;
AsyncWebServer server(80);

static constexpr uint16_t RX_SCREEN_TIMEOUT_SECONDS{10};
uint16_t m_RXDataShownSeconds{0};
bool m_isRxOrTxActive{false};

void checkLoraState(int state)
{
    if(state != ERR_NONE)
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while(true);
    }
}

struct BeaconConfig
{
    String ipaddr{"0.0.0.0"};
    String callsign{"MY0CALL"};
    String destination{"DIRECT"};
    String repeater{"DIRECT"};
    String companion{"CQCQCQ"};
    String callsignSuffix{"1"};
    String dStarMsg{"D-Star Beacon"};
    String dprsMsg{"ESP32 D-Star Beacon"};
    float qrt{434.800f};
    float offset{0.0f};
    uint beaconInterval{0};
    uint8_t txPower{5};
    String aprsSymbol{"/["};
};

BeaconConfig config;

uint8_t preambleAndBitSync[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x76, 0x50};
static constexpr uint16_t PREAMBLE_BITSIZE{sizeof(preambleAndBitSync) * 8};
volatile uint preambleBitPos{0};
uint8_t syncWord[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x76, 0x50};
uint8_t stoppingFrame[] = {0xaa, 0xaa, 0xaa, 0xaa, 0x13, 0x5e};
static constexpr uint16_t STOPPING_FRAME_BITSIZE{sizeof(stoppingFrame) * 8};
volatile uint stoppingFramePos{0};
uint8_t plainDataBTRXBuff[5];
uint8_t m_dStarMsg[DStarDV::DSTAR_MSG_SIZE + 1];

uint8_t dStarTxHeaderData[DSTAR::RF_HEADER_SIZE] = {0x0, 0x0, 0x0, //flag 1,2,3
                                                    0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //destination callsign
                                                    0x44, 0x49, 0x52, 0x45, 0x43, 0x54, 0x20, 0x20, //repeater callsign
                                                    0x43, 0x51, 0x43, 0x51, 0x43, 0x51, 0x20, 0x20, //companion callsign
                                                    'O', 'K', '1', 'C', 'H', 'P', 0x20, 0x20, //own callsign
                                                    'E', 'S', '3', '2', //rig ID
                                                    0x00, 0x00, //place for CRC
                                                    0x0 //padding
                                                   };
uint8_t dStarRxHeaderData[DSTAR::RF_HEADER_SIZE];

uint8_t headerRXTXBuffer[DSTAR::RF_HEADER_SIZE * 2];
volatile uint headerBitPos{0};
uint8_t history[DSTAR::RF_HEADER_SIZE * 8];  //buffer for viterbi decoding (large buffer need)

uint8_t payloadData[DStarDV::DSTAR_FRAME_SIZE] = {0x4d, 0xb2, 0x44, 0x12, 0x03, 0x68, 0x14, 0x64, 0x13, 0x66, 0x66, 0x66};//first 9 from real packet

DStarDV m_DStarData;
DSTAR m_dStarHeaderCoder;
BitSlicer m_bitSlicer;

volatile uint ambeDataBitPos{0};
bool isFirstAmbe{true};
volatile bool isPTTPressed{false};
volatile bool stopTx{false};
bool bluetoothXOFF{false};
volatile bool receivedPacket{false};

bool receivedValidRFHeader{false};
bool m_isAp{false};
float m_afc{0.0f};
uint m_timeToBeacon{0};

// Read line from file, independent of line termination (LF or CR LF)
String readLine(Stream& stream)
{
    String s = stream.readStringUntil('\n');
    int len = s.length();
    if(len == 0)
    {
        return s;
    }
    if(s.charAt(len - 1) == '\r')
    {
        s.remove(len - 1);
    }
    return s;
}

void setConfig(const char* cfg)
{
    while(*cfg == ' ' || *cfg == '\t')
    {
        cfg++;
    }
    if(*cfg == '#')
    {
        return;
    }
    char* s = strchr(cfg, '=');
    if(!s)
    {
        return;
    }
    char* val = s + 1;
    *s = 0;
    s--;
    while(s > cfg && (*s == ' ' || *s == '\t'))
    {
        *s = 0;
        s--;
    }
    String value(val);
    Serial.printf("configuration option '%s'=%s \n", cfg, val);
    if(strcmp(cfg, "Callsign") == 0)
    {
        value.toUpperCase();
        config.callsign = value;
        return;
    }
    if(strcmp(cfg, "dStarMsg") == 0)
    {
        config.dStarMsg = value;
        return;
    }
    if(strcmp(cfg, "dprsMsg") == 0)
    {
        config.dprsMsg = value;
        return;
    }
    if(strcmp(cfg, "QRT") == 0)
    {
        auto freq = value.toFloat();
        if(freq == 0.0f)
        {
            Serial << "Invalid frequency:" << value << endl;
        }
        else
        {
            config.qrt = freq;
        }
        return;
    }
    if(strcmp(cfg, "Offset") == 0)
    {
        config.offset = value.toFloat();
        return;
    }
    if(strcmp(cfg, "BeaconInterval") == 0)
    {
        config.beaconInterval = value.toInt();
        return;
    }
    if(strcmp(cfg, "Suffix") == 0)
    {
        config.callsignSuffix = value == "0" ? "" : value;
        return;
    }
    if(strcmp(cfg, "PWR") == 0)
    {
        config.txPower = min(max(int(value.toInt()), 2), 17);//min,max values from RadioLib
        return;
    }
    if(strcmp(cfg, "Companion") == 0)
    {
        value.toUpperCase();
        config.companion = value;
        return;
    }
    if(strcmp(cfg, "Destination") == 0)
    {
        value.toUpperCase();
        config.destination = value;
        return;
    }
    if(strcmp(cfg, "Repeater") == 0)
    {
        value.toUpperCase();
        config.repeater = value;
        return;
    }
    if(strcmp(cfg, "aprsSymbol") == 0)
    {
        config.aprsSymbol = value;
        return;
    }
    Serial.printf("Invalid config option '%s'=%s \n", cfg, val);
}
void readConfig()
{
    File file = SPIFFS.open("/config.txt", "r");
    if(!file)
    {
        Serial << "Error opening the file '/config.txt' for reading" << endl;
        return;
    }
    Serial << "Reading channel config:" << endl;
    while(file.available())
    {
        String line = readLine(file);
        setConfig(line.c_str());
    }
}
void repaintDisplay()
{
    display.clear();
    display.drawString(0, 0, isPTTPressed ? "TX" : "RX");
    char buff[100];
    sprintf(buff, "f:%7.3fMHz", config.qrt);
    display.drawString(20, 0, buff);
    display.drawString(90, 0, gps.location.isValid() ? "GPS" : "gps");
    display.drawString(115, 0, serialBT.connected() ? "BT" : "bt");
    display.drawString(0, 10, "CS:" + config.callsign);
    display.drawString(67, 10, "CP:" + config.companion);
    display.drawLine(0, 21, 128, 21);
    char buffHeader[20];
    if(receivedValidRFHeader && m_RXDataShownSeconds)
    {
        //callsign
        memcpy(buffHeader, dStarRxHeaderData + 27, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(0, 22, buff, "Cs:%s", buffHeader);
        //rig
        memcpy(buffHeader, dStarRxHeaderData + 35, 4);
        buffHeader[4] = 0x00;
        display.drawStringf(65, 22, buff, "Rig:%s", buffHeader);
        //dst
        memcpy(buffHeader, dStarRxHeaderData + 3, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(0, 32, buff, "Ds:%s", buffHeader);
        //companion
        memcpy(buffHeader, dStarRxHeaderData + 19, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(65, 32, buff, "Cp:%s", buffHeader);
        //D-Star msg
        display.drawStringf(0, 42, buff, "Msg:%s", m_dStarMsg);
        //AFC value
        display.drawStringf(0, 52, buff, "f er: %.1f kHz", m_afc / 1000);
        //Display timeout
        display.drawStringf(70, 52, buff, "to: %d s", m_RXDataShownSeconds);
    }
    else
    {
        display.drawString(0, 22, (m_isAp ? "AP:" : "IP:") + config.ipaddr);
        display.drawString(0, 32, "ssid:" + (m_isAp ? WiFi.softAPSSID() : WiFi.SSID()));
        display.drawStringf(0, 42, buff, "lat: %07.4f, lon:%08.4f", gps.location.lat(), gps.location.lng());
        display.drawStringf(0, 52, buff, "beacon in: %d s", config.beaconInterval ? m_timeToBeacon : 0);
    }
    display.display();
}
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
    m_DStarData.getNextData(data);
    memcpy(payloadData + 9, p_data, 3); //using just last 3 bytes in slowAmbeData and 3 first bytes from data
    ambeDataBitPos = 0;
    isFirstAmbe = false;
}

void sendPayloadBit()
{
    if(ambeDataBitPos == DStarDV::SLOW_AMBE_BITSIZE ||
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
//--------------------------------Interrupt handler-------------------------------------
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
    else if(!m_DStarData.comBuffer.isEmpty() ||
            ambeDataBitPos < DStarDV::SLOW_AMBE_BITSIZE)
    {
        sendPayloadBit();
    }
    else if(stoppingFramePos < STOPPING_FRAME_BITSIZE)
    {
        sendStoppingFrameBit();
    }
    else
    {
        radio.clearDio1Action();
        m_isRxOrTxActive = false;
        stopTx = true;
    }
}

void rxBit()
{
    auto receivedBit = digitalRead(LORA_IO2);
    auto commStopped = m_bitSlicer.appendBit(receivedBit);
    if(commStopped)
    {
        radio.clearDio1Action();
        m_isRxOrTxActive = false;
        receivedPacket = true;
    }
}

void receivedSyncWord(void)
{
    radio.setDio1Action(rxBit);
    m_isRxOrTxActive = true;
}
//----------------------------------------------------------------------------------------
void printHeader(uint8_t* rfHeader)
{
    Serial << endl << "RF Header:\"";
    for(uint i = 0; i < DSTAR::RF_HEADER_SIZE; i++)
    {
        if(rfHeader[i] >= 0x20 && rfHeader[i] <= 0x7F)
        {
            Serial << char(rfHeader[i]);
        }
        else
        {
            Serial << "0x" << _HEX(rfHeader[i]) << " ";
        }
    }
    Serial << "\"" << endl;
}
void prepareHeader()
{
    Serial << __FUNCTION__ << endl;
    memset(dStarTxHeaderData + 3, 0x20, 4 * 8);
    memcpy(dStarTxHeaderData + 3, config.destination.c_str(), min(int(config.destination.length()), 8));
    memcpy(dStarTxHeaderData + 3 + 8, config.repeater.c_str(), min(int(config.repeater.length()), 8));
    memcpy(dStarTxHeaderData + 3 + 16, config.companion.c_str(), min(int(config.companion.length()), 8));
    memcpy(dStarTxHeaderData + 3 + 24, config.callsign.c_str(), min(int(config.callsign.length()), 8));
    printHeader(dStarTxHeaderData);
    m_dStarHeaderCoder.add_crc(dStarTxHeaderData);
    m_dStarHeaderCoder.convolution(dStarTxHeaderData, headerRXTXBuffer);   //source, dest
    m_dStarHeaderCoder.interleave(headerRXTXBuffer);
    m_dStarHeaderCoder.pseudo_random(headerRXTXBuffer, DSTAR::RF_HEADER_TRANSFER_BITSIZE);
}

void decodeHeader(uint8_t* bufferConv)
{
    m_dStarHeaderCoder.pseudo_random(bufferConv, 660);
    m_dStarHeaderCoder.deInterleave(bufferConv);
    m_dStarHeaderCoder.viterbi(bufferConv, history, dStarRxHeaderData); //decode data

    Serial << "Nb errors:";
    Serial << int(m_dStarHeaderCoder.acc_error[1][0]) << endl;    //print nb errors

    auto isCrcFine = m_dStarHeaderCoder.check_crc(dStarRxHeaderData);
    Serial << "Checking crc: " << isCrcFine << endl;    //crc testing

    if(isCrcFine)
    {
        receivedValidRFHeader = true;
        printHeader(dStarRxHeaderData);
    }
}
void prepareDPRS(const String& data)
{
    auto crc = m_dStarHeaderCoder.calcCCITTCRC((uint8_t*)data.c_str(), 0, data.length());
    String dprs = String("$$CRC" + String(crc, HEX) + "," + data);
    Serial << "DPRS string:" << dprs;
    m_DStarData.setDPRS((uint8_t*)dprs.c_str(), dprs.length());
}

void startTX()
{
    isPTTPressed = true;
    repaintDisplay();
    prepareHeader();
    m_DStarData.reset();
    m_bitSlicer.reset();
    m_DStarData.setMSG(config.dStarMsg);
    ambeDataBitPos = DStarDV::SLOW_AMBE_BITSIZE;//to run into data fetch immediately
    radio.clearDio0Action();
    radio.setDio1Action(txBit);
    m_isRxOrTxActive = true;
    checkLoraState(radio.transmitDirect());
    Serial << __FUNCTION__ << endl;
}
void startRX()
{
    isPTTPressed = false;
    m_DStarData.reset();
    m_bitSlicer.reset();
    radio.setDio0Action(receivedSyncWord);
    checkLoraState(radio.receiveDirect());
    Serial << __FUNCTION__ << endl;
}

// Replaces placeholder with LED state value
String processor(const String& var)
{
    char buff[20];
    Serial.println(var);
    if(var == "CALLSIGN")
    {
        return config.callsign;
    }
    if(var == "QRT")
    {
        sprintf(buff, "%7.3f", config.qrt);
        return String(buff);
    }
    if(var == "Offset")
    {
        sprintf(buff, "%.3f", config.offset);
        return String(buff);
    }
    if(var == "AFC")
    {
        sprintf(buff, "%.3f", m_afc / 1000);
        return String(buff);
    }
    if(var == "dStarMsg")
    {
        return config.dStarMsg;
    }
    if(var == "dprsMsg")
    {
        return config.dprsMsg;
    }
    if(var == "BeaconInterval")
    {
        return String(config.beaconInterval);
    }
    if(var == "SUFFIX")
    {
        if(config.callsignSuffix.isEmpty())
        {
            return "0";
        }
        else
        {
            return config.callsignSuffix;
        }
    }
    if(var == "PWR")
    {
        return String(config.txPower);
    }
    if(var == "DESTINATION")
    {
        return String(config.destination);
    }
    if(var == "REPEATER")
    {
        return String(config.repeater);
    }
    if(var == "COMPANION")
    {
        return String(config.companion);
    }
    if(var == "ASYMBOL")
    {
        return config.aprsSymbol;
    }
    return String();
}

void writeParamToFile(AsyncWebParameter* param, File& file)
{
    char buff[100];
    sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
    int wlen = file.printf("%s\n", buff);
    Serial.printf("Written: %s %d\n", buff, wlen);
}

const char* handleConfigPost(AsyncWebServerRequest* request)
{
    File file = SPIFFS.open("/config.txt", "w");
    if(!file)
    {
        Serial.println("Error while opening '/config.txt' for writing");
        return "Error while opening '/config.txt' for writing";
    }

    //    Serial << "Params: " << endl;
    //    int params = request->params();
    //    for(int i = 0; i < params; i++)
    //    {
    //        String param = request->getParam(i)->name();
    //        Serial.println(param.c_str());
    //    }
    {
        auto param = request->getParam("Callsign", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            val.toUpperCase();
            config.callsign = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("Suffix", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.callsignSuffix = val == "0" ? "" : val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("QRT", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            auto freq = String(val).toFloat();
            if(freq != 0.0f)
            {
                config.qrt = freq;
                writeParamToFile(param, file);
            }
        }
    }
    {
        auto param = request->getParam("Offset", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            auto freqOffset = String(val).toFloat();
            config.offset = freqOffset;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("PWR", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.txPower = min(max(int(String(val).toInt()), 2), 17);
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("dStarMsg", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.dStarMsg = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("dprsMsg", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.dprsMsg = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("BeaconInterval", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.beaconInterval = String(val).toInt();
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("Destination", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            val.toUpperCase();
            config.destination = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("Repeater", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            val.toUpperCase();
            config.repeater = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("aprsSymbol", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            config.aprsSymbol = val;
            writeParamToFile(param, file);
        }
    }
    {
        auto param = request->getParam("Companion", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            val.toUpperCase();
            config.companion = val;
            writeParamToFile(param, file);
        }
    }
    Serial.printf("Flushing file\n");
    file.flush();
    Serial.printf("Closing file\n");
    file.close();
    radio.setFrequency(config.qrt + config.offset / 1000);
    startRX();
    return "";
}
void setupAsyncServer()
{
    Serial << __FUNCTION__ << endl;
    server.reset();
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", String("text/html"), false, processor);
    });
    server.on("/config.txt", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/config.txt", String("text/html"), false);
    });
    server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", String("text/html"), false, processor);
    });
    server.on("/index.html", HTTP_POST, [](AsyncWebServerRequest * request)
    {
        handleConfigPost(request);
        request->send(SPIFFS, "/index.html", String("text/html"), false, processor);
    });
    server.onNotFound([](AsyncWebServerRequest * request)
    {
        Serial << "Processing onNotFound!" << endl;
        if(request->method() == HTTP_OPTIONS)
        {
            request->send(200);
        }
        else
        {
            request->send(404);
        }
    });

    // Start server
    server.begin();
}

/////////////////// Functions for reading/writing Wifi networks from networks.txt

#define MAX_WIFI 10
int nNetworks;
struct
{
    String id;
    String pw;
} networks[MAX_WIFI];

// FIXME: For now, we don't uspport wifi networks that contain newline or null characters
// ... would require a more sophisicated file format (currently one line SSID; one line Password
void setupWifiList()
{
    File file = SPIFFS.open("/networks.txt", "r");
    if(!file)
    {
        Serial.println("There was an error opening the file '/networks.txt' for reading");
        networks[0].id = "D-StarBeacon";
        networks[0].pw = "D-StarBeacon";
        return;
    }
    int i = 0;

    while(file.available())
    {
        String line = readLine(file);  //file.readStringUntil('\n');
        if(!file.available())
        {
            break;
        }
        networks[i].id = line;
        networks[i].pw = readLine(file); // file.readStringUntil('\n');
        i++;
    }
    nNetworks = i;
    Serial.print(i);
    Serial.println(" networks in networks.txt\n");
    for(int j = 0; j < i; j++)
    {
        Serial.print(networks[j].id);
        Serial.print(": ");
        Serial.println(networks[j].pw);
    }
}

int fetchWifiIndex(const char* id)
{
    for(int i = 0; i < nNetworks; i++)
    {
        if(strcmp(id, networks[i].id.c_str()) == 0)
        {
            Serial.printf("Match for %s at %d\n", id, i);
            return i;
        }
        //Serial.printf("No match: '%s' vs '%s'\n", id, networks[i].id.c_str());
        const char* cfgid = networks[i].id.c_str();
        auto len = strlen(cfgid);
        if(strlen(id) > len)
        {
            len = strlen(id);
        }
    }
    return -1;
}

const char* fetchWifiSSID(int i)
{
    return networks[i].id.c_str();
}
const char* fetchWifiPw(int i)
{
    return networks[i].pw.c_str();
}

const char* fetchWifiPw(const char* id)
{
    for(int i = 0; i < nNetworks; i++)
    {
        //Serial.print("Comparing '");
        //Serial.print(id);
        //Serial.print("' and '");
        //Serial.print(networks[i].id.c_str());
        //Serial.println("'");
        if(strcmp(id, networks[i].id.c_str()) == 0)
        {
            return networks[i].pw.c_str();
        }
    }
    return NULL;
}

void startAP()
{
    Serial.println("Activating access point mode");
    WiFi.softAP(networks[0].id.c_str(), networks[0].pw.c_str());

    Serial.println("Wait 100 ms for AP_START...");
    delay(100);
    Serial.println(WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0)) ? "Ready" : "Failed!");

    IPAddress myIP = WiFi.softAPIP();
    config.ipaddr = myIP.toString();
    m_isAp = true;
}
String translateEncryptionType(wifi_auth_mode_t encryptionType)
{
    switch(encryptionType)
    {
    case(WIFI_AUTH_OPEN):
        return "Open";
    case(WIFI_AUTH_WEP):
        return "WEP";
    case(WIFI_AUTH_WPA_PSK):
        return "WPA_PSK";
    case(WIFI_AUTH_WPA2_PSK):
        return "WPA2_PSK";
    case(WIFI_AUTH_WPA_WPA2_PSK):
        return "WPA_WPA2_PSK";
    case(WIFI_AUTH_WPA2_ENTERPRISE):
        return "WPA2_ENTERPRISE";
    default:
        return "";
    }
}

#define MAXWIFIDELAY 20

void loopWifiScan()
{
    int cnt = 0;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    int index = -1;
    int n = WiFi.scanNetworks();
    for(int i = 0; i < n; i++)
    {
        String ssid = WiFi.SSID(i);
        String mac = WiFi.BSSIDstr(i);
        String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
        Serial.printf("Network %s: RSSI %d, MAC %s, enc: %s\n", ssid.c_str(), WiFi.RSSI(i), mac.c_str(), encryptionTypeDescription.c_str());
        int curidx = fetchWifiIndex(ssid.c_str());
        if(curidx >= 0 && index == -1)
        {
            index = curidx;
            Serial.printf("Match found at scan entry %d, config network %d\n", i, index);
        }
    }
    if(index >= 0)    // some network was found
    {
        Serial.print("Connecting to: ");
        Serial.print(fetchWifiSSID(index));
        Serial.print(" with password ");
        Serial.println(fetchWifiPw(index));

        WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
        while(WiFi.status() != WL_CONNECTED && cnt < MAXWIFIDELAY)
        {
            delay(500);
            Serial.print(".");
            if(cnt == 5)
            {
                // my FritzBox needs this for reconnecting
                WiFi.disconnect(true);
                delay(500);
                WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
                Serial.print("Reconnecting to: ");
                Serial.print(fetchWifiSSID(index));
                Serial.print(" with password ");
                Serial.println(fetchWifiPw(index));
                delay(500);
            }
            cnt++;
        }
    }
    if(index < 0 || cnt >= MAXWIFIDELAY)    // no network found, or connect not successful
    {
        WiFi.disconnect(true);
        delay(1000);
        startAP();
        IPAddress myIP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(myIP);
        delay(3000);
    }
    else
    {
        Serial << endl << "WiFi connected" << endl;
        String localIPstr = WiFi.localIP().toString();
        Serial << "IP address: " << localIPstr << endl;
        config.ipaddr = localIPstr;
        delay(3000);
    }
    setupAsyncServer();
}
void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);
    Serial.begin(1000000);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    serialBT.begin("D-Star Beacon");
    m_DStarData.setDataOutput(&serialBT);
    Wire.begin(SDA, SCL);
    if(!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    {
        Serial.println("AXP192 Begin PASS");
    }
    else
    {
        Serial.println("AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);
    axp.adc1Enable(AXP202_VBUS_VOL_ADC1, 1);
    axp.adc1Enable(AXP202_VBUS_CUR_ADC1, 1);
    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
    display.init();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    repaintDisplay();//to show something's happening ASAP after power on
    m_bitSlicer.setHeaderBuffer(headerRXTXBuffer, DSTAR::RF_HEADER_SIZE * 2); //TODO refactore to constrctor
    memset(m_dStarMsg, 0x20, DStarDV::DSTAR_MSG_SIZE);
    m_dStarMsg[DStarDV::DSTAR_MSG_SIZE] = 0x00;
    Serial.println("Initializing SPIFFS");
    if(!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    Serial.println("Reading initial configuration");
    readConfig();

    setupWifiList();
    loopWifiScan();

    radio.reset();
    Serial.print(F("Initializing ... "));
    pinMode(LORA_IO2, OUTPUT);
    pinMode(LORA_IO1, INPUT);
    checkLoraState(radio.beginFSK(config.qrt, 4.8f, 4.8 * 0.25f, 25.0f, config.txPower, 48, false));
    checkLoraState(radio.setEncoding(RADIOLIB_ENCODING_NRZ));
    checkLoraState(radio.setDataShaping(RADIOLIB_SHAPING_0_5));
    checkLoraState(radio.setSyncWord(syncWord, sizeof(syncWord)));

    startRX();
}

String formatDPRSString(String callsign, String symbol, double lat, double lon, int alt, String message)
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
    symbol = symbol.length() == 2 ? symbol : "/[";
    return String(callsign
                  + ">API705,DSTAR*:!"
                  + String(buff_lat) + String(min_lat, 2) + ((lat > 0.0f) ? "N" : "S")
                  + symbol[0]
                  + String(buff_long) + String(min_lon, 2) + ((lon > 0.0f) ? "E" : "W")
                  + symbol[1]
                  + "/A=" + String(buff_alt)
                  + message
                  + "\r");
}

long lastBEaconTime{0};
uint8_t readBTByte{0};
void resetTXData()
{
    preambleBitPos = 0;
    headerBitPos = 0;
    ambeDataBitPos = 0;
    stoppingFramePos = 0;
    isFirstAmbe = true;
}

long getGPSTime()
{
    time_t t_of_day{1618227800};
    struct tm t;
    if(gps.date.isValid())
    {
        t.tm_year = gps.date.year() - 1900;
        t.tm_mon = gps.date.month() - 1;         // Month, 0 - jan
        t.tm_mday = gps.date.day();          // Day of the month
        t.tm_hour = gps.time.hour();
        t.tm_min =  gps.time.minute();
        t.tm_sec = gps.time.second();
        t_of_day = mktime(&t);
    }
    return t_of_day;
}

auto m_lastTime = millis();
void loop()
{
    //run each 1s
    if(millis() - m_lastTime > 1000)
    {
        m_lastTime = millis();
        m_RXDataShownSeconds = m_RXDataShownSeconds > 0 ? m_RXDataShownSeconds - 1 : m_RXDataShownSeconds;
        if(!m_isRxOrTxActive)
        {
            repaintDisplay();
        }
    }
    while(gpsSerial.available() > 0)
    {
        auto ch = gpsSerial.read();
        gps.encode(ch);
        //        Serial.write(ch);
    }

    if(!m_DStarData.isBufferFull())
    {
        //if at least half the buffer capacity is empty, request more data
        if(isPTTPressed &&
           m_DStarData.isHalfBufferEmpty() &&
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
                m_DStarData.setDPRS(plainDataBTRXBuff, sizeof(plainDataBTRXBuff));
                readBTByte = 0;
                break;
            }
        }
        if(readBTByte != 0)
        {
            //            Serial << "Adding: " << uint(readBTByte) << endl;
            m_DStarData.setDPRS(plainDataBTRXBuff, readBTByte);
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

    if(!isPTTPressed &&
       gps.time.isUpdated() &&
       config.beaconInterval &&
       gps.location.isValid())
    {
        auto newSecond = getGPSTime();
        //        Serial << newSecond << endl;
        if(newSecond % config.beaconInterval == 0
           && lastBEaconTime != newSecond)
        {
            lastBEaconTime = newSecond;
            prepareDPRS(formatDPRSString(config.callsign +
                                         (config.callsignSuffix.isEmpty() ? "" : ("-" + config.callsignSuffix)),
                                         config.aprsSymbol,
                                         gps.location.lat(),
                                         gps.location.lng(),
                                         gps.altitude.feet(),
                                         config.dprsMsg));
        }
        else
        {
            m_timeToBeacon = config.beaconInterval - (newSecond - lastBEaconTime);
        }
    }
    if(!m_DStarData.comBuffer.isEmpty() && !isPTTPressed)
    {
        startTX();
    }

    if(isPTTPressed && stopTx)
    {
        startRX();
        stopTx = false;
        resetTXData();
    }

    if(receivedPacket)
    {
        Serial << endl << "RX Stopped" << endl;
        receivedPacket = false;
        m_RXDataShownSeconds = RX_SCREEN_TIMEOUT_SECONDS;
        m_afc = radio.getAFCError();
        Serial << "You are of:" << m_afc << "Hz from carrier.";
        radio.receiveDirect();//reset IRQ flags
        if(m_bitSlicer.haveHeader())
        {
            decodeHeader(headerRXTXBuffer);
        }
        if(m_DStarData.haveDStarMsg())
        {
            memcpy(m_dStarMsg, m_DStarData.getDStarMsg(), DStarDV::DSTAR_MSG_SIZE);
        }
        startRX();
    }
    if(m_bitSlicer.isEvenDataReady())
    {
        m_DStarData.receiveData(m_bitSlicer.getEvenData());
    }
    if(m_bitSlicer.isOddDataReady())
    {
        m_DStarData.receiveData(m_bitSlicer.getOddData());
    }
    if(m_bitSlicer.isSyncDataReady())
    {
        m_DStarData.receiveSyncData(m_bitSlicer.getSyncData());
    }
}
