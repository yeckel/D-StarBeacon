//For T-Beam with AXP202
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
#include <SlowAmbe.h>
#include <Scrambler.h>
#include <BitSlicer.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <sys/time.h>



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
AsyncWebServer server(80);

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
    // hardware configuration
    int wifi;				// connect to known WLAN 0=skip
    String ipaddr{"0.0.0.0"};
    String callsign{"MY0CALL"};
    String callsignSuffix{"1"};
    String dStarMsg{"D-Star Beacon"};
    String dprsMsg{"ESP32 D-Star Beacon"};
    float qrt{434.800f + 0.00244f};
    uint beaconInterval{0};
    uint8_t txPower{5};
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

uint8_t payloadData[SlowAmbe::SLOW_AMBE_SIZE] = {0x4d, 0xb2, 0x44, 0x12, 0x03, 0x68, 0x14, 0x64, 0x13, 0x66, 0x66, 0x66};//first 9 from real packet

SlowAmbe sa;
DSTAR dStar;
BitSlicer bs;

volatile uint ambeDataBitPos{0};
bool isFirstAmbe{true};
volatile bool isPTTPressed{false};
volatile bool stopTx{false};
bool bluetoothXOFF{false};
volatile bool receivedPacket{false};

bool isGPSValid{false};
bool isBTConnected{false};
bool receivedValidRFHeader{false};

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
    Serial.printf("configuration option '%s'=%s \n", cfg, val);
    if(strcmp(cfg, "CALLSIGN") == 0)
    {
        config.callsign = String(val);
        return;
    }
    if(strcmp(cfg, "dStarMsg") == 0)
    {
        config.dStarMsg = String(val);
        return;
    }
    if(strcmp(cfg, "dprsMsg") == 0)
    {
        config.dprsMsg = String(val);
        return;
    }
    if(strcmp(cfg, "QRT") == 0)
    {
        auto freq = String(val).toFloat();
        if(freq == 0.0f)
        {
            Serial << "Invalid frequency:" << val << endl;
        }
        else
        {
            config.qrt = freq;
        }
        return;
    }
    if(strcmp(cfg, "BeaconInterval") == 0)
    {
        config.beaconInterval = String(val).toInt();
        return;
    }
    if(strcmp(cfg, "Suffix") == 0)
    {
        config.callsignSuffix = val;
        return;
    }
    if(strcmp(cfg, "PWR") == 0)
    {
        config.txPower = min(max(int(String(val).toInt()), 2), 17);//min,max values from RadioLib
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
    display.drawString(90, 0, isGPSValid ? "GPS" : "gps");
    display.drawString(115, 0, isBTConnected ? "BT" : "bt");
    display.drawString(0, 10, "IP:" + config.ipaddr);
    display.drawString(60, 10, "CS:" + config.callsign);
    display.drawLine(0, 32, 128, 32);
    char buffHeader[10];
    if(receivedValidRFHeader)
    {
        //callsign
        memcpy(buffHeader, dStarRxHeaderData + 27, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(0, 33, buff, "Cs:%s", buffHeader);
        //rig
        memcpy(buffHeader, dStarRxHeaderData + 35, 4);
        buffHeader[4] = 0x00;
        display.drawStringf(65, 33, buff, "Rig:%s", buffHeader);
        //dst
        memcpy(buffHeader, dStarRxHeaderData + 3, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(0, 43, buff, "Ds:%s", buffHeader);
        //companion
        memcpy(buffHeader, dStarRxHeaderData + 19, 8);
        buffHeader[8] = 0x00;
        display.drawStringf(65, 43, buff, "Cc:%s", buffHeader);
    }
    if(sa.haveDStarMsg())
    {
        auto m = (const char*)sa.getDStarMsg();
        display.drawStringf(0, 52, buff, "Msg:%s", m);
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
        radio.clearDio1Action();
        stopTx = true;
    }
}
//-------------------------------RX bit routines----------------------------------------
void rxBit()
{
    auto receivedBit = digitalRead(LORA_IO2);
    auto commStopped = bs.appendBit(receivedBit);
    if(commStopped)
    {
        radio.clearDio1Action();
        receivedPacket = true;
    }
}
//--------------------------------Interrupt handler-------------------------------------

void receivedSyncWord(void)
{
    radio.setDio1Action(rxBit);
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

    Serial << "Nb errors:";
    Serial << int(dStar.acc_error[1][0]) << endl;    //print nb errors

    auto isCrcFine = dStar.check_crc(dStarRxHeaderData);
    Serial << "Checking crc: " << isCrcFine << endl;    //crc testing

    if(isCrcFine)
    {
        receivedValidRFHeader = true;
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
    repaintDisplay();
    prepareHeader();
    sa.reset();
    bs.reset();
    sa.setMSG(config.dStarMsg);
    ambeDataBitPos = SlowAmbe::SLOW_AMBE_BITSIZE;//to run into data fetch immediately
    radio.clearDio0Action();
    radio.setDio1Action(txBit);
    checkLoraState(radio.transmitDirect());
    radio.setDio1Action(txBit);
    Serial << __FUNCTION__ << endl;
}
void startRX()
{
    isPTTPressed = false;
    repaintDisplay();
    sa.reset();
    bs.reset();
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
        return config.callsignSuffix;
    }
    if(var == "PWR")
    {
        return String(config.txPower);
    }
    return String();
}

const char* handleQRGPost(AsyncWebServerRequest* request)
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
    char buff[100];
    {
        auto param = request->getParam("CALLSIGN", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "Callsign: " << val << endl;
            config.callsign = val;
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    {
        auto param = request->getParam("Suffix", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "Suffix: " << val << endl;
            config.callsignSuffix = val;
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    {
        auto param = request->getParam("QRT", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "QRT: " << val << endl;
            auto freq = String(val).toFloat();
            if(freq != 0.0f)
            {
                config.qrt = freq;
                sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
                int wlen = file.printf("%s\n", buff);
                Serial.printf("Written: %s %d\n", buff, wlen);
            }
        }
    }
    {
        auto param = request->getParam("PWR", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "PWR: " << val << endl;
            config.txPower = min(max(int(String(val).toInt()), 2), 17);
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    {
        auto param = request->getParam("dStarMsg", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "dStarMsg: " << val << endl;
            config.dStarMsg = val;
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    {
        auto param = request->getParam("dprsMsg", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "dprsMsg: " << val << endl;
            config.dprsMsg = val;
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    {
        auto param = request->getParam("BeaconInterval", true);
        if(param != nullptr)
        {
            auto val = param->value();
            val.trim();
            Serial << "BeaconInterval: " << val << endl;
            config.beaconInterval = String(val).toInt();
            sprintf(buff, "%s=%s", param->name().c_str(), param->value().c_str());
            int wlen = file.printf("%s\n", buff);
            Serial.printf("Written: %s %d\n", buff, wlen);
        }
    }
    Serial.printf("Flushing file\n");
    file.flush();
    Serial.printf("Closing file\n");
    file.close();
    repaintDisplay();
    return "";
}
void SetupAsyncServer()
{
    Serial << __FUNCTION__ << endl;
    server.reset();
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", String(), false, processor);
    });
    server.on("/index.html", HTTP_POST, [](AsyncWebServerRequest * request)
    {
        handleQRGPost(request);
        request->send(SPIFFS, "/index.html", String(), false, processor);
    });
    server.onNotFound([](AsyncWebServerRequest * request)
    {
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
        networks[0].id = "RDZsonde";
        networks[0].pw = "RDZsonde";
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

void enableNetwork(bool enable)
{
    if(enable)
    {
        MDNS.begin("D-Star Beacon");
        SetupAsyncServer();
        MDNS.addService("http", "tcp", 80);
        //connected = true;
    }
}

enum t_wifi_state { WIFI_DISABLED, WIFI_SCAN, WIFI_CONNECT, WIFI_CONNECTED, WIFI_APMODE };

static t_wifi_state wifi_state = WIFI_DISABLED;

// Events used only for debug output right now
void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch(event)
    {
    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case SYSTEM_EVENT_STA_START:
        Serial.println("WiFi client started");
        break;
    case SYSTEM_EVENT_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("Connected to access point");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        if(wifi_state == WIFI_CONNECT)
        {
            // If we get a disconnect event while waiting for connection (as I do sometimes with my FritzBox),
            // just start from scratch with WiFi scan
            wifi_state = WIFI_DISABLED;
            WiFi.disconnect(true);
        }
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case SYSTEM_EVENT_AP_START:
        Serial.println("WiFi access point started");
        break;
    case SYSTEM_EVENT_AP_STOP:
        Serial.println("WiFi access point  stopped");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("Client connected");
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("Client disconnected");
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        Serial.println("Assigned IP address to client");
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("Received probe request");
        break;
    case SYSTEM_EVENT_GOT_IP6:
        Serial.println("IPv6 is preferred");
        break;
    case SYSTEM_EVENT_ETH_START:
        Serial.println("Ethernet started");
        break;
    case SYSTEM_EVENT_ETH_STOP:
        Serial.println("Ethernet stopped");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        Serial.println("Ethernet connected");
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        Serial.println("Ethernet disconnected");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        Serial.println("Obtained IP address");
        break;
    default:
        break;
    }
}

void wifiConnect(int16_t res)
{
    Serial.printf("WiFi scan result: found %d networks\n", res);

    // pick best network
    int bestEntry = -1;
    int bestRSSI = INT_MIN;
    uint8_t bestBSSID[6];
    int32_t bestChannel = 0;

    for(int8_t i = 0; i < res; i++)
    {
        String ssid_scan;
        int32_t rssi_scan;
        uint8_t sec_scan;
        uint8_t* BSSID_scan;
        int32_t chan_scan;
        WiFi.getNetworkInfo(i, ssid_scan, sec_scan, rssi_scan, BSSID_scan, chan_scan);
        int networkEntry = fetchWifiIndex(ssid_scan.c_str());
        if(networkEntry < 0)
        {
            continue;
        }
        if(rssi_scan <= bestRSSI)
        {
            continue;
        }
        bestEntry = networkEntry;
        bestRSSI = rssi_scan;
        bestChannel = chan_scan;
        memcpy((void*) &bestBSSID, (void*) BSSID_scan, sizeof(bestBSSID));
    }
    WiFi.scanDelete();
    if(bestEntry >= 0)
    {
        Serial.printf("WiFi Connecting BSSID: %02X:%02X:%02X:%02X:%02X:%02X SSID: %s PW %s Channel: %d (RSSI %d)\n", bestBSSID[0], bestBSSID[1], bestBSSID[2], bestBSSID[3], bestBSSID[4], bestBSSID[5],
                      fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestRSSI);
        WiFi.begin(fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestBSSID);
        wifi_state = WIFI_CONNECT;
    }
    else
    {
        // rescan
        // wifiStart();
        WiFi.disconnect(true);
        wifi_state = WIFI_DISABLED;
    }
}

static int wifi_cto;

void loopWifiBackground()
{
    //    Serial.printf("WifiBackground: state %d\n", wifi_state);
    // handle Wifi station mode in background

    if(wifi_state == WIFI_DISABLED)     // stopped => start can
    {
        wifi_state = WIFI_SCAN;
        Serial.println("WiFi start scan");
        WiFi.scanNetworks(true); // scan in async mode
    }
    else if(wifi_state == WIFI_SCAN)
    {
        int16_t res = WiFi.scanComplete();
        if(res == 0 || res == WIFI_SCAN_FAILED)
        {
            // retry
            Serial.println("WiFi restart scan");
            WiFi.disconnect(true);
            wifi_state = WIFI_DISABLED;
            return;
        }
        if(res == WIFI_SCAN_RUNNING)
        {
            return;
        }
        // Scan finished, try to connect
        wifiConnect(res);
        wifi_cto = 0;
    }
    else if(wifi_state == WIFI_CONNECT)
    {
        wifi_cto++;
        if(WiFi.isConnected())
        {
            wifi_state = WIFI_CONNECTED;
            // update IP in display
            String localIPstr = WiFi.localIP().toString();
            Serial << "MyIP:" << localIPstr << endl;
            config.ipaddr = localIPstr;
            repaintDisplay();
            enableNetwork(true);
        }
        if(wifi_cto > 20)    // failed, restart scanning
        {
            wifi_state = WIFI_DISABLED;
            WiFi.disconnect(true);
        }
    }
    else if(wifi_state == WIFI_CONNECTED)
    {
        if(!WiFi.isConnected())
        {
            config.ipaddr = "";
            repaintDisplay();
            wifi_state = WIFI_DISABLED;  // restart scan
            enableNetwork(false);
            WiFi.disconnect(true);
        }
        return;
    }
    Serial << "Delay(500)" << endl;
    delay(500);
}

void startAP()
{
    Serial.println("Activating access point mode");
    wifi_state = WIFI_APMODE;
    WiFi.softAP(networks[0].id.c_str(), networks[0].pw.c_str());

    Serial.println("Wait 100 ms for AP_START...");
    delay(100);
    Serial.println(WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0)) ? "Ready" : "Failed!");

    IPAddress myIP = WiFi.softAPIP();
    String myIPstr = myIP.toString();
    config.ipaddr = myIP.toString();
    repaintDisplay();
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

// Wifi modes
// 0: disabled. directly start initial mode (spectrum or scanner)
// 1: station mode in background. directly start initial mode (spectrum or scanner)
// 2: access point mode in background. directly start initial mode (spectrum or scanner)
// 3: traditional sync. WifiScan. Tries to connect to a network, in case of failure activates AP.
//    Mode 3 shows more debug information on serial port and display.
#define MAXWIFIDELAY 20

void loopWifiScan()
{
    if(config.wifi == 0)      // no Wifi
    {
        wifi_state = WIFI_DISABLED;
        return;
    }
    if(config.wifi == 1)    // station mode, setup in background
    {
        wifi_state = WIFI_DISABLED;  // will start scanning in wifiLoopBackgroiund
        return;
    }
    if(config.wifi == 2)    // AP mode, setup in background
    {
        startAP();
        return;
    }
    // wifi==3 => original mode with non-async wifi setup

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
        //        disp.rdis->drawString(0, lastl, "AP:             ");
        //        disp.rdis->drawString(6 * dispxs, lastl + 1, networks[0].id.c_str());
        delay(3000);
    }
    else
    {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        String localIPstr = WiFi.localIP().toString();
        Serial.println(localIPstr);
        //        sonde.setIP(localIPstr.c_str(), false);
        //        sonde.updateDisplayIP();
        wifi_state = WIFI_CONNECTED;
        delay(3000);
    }
    enableNetwork(true);
}
void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 12, 15);
    serialBT.begin("D-Star Beacon");
    sa.setDataOutput(&serialBT);
    display.init();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    bs.setHeaderBuffer(headerRXTXBuffer, DSTAR::RF_HEADER_SIZE * 2); //TODO refactore to constrctor

    Serial.println("Initializing SPIFFS");
    if(!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    Serial.println("Reading initial configuration");
    readConfig();

    setupWifiList();

    radio.reset();
    Serial.print(F("Initializing ... "));
    pinMode(LORA_IO2, OUTPUT);
    pinMode(LORA_IO1, INPUT);
    checkLoraState(radio.beginFSK(config.qrt, 4.8f, 4.8 * 0.25f, 25.0f, config.txPower, 48, false));
    //    MorseClient morse(&radio);
    //    morse.begin(f);
    //    morse.startSignal();
    //    morse.print("001");
    checkLoraState(radio.setEncoding(RADIOLIB_ENCODING_NRZ));
    checkLoraState(radio.setDataShaping(RADIOLIB_SHAPING_0_5));
    checkLoraState(radio.setSyncWord(syncWord, sizeof(syncWord)));

    repaintDisplay();

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
void loop()
{
    loopWifiBackground();

    while(gpsSerial.available() > 0)
    {
        auto ch = gpsSerial.read();
        gps.encode(ch);
    }
    if(gps.location.isValid() != isGPSValid)
    {
        isGPSValid = gps.location.isValid();
        repaintDisplay();
    }
    if(serialBT.connected() != isBTConnected)
    {
        isBTConnected =  serialBT.connected();
        repaintDisplay();
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
    if(!isPTTPressed &&
       gps.time.isUpdated() &&
       config.beaconInterval &&
       wifi_state == WIFI_CONNECTED &&
       gps.location.isValid())
    {
        auto newSecond = getGPSTime();
        //        Serial << newSecond << endl;
        if(newSecond % config.beaconInterval == 0
           && lastBEaconTime != newSecond)
        {
            lastBEaconTime = newSecond;
            prepareDPRS(formatDPRSString(config.callsign + "-" + config.callsignSuffix,
                                         gps.location.lat(), gps.location.lng(), gps.altitude.feet(),
                                         config.dprsMsg));
        }
    }
    if(!sa.comBuffer.isEmpty() && !isPTTPressed)
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
        radio.receiveDirect();//reset IRQ flags
        repaintDisplay();
        if(bs.haveHeader())
        {
            decodeHeader(headerRXTXBuffer);
        }
        startRX();
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
