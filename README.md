# D-Star Beacon by OK1CHP

Simple D-Star transceiver for TTGO T-Beam ESP32 SX1278 written with Platformio. 

## Technical details and features
* It uses sx127x FSK Continuous direct Mode, PreambleDetect and SyncAddress on DIO0, Dclk(DIO1) and Data(DIO2)
* RX stop happens after D-Star ending frame or failed receiving frame sync packet.
* Decodes D-Star RF Header, 20 characters message and and sends DV Slow data payload to the Bluetooth Connection where it could be decoded with RS-MS1A or D-Rats or some other app.
* Could send DPRS possition report with coordinates from local GPS with configured timeout.
* Web interface is used to configure the transceiver.
* It looks for preconfigured WiFi ssid and password and eventually starts it's own hotspot.

## Licences
My code is available under BSD2. In some parts I was heavily inspired (an euphemism for Ctrl+C&Ctrl+V) by code of others open source projects. Sometimes whole libraries are used which are using other licences (GPL).

For header viterbi&co code from Anthonys F4GOH [DSTAR](https://github.com/f4goh/DSTAR) great work is used with some modification. Anthony also gave me some of his private code for inspiration. Tnx!  The configuration and WiFi code was heavily inspired by DL9RDZ [RDZ_TTGO_SONDE](https://github.com/dl9rdz/rdz_ttgo_sonde) Radio handling is done with user friendly [RadioLib](https://github.com/jgromes/RadioLib)

If you feel your copyrights might be offended, please don't heasitate contacting me. It's shall be fun.

## Sources wisdom
Probably best D-Star info sources:
[kb9mwr](https://www.qsl.net/kb9mwr/projects/dv/dstar/)
[Slow data](https://www.qsl.net/kb9mwr/projects/dv/dstar/Slow%20Data.pdf)
[MMDVMHost](https://github.com/g4klx/MMDVMHost)

# Usage
## Screens
### Status screen
<img src="doc/status.png" alt="Standby screen" width="200">

* __RX/TX__ - shows current status 
* __f__ - current frequency
* __GPS/gps__ - if it has GPS fix
* __BT/bt__ - if there is a Bluetooth connection
* __CS__ - own callsign
* __CP__ - companion callsing (also target callsign)
* __IP__ - current IP address to connect to (192.168.4.1 when in AP mode)
* __ssid__ - used WifiNetwork ssid
* __LA&LO__ - GPS coordinates
* __beacon in__ - time to next beacon, or permanent 0 when there is no GPS or beacon is disabled
* __Batt__ - battery voltage  

### Received screen
<img src="doc/received.png" alt="Received screen" width="200">

* __Cs__ - heard station callsign
* __Cp__ - companion callsign (also target)
* __Rig__ - 4 characters used as RIG id
* __Ds__ - destination callsign
* __Msg__ - D-Star 20 characters message
* __f er__ - frequency offset from the transmitter
* __to__ -  timeout until the Received screen is shown

### Web interface
You shall connect to the provided IP address with an browser and set your callsign and so on. The items shall be self-explanatory. Setting is stored locally into the file config.txt

# Installation
I'm using [Platformio](https://platformio.org/) for development. Have a look on their great documentation. Simple command ```pio run -t upload``` and ```pio run -t uploadfs``` shall work. Before uploading please configure your callsign in the __src/data/config.txt__ and your WiFi network(s) in __src/data/networks.txt__ The format is:
```
ssid network1
password network1
ssid network2
password network2
...
```
If no preconfigured network is found, then the first network from the list is created as local access point.

## HW requirement
The transceiver was developed on TTGO T-Beam 433MHz version with AXP power management. The older one with the manual power switch does work too. However mine has hard time getting GPS fix. Other ESP32 SX1278 boards could be used too, just check pins assignments and modify platformio.ini ```board = ttgo-lora32-v21``` works out of the box.

### Warning
XTAL on board the SX1278 transceiver module has a random frequency offset which driffts with temperature! The offset could be measured by sending CW and tuning an SDR receiver into that frequency. (Note that CW mode at most radios has an tone offset like 600Hz) From my experience with SX1278 modules and IC-705 TRX
works GMSK at 4800bits/s fine at +/- 600Hz offset from the carrier frequency. With AFC (automatic frequency correction) enabled set to this works fine to +/- 12kHz! Don't forget setting our frequency offset in web gui.

**I have currenty 4 modules and they are up to 4kHz off frequency!**

## TODO and also features
* <del> Encode possition from on board GPS
* <del> Decode D-Star header
* <del> Decode GPS coord?
* <del> Send same data to serial and BT as IC-705 does (images, text messages)
* <del> Unite transmitter with receiver into transceiver
* <del> Use SX1278 AFC (automatic frequency correction) to correct XTAL offset and thermal drifft
* Create some ??web?? interface for setting 
    * <del>Callsign
    * <del>D-Star message
    * <del>DPRS - message
    * <del>DPRS - symbol
    * <del>beacon period
* <del> Use D-Star sync frame to get into receive stream (low prio)
   * <del> Using for sync in RX DV fast data
   * <del> Not possible to start RX without preamble and prefix (SX1278 allows just one prefix)   
* Implement support for DV fast data - IC-705 sometimes switches to them even when turned off (SW Bug?)
   * <del> RX
   * TX
     * <del> Configuration  enable fast data, enable fast GPS
     * <del> encode and send
* <del>Implement setting RF offset with suggestion from the last AGC run
* <del>Show when the beacon is running with time to next TX
* <del>Show GPS coordinates in web and maybe display

## Notes
* D-Star SMS message is transmitted in format: ```$$Msg,OK1CHP,OK1LOL,001172ahoj 0xA2, 0xD, 0x0``` no clue what that 001172 means, RS-MS1A app handles it.
* D-Star Image block is transmitted as ```$$Pic,OK1CHP,OK1LOL,0111 0xEF, 0x80, 0x9, 0x7,_Z 0xEF, 0x8D,aQ@ 0x1F, 0xEF,g 0xD``` 
All those ICOM extensions are sent as a message type 0x30 and are thus dumped directly into serial port

The official ICOM app RS-MS1A for Android could be used to communicate via Bluetooth (model Others) It's then receiving and sending the data from serial or bluetooth port. ```$$Msg,OK1CHP,OK1LOL,001172ahoj  0xC2, 0xD, 0x0,``` 

Picture 160x120 (19200px) is sent with 10x8 (80) blocks with 25% quality -> 240px/block, one block has 18bytes but 4 of them (0xEF, 0x80, 0x9, 0x7) represents the format. (??,??,column,row)
  * 0111 0xEF, 0x80, 0x09, 0x7  ---25%, 160x120
  * 0212 0x01, 0x09, 0x07 --- 50%, 160x120
  * 0313 0x02, 0x09, 0x07,--- 75%, 160x120
  * 0414 0x12, 0x27, 0x1D,--- 25%, 640x480
 
* There are probably two types of image sending. 
 * When is the image sent from RS-MS1A -> IC705 -> ESP32 -> RS-MS1A then it works.
 * Sending from IC-705 when PTT is pressed (together with voice) works fine as well.
 * Sending whole image from IC-705 does not work. It sens for me unknown data (is that maybe DV Fast):
```U0x85, 0x24, 0x24, u0x83, 0x43, 0x52, 
U0x85, 0x31, 0x46, u0x83, 0x34, 0x44, 
U0x85, 0x4F, 0x4B, u0x83, 0x31, 0x43, 
U0x85, 0x50, 0x2D, u0x83, 0x31, 0x3E, 
U0x85, 0x50, 0x49, u0x83, 0x37, 0x30, 
U0x85, 0x2C, 0x44, u0x83, 0x53, 0x54, 
U0x94, 0x52, 0x2A, u0x83, 0x3A, 0x21, 
U0x94, 0x33, 0x45, u0x83, 0x2D, 0x2F,
U0x94, 0x77, 0x69, u0x83, 0x74, 0x68, 
U0x94, 0x31, 0x43, u0x83, 0x48, 0x50, 
U0x8C, 0x83, 0x39, u0x83, 0xE2, 0x9A, 
```
