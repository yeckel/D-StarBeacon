# D-Star Beacon by OK1CHP

Simple D-Star transceiver for TTGO T-Beam ESP32 SX1278 written with Platformio. 

## Technical details
* It uses sx127x FSK Continuous Mode, PreambleDetect and SyncAddress on DIO0, Dclk(DIO1) and Data(DIO2)
* D-Star ending frame is caught by local configurable shift register
* D-Star sync frames are used to break RX when sync lost
* Decodes D-Star RF Header, 20 characters message and and sends DV Slow data payload to the Bluetooth Connection where it could be decoded with RS-MS1A
 * Text messages are working fine
 * Images are loosing synchronization after some time, smaller ones are working fine. ??Freq drift??

Currently the D-Star header, D-Star message and D-GPS is properly encoded and sent every 20s.

For header viterbi&co code from Anthonys F4GOH [DSTAR](https://github.com/f4goh/DSTAR) great work is used with some modification. The configuration and WiFi code was heavily inspired by DL9RDZ [RDZ_TTGO_SONDE](https://github.com/dl9rdz/rdz_ttgo_sonde) Radio handling is done with user friendly [RadioLib](https://github.com/jgromes/RadioLib)

Probably best D-Star info sources:
[kb9mwr](https://www.qsl.net/kb9mwr/projects/dv/dstar/)
[Slow data](https://www.qsl.net/kb9mwr/projects/dv/dstar/Slow%20Data.pdf)

Have a look in (D-StarReceive)[https://github.com/yeckel/D-StarReceive] for complementary receiver.

## Warning
XTAL on board the SX1278 transceiver module has a random frequency offset which driffts with temperature! The offset could be measured by sending CW and tuning an SDR receiver 
into that frequency. (Note that CW mode at most radios has an tone offset like 600Hz) From my experience with SX1278 modules and IC-705 TRX
works GMSK at 4800bits/s fine at +/- 600Hz offset from the carrier frequency.

**I have currenty just 2 modules and they are 2kHz off frequency!**

## TODO
* <del> Encode possition from on board GPS
* <del> Decode D-Star header
* <del> Decode GPS coord?
* <del> Send same data to serial and BT as IC-705 does (images, text messages)
* <del> Unite transmitter with receiver into transceiver
* Use SX1278 AFC (automatic frequency correction) to correct XTAL offset and thermal drifft
* Create some ??web?? interface for setting 
    * <del>Callsign
    * Coordsinates source
    * <del>D-Star message
    * <del>DPRS - message
    * DPRS - symbol
    * <del>beacon period
* Use D-Star sync frame to get into receive stream (low prio)
* Implement support for DV fast data

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
