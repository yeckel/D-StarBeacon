# D-StarBeacon

Simple D-Star transmitter for TTGO T-Beam ESP32 SX1278 written with Platformio. It uses DIO1,2 for data sending. 

Currently the D-Star header, D-Star message and D-GPS is properly encoded and sent every 20s.

For header viterbi&co code from Anthonys F4GOH
 (DSTAR)[https://github.com/f4goh/DSTAR] great work is used with some modification.

Have a look in (D-StarReceive)[https://github.com/yeckel/D-StarReceive] for complementary receiver.

## Warning
XTAL on board the SX1278 transceiver module has a random frequency offset which driffts with temperature! The offset could be measured by sending CW and tuning an SDR receiver 
into that frequency. (Note that CW mode at most radios has an tone offset like 600Hz) From my experience with SX1278 modules and IC-705 TRX
works GMSK at 4800bits/s fine at +/- 600Hz offset from the carrier frequency.

**I have currenty just 2 modules and they are 2kHz off frequency!**

# TODO
* <del> Encode possition from on board GPS
* Create some ??web?? interface forsetting 
    * Callsign
    * Coordsinates
    * D-Star message
    * DPRS - message
    * DPRS - symbol
    * beacon period
* ?Unite transmitter and receiver?
* ?Implement support for SMS and Image sending from RS-MS1 app? 
