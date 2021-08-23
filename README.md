# esphome-hmlgw

This is a component for ESPHome that allows to emulate a Homematic LAN gateway
using a HM-MOD-RPI-PCB connected to the pins of an ESP module.
There is only limited testing being done on a WT32-ETH01 board and the LAN
connection. WiFi is untested, but should work.

## Implementation

The implementation is heavily based on two other projects:
  * https://github.com/oxan/esphome-stream-server
  * hmlangw, https://homematic-forum.de/forum/viewtopic.php?t=27705

Actually, this is a port of hmlangw to the ESP platform.

## Why?

If you use FHEM, the serial bridge could be used directly. However, if
Homegear is used, the serial bridge is not directly supported and, at best,
hacky. Using this HMLGW emulation, Homegear is happy.
