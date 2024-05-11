# esphome-hmlgw

This is a component for ESPHome that allows to emulate a Homematic LAN gateway
using a HM-MOD-RPI-PCB connected to the pins of an ESP module.
There is only limited testing being done on a WT32-ETH01 board and the LAN
connection. WiFi is untested, but should work.

If you like my work, consider sponsoring this project via [Github Sponsors](https://github.com/sponsors/andyboeh) or by acquiring one of my [Amazon Wishlist](https://www.amazon.de/hz/wishlist/ls/ROO2X0G63PCT?ref_=wl_share) items.

## Implementation

The implementation is heavily based on three other projects:
  * https://github.com/oxan/esphome-stream-server
  * hmlangw, https://homematic-forum.de/forum/viewtopic.php?t=27705
  * https://github.com/alexreinert/HB-RF-ETH/

Actually, this is a port of hmlangw to the ESP platform.

## Why?

If you use FHEM, the serial bridge could be used directly. However, if
Homegear is used, the serial bridge is not directly supported and, at best,
hacky. Using this HMLGW emulation, Homegear is happy.

Only quick testing has been performed on RaspberryMatic and FHEM.

## HB-RF-ETH

This component can also be used on HB-RF-ETH to support LAN Gateway mode. The
following configuration is needed:

```
esphome:
  name: hb-rf-eth
  platform: ESP32
  board: esp-wrover-kit

# Enable logging
logger:

# Enable Home Assistant API
api:
  reboot_timeout: 0s

ota:

ethernet:
  type: LAN8720
  mdc_pin: GPIO32
  mdio_pin: GPIO33
  clk_mode: GPIO17_OUT
  phy_addr: 0
  power_pin: GPIO13

time:
  - platform: homeassistant
    id: homeassistant_time
    timezone: Europe/Vienna

external_components:
  - source: github://andyboeh/esphome-hmlgw
    components: hmlgw

status_led:
  pin: GPIO4

output:
  - platform: gpio
    id: power_led_output
    pin: GPIO16

light:
  - platform: binary
    name: "Power LED"
    output: power_led_output

binary_sensor:
  - platform: gpio
    pin: 
      number: GPIO34
      inverted: true
    name: "HM Button"

uart:
  id: uart_bus
  baud_rate: 115200
  tx_pin: 2
  rx_pin: 35

hmlgw:
  port: 2000
  keepalive_port: 2001
  reset_pin: 
    number: GPIO23
    inverted: true
```
