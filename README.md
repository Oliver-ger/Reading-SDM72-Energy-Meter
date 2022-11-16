# Reading-SDM72-Energy-Meter
Reading EnergyMeter SDM72 values with ModBus and publish them per MQTT
Needed Hardware: Wemos D1 Mini or NodeMCU and TTL to RS485 Adapter
__________         ________________
NodeMCU   |        |  TTL to RS485 |
          |        |               |
     3.3V |--------| VCC        A+ |
     GND  |--------| GND        B- |
D5 GPIO14 |--------| RXD           |
D6 GPIO12 |--------| TXD           |
__________|        |_______________|
