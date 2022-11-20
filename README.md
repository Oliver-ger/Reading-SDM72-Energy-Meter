# Reading SDM72 Energy Meter via ModBus and ESP8266
Reading EnergyMeter SDM72 values with ModBus and publish them per MQTT
Needed Hardware: Wemos D1 Mini or NodeMCU and TTL to RS485 Adapter.

Wiring is very simple - see photos.

Import and Export Energy/Power are multiplicated wit (-1) in the program,
because the wiring of In and Out at SDM72 are swapped due to intallation conditions in the panel.
