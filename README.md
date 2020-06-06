# Minimal UAVCAN Node (wave-canard)

Easy "clone and go" repository for a [UAVCAN v0] node based on [libopencm3] and [libcanard].

UAVCAN broadcast messages:
* `uavcan.protocol.NodeStatus` fixed at 1 Hz

UAVCAN provided services:
* `uavcan.protocol.GetNodeInfo`

It does nothing.
Use it as a base to build cool stuff.

# Hardware Setup

STM32F103

- PA11 CAN\_RX
- PA12 CAN\_TX

The code was tested on a
[UC4H General Purpose Node](https://github.com/olliw42/uavcan4hobbyists/tree/master/pcbs/UC4H%20General%20Purpose%20Node%202nd%20Generation)
and NUCLEO-F103RB.

# Instructions
 1. git clone --recurse-submodules https://github.com/libopencm3/libopencm3-template.git your-project
 2. cd your-project
 3. make -C libopencm3 # (Only needed once)
 4. make -C wave-canard

If you have an older git, or got ahead of yourself and skipped the ```--recurse-submodules```
you can fix things by running ```git submodule update --init``` (This is only needed once)

# Directories
* wave-canard contains the application

[libopencm3]: https://github.com/libopencm3/libopencm3
[libcanard]: https://github.com/UAVCAN/libcanard/tree/legacy-v0
[UAVCAN v0]: https://legacy.uavcan.org/
