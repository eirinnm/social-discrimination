# Dynamixel Rotation Controller

Firmware for driving a Dynamixel servo using a single pin. It uses [Dynamixel2Arduino](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/) from Robotis, with an interrupt tied to the "direction" pin. The interrupt engages or disengages the UART TX pin, allowing for half-duplex signalling.

The reference board is a custom PCB designed for an Adafruit Itsy Bitsy 32u4 5V, with 3 momentary buttons. PCB design files are in the repo.


