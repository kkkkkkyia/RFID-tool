# RFID-tool

## Project Description
This project implements a dual-interface RFID access control system on a Raspberry Pi, supporting both RS485 and Wiegand 26-bit protocols. It performs UID validation, real-time HMI updates over Modbus RTU, and multithreaded logging of access events with timestamps and optional fueling data.

## How Wiegand & RS485 work in This Project

### Wiegand

- Hardware Connection: D0 → Connect to Raspberry Pi GPIO; D1 → Connect to another GPIO

- Signal Decoding: Wiegand transmits one bit at a time using D0 (0) and D1 (1) pulses and code listens for falling edges and reconstructs the 26-bit UID

### RS485

- Hardware Connection: RS485 A/B lines connected to USB-RS485 converter; Serial interface appears as /dev/ttyUSB0

- Signal Decoding: RS485 reader sends UID as an ASCII or hex string over UART; Baud rate, data bits, and stop bits must match reader settings