# C++ Port of the Espressif RMT 5.1 Driver for the ESP32 Family of Chips

## Overview

This repository contains a C++ port of the Espressif RMT LED strip component (version 2.2.5) specifically designed for the ESP32 family of chips. The code is compatible with the Arduino and PlatformIO ecosystems, allowing for easy integration into existing projects.

The library addresses several C-specific features in the original codebase that do not compile in standard C++. Future updates may include support for asynchronous RMT output to LED strips, enhancing performance by allowing parallel output operations.

## Features

- Compatible with Arduino and PlatformIO build systems.
- C++ port of the Espressif RMT LED strip component.
- Configurable for various LED strip models (e.g., WS2812).
- RGBW support


## Getting Started

### Prerequisites

Ensure you have the following installed:

- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- ESP32 board (e.g., ESP32-S3 DevKitC)

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/zackees/esp-rmt-led-strip-component-idf-5-1-cpp
   cd esp-rmt-led-strip-component-idf-5-1-cpp
   ```

2. Open `platformio.ini` in your preferred editor and configure it according to your board specifications if necessary.

3. Build and upload the project using PlatformIO or Arduino IDE.

### Usage

This repo will automatically activate it's own main.cpp located at [src/main.cpp](src/main.cpp).
If this is included in a library then this will be compiled out.

#### Example Configuration

See [src/main.cpp](src/main.cpp)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Happy coding!

