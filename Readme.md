# BT Interface for TSDZ2 Open Source Firmware

This reposiitory contains the source code of the ESP32 board

Wiki page at 
[wiki](https://github.com/TSDZ-ESP32/TSDZ-ESP32-wiki/wiki)

## Requirements
- ESP-IDF 4.4

## Development
If you want to build the project yourself or change some lines of code here is a quick start
- Install [VSCode](https://code.visualstudio.com/) (On windows you might need the ``Visual C++ Build Tools``)
- [Download](https://github.com/TSDZ2-ESP32/TSDZ2-ESP32-Main/archive/refs/heads/master.zip) and extract the repository
- Open the folder with VSCode, the editor will ask you to install recommended extensions. Install them all.
- The ESP-IDF extension will download/setup everything. Choose the "Advanced" setup option and select ESP-IDF version 4.4.X
- If no errors show up you can compile the project now.
- Click on the ``ESP-IDF Build Project`` button at the bottom left (blue bar) or press ``Ctrl + E + B``
- If there are no errors, you will get a ``TSDZ2-ESP32-Main.bin`` at the build directory
