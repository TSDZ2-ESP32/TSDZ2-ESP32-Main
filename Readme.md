# BT Interface for TSDZ2 Open Source Firmware

## HW Components
### ESP32 DevKit

![Alt text](img/ESP32DevKit.jpg?raw=true "ESP32DevKit")

### DC-DC StepDown

![Alt text](img/StepDown.png?raw=true "Step Down")

### Logic level converter 5v/3.3v

![Alt text](img/LogicLevelConverter.jpg?raw=true "Logic level converter")

### DS18B20 Temperature Sensor (Optional)

![Alt text](img/ds18b20.jpg?raw=true "DS18B20")

## Wiring scheme
![Alt text](img/Schema.png?raw=true "Schema")

**ESP32 Wiring**

|ESP32 GPIO | Name | ESP32 funct. | Function | 6 pin Cable color|
|-|-|-|-|-|
|GPIO 16|RX2|UART 2 RX|Controller TX|Brown|
|GPIO 17|TX2|UART 2 TX|Controller RX|Orange|
|GPIO 33|D33|UART 1 RX|LCD TX|Orange|
|GPIO 32|D32|UART 1 TX|LCD RX|Brown|
|GPIO 4|D4|One Wire In.|DS18B20 Signal|-|
|GPIO 1|TX0|UART 0 TX|USB debug RX|-|
|GPIO 3|RX0|UART 0 RX|USB debug TX|-|
