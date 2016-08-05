# STM32-nRF8001 `

Project to build a driver for the nrf8001, using SPI/ACI, and implement the Nordic UART service (nus)

## Hardware
- STM32VL Discovery Board
- Adafruit nrf8001 breakout board

## Pinouts

| Signal Name   | STM32VL Brd   | nrf8001 Board |
| ------------- | ------------- | ------------- |
| SCLK          | PB13          | SCLK          |
| MISO          | PB14          | MISO          |
| MOSI          | PB15          | MOSI          |
| REQN          | PC0           | REQN          |
| RDYN          | PC1           | RDYN          |
| RSTN          | PC2           | RSTN          |
