/***************************************************
  This is a library is a modification of the Mbed Adafruit_ST7735 library
  for 1.3" IPS displays commonly available, which are 240x240 and run
  on the ST7789.
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Modified by Sreeteja Jonnada.
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ADAFRUIT_ST7789H_
#define _ADAFRUIT_ST7789H_

#include "mbed.h"
#include "Adafruit_GFX.h"

#define boolean bool

#define WIDTH     240
#define HEIGHT     240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0

#define ST_CMD_DELAY   0x80    // special signifier for command lists

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD


class Adafruit_ST7789 : public Adafruit_GFX {

 public:

  Adafruit_ST7789(PinName mosi, PinName miso, PinName sck, PinName CS, PinName RS, PinName RST);

  void     init(void);
  void     setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
  void     pushColor(uint16_t color);

  void     fillScreen(uint16_t color);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void     drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void     invertDisplay(boolean i);

  void     setRotation(uint8_t r);
  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

 private:
  uint8_t  tabcolor;
  void     spiwrite(uint8_t),
           writecommand(uint8_t c),
           writedata(uint8_t d),
           commandList(uint8_t *addr),
           commonInit(uint8_t *cmdList);

    SPI lcdPort;            // does SPI MOSI, MISO and SCK
    DigitalOut _cs;         // does SPI CE
    DigitalOut _rs;         // register/date select
    DigitalOut _rst;        // does 3310 LCD_RST
};

#endif
