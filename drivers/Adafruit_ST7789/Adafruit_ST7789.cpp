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

#include "mbed.h"
#include "Adafruit_ST7789.h"

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

// Constructor 
Adafruit_ST7789::Adafruit_ST7789(PinName mosi, PinName miso, PinName sck, PinName cs, PinName rs, PinName rst) 
        : lcdPort(mosi, miso, sck), _cs(cs), _rs(rs), _rst(rst), Adafruit_GFX(WIDTH, HEIGHT) 
{ }


void Adafruit_ST7789::writecommand(uint8_t c)
{
    _rs = 0;
    _cs = 0;
    lcdPort.write( c );
    _cs = 1;
}


void Adafruit_ST7789::writedata(uint8_t c)
{
    _rs = 1;
    _cs = 0;
    lcdPort.write( c );

    _cs = 1;
}


// Initialization for ST7789 screens
void Adafruit_ST7789::init(void)
{
    lcdPort.format(8,3);
    lcdPort.frequency(8000000);
    
    _rst = 0;
    wait_ms(50);
    _rst = 1;
    wait_ms(100);
    
    writecommand(ST7789_SWRESET);   
    wait_ms(150);
    writecommand(ST7789_SLPOUT);    
    wait_ms(500);
    writecommand(ST7789_COLMOD);    
    writedata(0x55);
    wait_ms(10);
    writecommand(ST7789_MADCTL);    
    writedata(0x00);
    writecommand(ST7789_CASET);
    writedata(0x00);
    writedata(0);
    writedata(HEIGHT >> 8);
    writedata(HEIGHT & 0xFF);
    writecommand(ST7789_RASET);
    writedata(0x00);
    writedata(0);
    writedata(WIDTH >> 8);
    writedata(WIDTH & 0xFF);
    writecommand(ST7789_INVON);
    wait_ms(10);
    writecommand(ST7789_NORON);
    wait_ms(10);
    writecommand(ST7789_DISPON);
    wait_ms(500);
    
    writecommand(ST7789_DISPOFF);
    wait_ms(500);
    writecommand(ST7789_DISPON);
    wait_ms(500);
    writecommand(ST7789_DISPOFF);
    wait_ms(500);
    writecommand(ST7789_DISPON);
    wait_ms(500);
    writecommand(ST7789_DISPOFF);
    wait_ms(500);
    writecommand(ST7789_DISPON);
    wait_ms(500);
}

void Adafruit_ST7789::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
                                    uint8_t y1)
{

    writecommand(ST7789_CASET); // Column addr set
    writedata(0x00);
    writedata(x0);     // XSTART
    writedata(0x00);
    writedata(x1);     // XEND

    writecommand(ST7789_RASET); // Row addr set
    writedata(0x00);
    writedata(y0);     // YSTART
    writedata(0x00);
    writedata(y1);     // YEND

    writecommand(ST7789_RAMWR); // write to RAM
}


void Adafruit_ST7789::pushColor(uint16_t color)
{
    _rs = 1;
    _cs = 0;

    lcdPort.write( color >> 8 );
    lcdPort.write( color );
    _cs = 1;
}


void Adafruit_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color)
{

    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x, y, x+1, y+1);
    _rs = 1;
    _cs = 0;
    lcdPort.write(color >> 8);
    lcdPort.write(color & 0xFF);
}


void Adafruit_ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                    uint16_t color)
{

    // Rudimentary clipping
    if((x >= _width) || (y >= _height)) return;
    if((y+h-1) >= _height) h = _height-y;
    setAddrWindow(x, y, x, y+h-1);

    uint8_t hi = color >> 8, lo = color;
    _rs = 1;
    _cs = 0;
    while (h--) {
        lcdPort.write( hi );
        lcdPort.write( lo );
    }
    _cs = 1;
}


void Adafruit_ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                    uint16_t color)
{

    // Rudimentary clipping
    if((x >= _width) || (y >= _height)) return;
    if((x+w-1) >= _width)  w = _width-x;
    setAddrWindow(x, y, x+w-1, y);

    uint8_t hi = color >> 8, lo = color;
    _rs = 1;
    _cs = 0;
    while (w--) {
        lcdPort.write( hi );
        lcdPort.write( lo );
    }
    _cs = 1;
}



void Adafruit_ST7789::fillScreen(uint16_t color)
{
    fillRect(0, 0,  _width, _height, color);
}


// fill a rectangle
void Adafruit_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color)
{

    // rudimentary clipping (drawChar w/big text requires this)
    if((x >= _width) || (y >= _height)) return;
    if((x + w - 1) >= _width)  w = _width  - x;
    if((y + h - 1) >= _height) h = _height - y;

    setAddrWindow(x, y, x+w-1, y+h-1);

    uint8_t hi = color >> 8, lo = color;
    _rs = 1;
    _cs = 0;
    for(y=h; y>0; y--) {
        for(x=w; x>0; x--) {
            lcdPort.write( hi );
            lcdPort.write( lo );
        }
    }

    _cs = 1;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ST7789::Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}



void Adafruit_ST7789::setRotation(uint8_t m) {

//  writecommand(ST7789_MADCTL);
//  rotation = m % 4; // can't be higher than 3
}


void Adafruit_ST7789::invertDisplay(boolean i)
{
    writecommand(i ? ST7789_INVON : ST7789_INVOFF);
}


