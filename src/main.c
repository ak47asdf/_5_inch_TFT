// 5 inch TFT Display Interface
// Name: Sifat-Ul-Alam

#include <stdio.h>
#include <math.h>
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "Image_file.c"


#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define swap(a, b) { int16_t t = a; a = b; b = t; }
#define min(a,b) (((a)<(b))?(a):(b))

extern uint8_t temp [];
extern uint8_t font1 [];

extern uint8_t eco_mode[];
extern uint8_t fridge_mode[];
extern uint8_t super_mode[];
extern uint8_t defrost_mode[];

extern uint8_t walton_logo [];
extern uint8_t walton_logo_2 [];
extern uint16_t walton_x [];
extern uint8_t walton_text [];


///////////////////////////////////////////////////////////////////////
/////////////////////LOAD_CELL Parameter///////////////////////////////
///////////////////////////////////////////////////////////////////////

// this code is in git









///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
#define SSD1963_BLACK       0x0000      /*   0,   0,   0 */
#define SSD1963_NAVY        0x000F      /*   0,   0, 128 */
#define SSD1963_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define SSD1963_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define SSD1963_MAROON      0x7800      /* 128,   0,   0 */
#define SSD1963_PURPLE      0x780F      /* 128,   0, 128 */
#define SSD1963_OLIVE       0x7BE0      /* 128, 128,   0 */
#define SSD1963_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define SSD1963_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define SSD1963_BLUE        0x001F      /*   0,   0, 255 */
#define SSD1963_CYAN        0x07FF      /*   0, 255, 255 */
#define SSD1963_RED         0xF800      /* 255,   0,   0 */
#define SSD1963_MAGENTA     0xF81F      /* 255,   0, 255 */
#define SSD1963_GREEN       0x7E00       /* 255,   0, 255 */
#define SSD1963_YELLOW      0xFFE0      /* 255, 255,   0 */
#define SSD1963_WHITE       0xFFFF     /* 255, 255, 255 */
#define SSD1963_ORANGE      0xFD20      /* 255, 165,   0 */
#define SSD1963_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define SSD1963_PINK        0xF81F
#define SSD1963_DARKORANGE  0xFB60


#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
///////////////////////////////////////////////////////////////////////
#define SSD1963_COLUMN_ADDR			0x2A
#define SSD1963_PAGE_ADDR			  0x2B
#define SSD1963_GRAM				    0x2C
///////////////////////////////////////////////////////////////////////
//////////////////////////////Rotation/////////////////////////////////
///////////////////////////////////////////////////////////////////////
#define MEMCONTROL 0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

///////////////////////////////////////////////////////////////////////
#define SSD1963_WIDTH       800
#define SSD1963_HEIGHT      480
#define SSD1963_PIXEL_COUNT	SSD1963_WIDTH * SSD1963_HEIGHT
static bool _cp437    = false;
static uint8_t rotationNum=1;
///////////////////////////////////////////////////////////////////////
#define LCD_RST 4
#define LCD_CS  2
#define LCD_RS  3    
#define LCD_WR  40
#define LCD_RD  1
///////////////////////////////////////////////////////////////////////
/////////////////////////Parallel Interface////////////////////////////
///////////////////////////////////////////////////////////////////////
#define LCD_D0 5
#define LCD_D1 6
#define LCD_D2 7
#define LCD_D3 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12
#define LCD_D8  13
#define LCD_D9  14
#define LCD_D10 15
#define LCD_D11 16
#define LCD_D12 17
#define LCD_D13 45
#define LCD_D14 42
#define LCD_D15 41
/////////////////////////////////////////////////////////////////////
////////////////////////Load_cell////////////////////////////////////
/////////////////////////////////////////////////////////////////////










/////////////////////////////////////////////////////////////////////
//////////////////////////Parallel Interface/////////////////////////
/////////////////////////////////////////////////////////////////////
void Byte_TO_bits (uint16_t *bit_array, uint16_t dt)
{

for (uint16_t j=0;j<16;j++)
{
  bit_array[j] = dt&0x01;
  dt>>=1;
}

}
//////////////////////////////////////////////////////////////////////
// void LCD_write (uint16_t d)
// {


// gpio_set_level (LCD_WR,0);

//     uint16_t bits [16];
//     Byte_TO_bits (bits,d);

//      if (bits[0])
//      {gpio_set_level (LCD_D0, 1);} else {gpio_set_level (LCD_D0, 0);}
//      if (bits[1])
//      {gpio_set_level (LCD_D1, 1);} else {gpio_set_level (LCD_D1, 0);}
//      if (bits[2])
//      {gpio_set_level (LCD_D2, 1);} else {gpio_set_level (LCD_D2, 0);}
//      if (bits[3])
//      {gpio_set_level (LCD_D3, 1);} else {gpio_set_level (LCD_D3, 0);}
//      if (bits[4])
//      {gpio_set_level (LCD_D4, 1);} else {gpio_set_level (LCD_D4, 0);}
//      if (bits[5])
//      {gpio_set_level (LCD_D5, 1);} else {gpio_set_level (LCD_D5, 0);}
//      if (bits[6])
//      {gpio_set_level (LCD_D6, 1);} else {gpio_set_level (LCD_D6, 0);}
//      if (bits[7])
//      {gpio_set_level (LCD_D7, 1);} else {gpio_set_level (LCD_D7, 0);}
//      if (bits[8])
//      {gpio_set_level (LCD_D8, 1);} else {gpio_set_level (LCD_D8, 0);}
//      if (bits[9])
//      {gpio_set_level (LCD_D9, 1);} else {gpio_set_level (LCD_D9, 0);}
//      if (bits[10])
//      {gpio_set_level (LCD_D10, 1);} else {gpio_set_level (LCD_D10, 0);}
//      if (bits[11])
//      {gpio_set_level (LCD_D11, 1);} else {gpio_set_level (LCD_D11, 0);}
//      if (bits[12])
//      {gpio_set_level (LCD_D12, 1);} else {gpio_set_level (LCD_D12, 0);}
//      if (bits[13])
//      {gpio_set_level (LCD_D13, 1);} else {gpio_set_level (LCD_D13, 0);}
//      if (bits[14])
//      {gpio_set_level (LCD_D14, 1);} else {gpio_set_level (LCD_D14, 0);}
//      if (bits[15])
//      {gpio_set_level (LCD_D15, 1);} else {gpio_set_level (LCD_D15, 0);}

// gpio_set_level (LCD_WR,1);   

// }

void LCD_write (uint16_t pixel)
{
//gpio_set_level(LCD_CS, 0); // Assert Chip Select for the TFT

// Command part is written first, omitted here for brevity

// Send one 16-bit pixel data:

//gpio_set_level(LCD_RS, 1); // This is pixel data, not command/register (those only use D0..D7)

gpio_set_level(LCD_WR, 0); // Writing to TFT
// Start setting up the data pins,
gpio_set_level(LCD_D0, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D1, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D2, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D3, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D4, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D5, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D6, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D7, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D8, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D9, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D10, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D11, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D12, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D13, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D14, pixel & 1); pixel >>= 1;
gpio_set_level(LCD_D15, pixel & 1);
// and at this point, make sure at least 15ns passed
// since the previous comment (including data pin setting)
gpio_set_level(LCD_WR, 1); // Tell TFT to latch the parallel data.

}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
void LCD_command_WRITE (uint16_t command)
{

gpio_set_level (LCD_RS,0);

LCD_write (command);

}
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void LCD_data_WRITE (uint16_t data)
{
gpio_set_level (LCD_RS,1);

LCD_write (data); 
}
////////////////////////////////////////////////////////////////////////

void LCD_INIT (void)
{

   gpio_set_level (LCD_RST,1);
   vTaskDelay (1/portTICK_RATE_MS);
   gpio_set_level (LCD_RST,0);
   vTaskDelay (15/portTICK_RATE_MS);
   gpio_set_level (LCD_RST,1);
   vTaskDelay (15/portTICK_RATE_MS);

   gpio_set_level (LCD_CS,1);
   gpio_set_level (LCD_WR,1);
   gpio_set_level (LCD_RD,1);
   gpio_set_level (LCD_CS,0);

   ///////////////////////////////////////////////////






  LCD_command_WRITE(0xE2);   //PLL multiplier, set PLL clock to 120M
  LCD_data_WRITE(0x23);      //N=0x36 for 6.5M, 0x23 for 10M crystal
  LCD_data_WRITE(0x02);
  LCD_data_WRITE(0x54);

  LCD_command_WRITE(0xE0);   // PLL enable
  LCD_data_WRITE(0x01);
  vTaskDelay (0.1/portTICK_RATE_MS);

  LCD_command_WRITE(0xE0);
  LCD_data_WRITE(0x03);
  vTaskDelay (0.1/portTICK_RATE_MS);

  LCD_command_WRITE(0x01);   // software reset
  vTaskDelay (1/portTICK_RATE_MS);

  LCD_command_WRITE(0xE6);   // 0x03FFFF PLL setting for PCLK, depends on resolution
  LCD_data_WRITE(0x03);
  LCD_data_WRITE(0xFF);
  LCD_data_WRITE(0xFF);

  LCD_command_WRITE(0xB0);  //LCD SPECIFICATION
  LCD_data_WRITE(0x3F);     //0x24
  LCD_data_WRITE(0x20);     //0x20
  LCD_data_WRITE(0x03);     //0x03 Set HDP 799
  LCD_data_WRITE(0x1F);     //0x1F 
  LCD_data_WRITE(0x01);     //0x01 Set VDP 479
  LCD_data_WRITE(0xDF);     //0xDF
  LCD_data_WRITE(0x00);     //0x00 RGB

  LCD_command_WRITE(0xB4); //HSYNC
  LCD_data_WRITE(0x03);    //0x03 Set HT  928
  LCD_data_WRITE(0xA0);    //0xA0
  LCD_data_WRITE(0x00);    //0x00 Set HPS 46
  LCD_data_WRITE(0x2E);    //0x2E
  LCD_data_WRITE(0x30);    //0x30 Set HPW 48
  LCD_data_WRITE(0x00);    //0x00 Set LPS 15
  LCD_data_WRITE(0x0F);    //0x0F
  LCD_data_WRITE(0x00);    //0x00

  LCD_command_WRITE(0xB6); //VSYNC
  LCD_data_WRITE(0x02);    //0x02 Set VT  525
  LCD_data_WRITE(0x0D);    //0x0D
  LCD_data_WRITE(0x00);    //0x00 Set VPS 16
  LCD_data_WRITE(0x10);    //0x10
  LCD_data_WRITE(0x10);    //0x10 Set VPW 16
  LCD_data_WRITE(0x00);    //0x00 Set FPS 8
  LCD_data_WRITE(0x01);    //0x01

  LCD_command_WRITE(0xB8);
  LCD_data_WRITE(0x07);      //GPIO3=input, GPIO[2:0]=output
  LCD_data_WRITE(0x01);      //GPIO0 normal

  LCD_command_WRITE(0x36);   //rotation
  //LCD_data_WRITE(0x21 | MADCTL_BGR);
  //LCD_data_WRITE(0x22);		   // -- Set to 0x21 to rotate 180 degrees
	LCD_data_WRITE(MADCTL_MY | 0x21 | 0x22 | MADCTL_RGB); 


  LCD_command_WRITE (0x3A);   //Pixel format set
  LCD_data_WRITE (0x50);

  LCD_command_WRITE(0xF0);    //pixel data interface
  LCD_data_WRITE(0x03);       //8 bit bus //16 bit bus = 0x02/0x03
  vTaskDelay (1/portTICK_RATE_MS);

  LCD_command_WRITE(0xB8);
  LCD_data_WRITE(0x0f);     //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF 
  LCD_data_WRITE(0x01);     //GPIO0 normal

  LCD_command_WRITE(0xBA);
  LCD_data_WRITE(0x05);     //GPIO[0] out 1 --- LCD display on/off control PIN

  // LCD_command_WRITE(0xBC);
  // LCD_data_WRITE(0xFF);
  // LCD_data_WRITE(0xFF);
  // LCD_data_WRITE(0xFF);
  // LCD_data_WRITE(0x01);

  LCD_command_WRITE(0x29);   //display on

  LCD_command_WRITE(0xBE);   //set PWM for B/L
  LCD_data_WRITE(0x06);   //0x06
  LCD_data_WRITE(0xFF);   //0xF0
  LCD_data_WRITE(0x01);
  LCD_data_WRITE(0xf0);   //0xF0
  LCD_data_WRITE(0x00);
  LCD_data_WRITE(0x00);

  LCD_command_WRITE(0xD0); 
  LCD_data_WRITE(0x6D);   //0x0D

  LCD_command_WRITE(0x2C); 
}

/////////////////////////////////////////////////////////////////////////
////////////////////////////Address_Pin//////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void Address_pin_set (uint16_t y1, uint16_t y2, uint16_t x1, uint16_t x2)
{

LCD_command_WRITE (0x2A);  //Column address set
LCD_data_WRITE (y1>>8);
LCD_data_WRITE (y1);
LCD_data_WRITE (y2>>8);
LCD_data_WRITE (y2);

LCD_command_WRITE (0x2B);  //Page address set
LCD_data_WRITE (x1>>8);
LCD_data_WRITE (x1);
LCD_data_WRITE (x2>>8);
LCD_data_WRITE (x2);

LCD_command_WRITE (0x2C);

}


void SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) 
{

  LCD_command_WRITE (SSD1963_COLUMN_ADDR);
  LCD_data_WRITE(x1>>8);
  LCD_data_WRITE(x1 );
  LCD_data_WRITE(x2>>8);
  LCD_data_WRITE(x2 );

  LCD_command_WRITE (SSD1963_PAGE_ADDR);
  LCD_data_WRITE(y1>>8);
  LCD_data_WRITE(y1 & 0xFF); //& OxFF
  LCD_data_WRITE(y2>>8);
  LCD_data_WRITE(y2 & 0xFF); //& 0xFF
  LCD_command_WRITE (SSD1963_GRAM);
}

void SetCursorPosition_2 (uint16_t y1, uint16_t x1, uint16_t y2, uint16_t x2) 
{

  LCD_command_WRITE (SSD1963_COLUMN_ADDR);
  LCD_data_WRITE(x1>>8);
  LCD_data_WRITE(x1 );
  LCD_data_WRITE(x2>>8);
  LCD_data_WRITE(x2 );

  LCD_command_WRITE (SSD1963_PAGE_ADDR);
  LCD_data_WRITE(y1>>8);
  LCD_data_WRITE(y1 & 0xFF); //& OxFF
  LCD_data_WRITE(y2>>8);
  LCD_data_WRITE(y2 & 0xFF); //& 0xFF
  LCD_command_WRITE (SSD1963_GRAM);
}
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Draw_Pixel///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void Draw_Pixel (int16_t x, int16_t y, uint16_t color) 

{
  gpio_set_level (LCD_CS,0); //Chip Select active

  SetCursorPosition   (y, y+1, x, x+1);
//Address_pin_set   (y, y+1, x, x+1);
  LCD_command_WRITE(0x2C);
  LCD_data_WRITE(color);
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////Draw_Line///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void Draw_line (void)
{
for (int i=10; i<310; i++)
     {
       for (int j=0; j<5; j++)
       {
        //Draw_Pixel(j+235, i, SSD1963_BLUE);
        Draw_Pixel (j+235, i, SSD1963_RED);
       }
       
      } 
}

void SSD1963_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      Draw_Pixel(y0, x0, color);
    } else {
      Draw_Pixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}	


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      Draw_Pixel(y0, x0, color);
    } else {
      Draw_Pixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void SSD1963_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	SSD1963_drawLine(x, y, x+w-1, y, color);
}

void SSD1963_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	SSD1963_drawLine(x, y, x, y+h-1, color);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Draw_Circle///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void SSD1963_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  Draw_Pixel(x0  , y0+r, color);
  Draw_Pixel(x0  , y0-r, color);
  Draw_Pixel(x0+r, y0  , color);
  Draw_Pixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    Draw_Pixel(x0 + x, y0 + y, color);
    Draw_Pixel(x0 - x, y0 + y, color);
    Draw_Pixel(x0 + x, y0 - y, color);
    Draw_Pixel(x0 - x, y0 - y, color);
    Draw_Pixel(x0 + y, y0 + x, color);
    Draw_Pixel(x0 - y, y0 + x, color);
    Draw_Pixel(x0 + y, y0 - x, color);
    Draw_Pixel(x0 - y, y0 - x, color);
  }
}
 void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
	int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      Draw_Pixel(x0 + x, y0 + y, color);
      Draw_Pixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      Draw_Pixel(x0 + x, y0 - y, color);
      Draw_Pixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      Draw_Pixel(x0 - y, y0 + x, color);
      Draw_Pixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      Draw_Pixel(x0 - y, y0 - x, color);
      Draw_Pixel(x0 - x, y0 - y, color);
    }
  }
}

 void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      SSD1963_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      SSD1963_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      SSD1963_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      SSD1963_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

void SSD1963_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	SSD1963_drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////Rectangle_Draw///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void SSD1963_Fill_Rect_2(unsigned int x0,unsigned int y0, unsigned int x1,unsigned int y1, uint16_t color) { 
	uint32_t n = ((x1+1)-x0)*((y1+1)-y0);
	if (n>SSD1963_PIXEL_COUNT) n=SSD1963_PIXEL_COUNT;
	SetCursorPosition_2(x0, y0, x1, y1);
	while (n) {
			n--;
      
			LCD_data_WRITE(color);
	}
}

void SSD1963_Fill_Rect(unsigned int x0,unsigned int y0, unsigned int x1,unsigned int y1, uint16_t color) { 
	uint32_t n = ((x1+1)-x0)*((y1+1)-y0);
	if (n>SSD1963_PIXEL_COUNT) n=SSD1963_PIXEL_COUNT;
	SetCursorPosition (x0, y0, x1, y1);
	while (n) {
			n--;
      
			LCD_data_WRITE(color);
	}
}

 void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) 
  {
  // smarter version
  SSD1963_drawFastHLine(x+r  , y    , w-2*r, color); // Top
  SSD1963_drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  SSD1963_drawFastVLine(x    , y+r  , h-2*r, color); // Left
  SSD1963_drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
  }

void fillRect (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
  {
  // Update in subclasses if desired!
    for (int16_t i=x; i<x+w; i++) 
   {
    SSD1963_drawFastVLine (i, y, h, color);
   }

  }

void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) 
{
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////Draw Triangle////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void SSD1963_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

void SSD1963_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    SSD1963_drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if(a > b) swap(a,b);
    SSD1963_drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if(a > b) swap(a,b);
    SSD1963_drawFastHLine(a, y, b-a+1, color);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////Set_Rotation////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void setRotation(uint8_t rotate)
{
	switch(rotate)
	{
		case 1:
			rotationNum = 1;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(0x22  | MADCTL_RGB);   // (0x21 | MADCTL_BGR);
			break;
		case 2:
			rotationNum = 2;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(0x00 | MADCTL_RGB);   // (0x00 | MADCTL_BGR);
			break;
		case 3:
			rotationNum = 3;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(MADCTL_MX | 0x21 | 0x22 | MADCTL_RGB);   // (0x22 | MADCTL_BGR);
			break;
		case 4:
			rotationNum = 4;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_RGB);
			break;
    case 5:
			rotationNum = 5;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(0x03  | MADCTL_RGB);  // (0x03 | MADCTL_BGR);
			break;
		default:
			rotationNum = 1;
			LCD_command_WRITE(MEMCONTROL);
			LCD_data_WRITE(MADCTL_MY | MADCTL_RGB);
			break;
	}
}
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Bitmap//////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void draw_Bitmap (int16_t x, int16_t y,const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte = 0;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++) {
      if(i & 7) 
      {byte<<=1;}
      else     
      {byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);}
      if(byte & 0x80) 
      {Draw_Pixel (x+i, y+j, color);}
    }
  }
}

void drawRGBBitmap_1 (int16_t x, int16_t y, uint16_t *bitmap,int16_t w, int16_t h) 
{
  
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      Draw_Pixel(x + i, y, bitmap[j * w + i]);
    }
  }
 
}

void draw_RGB_Bitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h) 
{
  
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      Draw_Pixel(x + i, y, pgm_read_word(&bitmap[j * w + i]));
    }
  }
  
}

void printImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data, uint32_t size)
{
	uint32_t n = size;
	SetCursorPosition(x, y, w+x-1, h+y-1);
	for(uint32_t i=0; i<n ; i++)
	{
		LCD_data_WRITE(data[i]);
    vTaskDelay(4/portTICK_RATE_MS);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////Rectangle_Offset///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void offset(void)
{

      for (int i=0; i<480; i++)
     {
       for (int j=0; j<5; j++)
       {
        Draw_Pixel(i, j, SSD1963_WHITE);
        Draw_Pixel(i, j+315, SSD1963_WHITE);
        Draw_Pixel(j, i, SSD1963_WHITE);
        Draw_Pixel(j+475, i, SSD1963_WHITE);

       }
     }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Text_Print//////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void SSD1963_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	if((x >= SSD1963_WIDTH)            || // Clip right
     (y >= SSD1963_HEIGHT)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

  for (int8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5) 
      line = 0x0;
    else 
      line = pgm_read_byte(font1+(c*5)+i); ///////////////////////////////////////////////////////
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          Draw_Pixel(x+i, y+j, color);
        else {  // big size
          SSD1963_Fill_Rect(x+(i*size), y+(j*size), size + x+(i*size), size+1 + y+(j*size), color);
        } 
      } else if (bg != color) {
        if (size == 1) // default size
          Draw_Pixel(x+i, y+j, bg);
        else {  // big size
          SSD1963_Fill_Rect(x+i*size, y+j*size, size + x+i*size, size+1 + y+j*size, bg);
        }
      }
      line >>= 1;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void printText(char text[], int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size)
{
	int16_t offset;
	offset = size*6;
	for(uint16_t i=0; i<40 && text[i]!=NULL; i++)
	{
		SSD1963_drawChar(x+(offset*i), y, text[i],color,bg,size);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////Fill_screen////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void SSD1963_Fill(uint16_t color) 
{
	uint32_t n = SSD1963_PIXEL_COUNT;
	
	if(rotationNum==1 || rotationNum==3)
	{
		SetCursorPosition_2(0, 0,   SSD1963_WIDTH -1, SSD1963_HEIGHT -1);
	}
	else if(rotationNum==2 || rotationNum==4)
	{
		SetCursorPosition_2(0, 0, SSD1963_HEIGHT -1, SSD1963_WIDTH -1);
	}
	
	while (n) {
			n--;
       //LCD_data_WRITE(color>>8);
			 LCD_data_WRITE(color);
	}
}


void fill_screen (void)
{
for (int i=10; i<241; i++)
     {
     for (int j=0; j<300; j++)
       {
         
        Draw_Pixel(i, j+10, SSD1963_WHITE);
        Draw_Pixel(480-i, j+10, SSD1963_WHITE);
      
       }
     }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////Custom_Bar_GFX//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void custom_slide_gfx (void)

{


      for (int i=430; i<450; i++)   
     {
       for (int j=0; j<2; j++)
       {
        Draw_Pixel(i, j+30, SSD1963_BLUE);
        Draw_Pixel(i, j+290, SSD1963_BLUE);
       }
       
      } 
      for (int i=30; i<292; i++)
     {
       for (int j=0; j<2; j++)
       {
        Draw_Pixel(j+430, i, SSD1963_BLUE);
        Draw_Pixel(j+450, i, SSD1963_BLUE);
       }
       
      } 

       for (int i=423; i<430; i++)   
     {
       for (int j=0; j<2; j++)
       {
        Draw_Pixel(i, j+56, SSD1963_BLUE);
        Draw_Pixel(i, j+82, SSD1963_BLUE);
        Draw_Pixel(i, j+108, SSD1963_BLUE);
        Draw_Pixel(i, j+134, SSD1963_BLUE);
        Draw_Pixel(i, j+160, SSD1963_BLUE);
        Draw_Pixel(i, j+186, SSD1963_BLUE);
        Draw_Pixel(i, j+212, SSD1963_BLUE);
        Draw_Pixel(i, j+238, SSD1963_BLUE);
        Draw_Pixel(i, j+264, SSD1963_BLUE);
      
       }
     }
       for (int i=450; i<457; i++)   
     {
       for (int j=0; j<2; j++)
       {
        Draw_Pixel(i, j+56, SSD1963_BLUE);
        Draw_Pixel(i, j+82, SSD1963_BLUE);
        Draw_Pixel(i, j+108, SSD1963_BLUE);
        Draw_Pixel(i, j+134, SSD1963_BLUE);
        Draw_Pixel(i, j+160, SSD1963_BLUE);
        Draw_Pixel(i, j+186, SSD1963_BLUE);
        Draw_Pixel(i, j+212, SSD1963_BLUE);
        Draw_Pixel(i, j+238, SSD1963_BLUE);
        Draw_Pixel(i, j+264, SSD1963_BLUE);
      
       }

      } 

     }

 ///////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////Float to String////////////////////////////////   
 ///////////////////////////////////////////////////////////////////////////////
 void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

 int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
  
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
  
    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
  
    // Extract floating part
    float fpart = n - (float)ipart;
  
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
  
    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot
  
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter 
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
  
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
 void draw_bar(int x_bar, int y_bar) 
{
  fillRoundRect(x_bar, y_bar, 35, 190, 5, SSD1963_BLACK); //20,60
  fillRoundRect(x_bar+4, y_bar+4, 27, 182, 2, SSD1963_WHITE);

  drawLine(x_bar+37, y_bar+8, x_bar+42, y_bar+8, SSD1963_RED);
  drawLine(x_bar+37, y_bar+95, x_bar+42, y_bar+95, SSD1963_RED);
  drawLine(x_bar+37, y_bar+180, x_bar+42, y_bar+180, SSD1963_RED);

  printText ("15", x_bar+47, y_bar+4, SSD1963_RED, SSD1963_WHITE, 1);
  printText ("7", x_bar+47, y_bar+92, SSD1963_RED, SSD1963_WHITE, 1);
  printText ("0", x_bar+47, y_bar+178, SSD1963_RED, SSD1963_WHITE, 1);
} 

 int x_bar_t = 20;
 int y_bar_t = 60;

 void draw_Circular_bar (int x_bar, int y_bar) 
{
 
 
  SSD1963_fillCircle (x_bar+17, y_bar+140, 30, SSD1963_BLACK);
  SSD1963_fillCircle (x_bar+17, y_bar+140, 25, SSD1963_WHITE);

  SSD1963_fillCircle (x_bar+17, y_bar+140, 21, SSD1963_DARKORANGE);

} 



unsigned int rainbow(uint8_t value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red
 
  uint8_t red = 0;   // Red is the top 5 bits of a 16 bit colour value
  uint8_t green = 0; // Green is the middle 6 bits
  uint8_t blue = 0;  // Blue is the bottom 5 bits
 
  uint8_t quadrant = value / 32;
 
  if (quadrant == 0)
  {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1)
  {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2)
  {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3)
  {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


long map(long x, long in_min, long in_max, long out_min, long out_max) 

{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, uint8_t scheme) //const char *units

{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
 
  x += r;
  y += r; // Calculate coords of centre of ring
 
  int w = r / 3; // Width of outer ring is 1/4 of radius
 
  int angle = 150; // Half the sweep angle of meter (300 degrees)
 
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
 
  uint8_t seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  uint8_t inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
 
  // Variable to save "value" text colour from scheme and set default
  int colour = SSD1963_BLUE;
 
  // Draw colour blocks every inc degrees
  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc)
  {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;
 
    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;
 
    if (i < v)
    { // Fill in coloured segments with 2 triangles
      switch (scheme)
      {
      case 0:
        colour = SSD1963_RED;
        break; // Fixed colour
      case 1:
        colour = SSD1963_GREEN;
        break; // Fixed colour
      case 2:
        colour = SSD1963_BLUE;
        break; // Fixed colour
      case 3:
        colour = rainbow(map(i, -angle, angle, 0, 127));
        break; // Full spectrum blue to red
      case 4:
        colour = rainbow(map(i, -angle, angle, 70, 127));
        break; // Green to red (high temperature etc)
      case 5:
        colour = rainbow(map(i, -angle, angle, 127, 63));
        break; // Red to green (low battery etc)
      default:
        colour = SSD1963_BLUE;
        break; // Fixed colour
      }
      SSD1963_fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      SSD1963_fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      //text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      SSD1963_fillTriangle(x0, y0, x1, y1, x2, y2, SSD1963_WHITE);
      SSD1963_fillTriangle(x1, y1, x2, y2, x3, y3, SSD1963_WHITE);
    }
  }

 return x + r;

}


///////////////////////////////////////////////////////////////////////////
////////////////////////////Main_Function//////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void app_main(void)
{
 
     gpio_pad_select_gpio(LCD_D0);
     gpio_pad_select_gpio(LCD_D1);
     gpio_pad_select_gpio(LCD_D2);
     gpio_pad_select_gpio(LCD_D3);
     gpio_pad_select_gpio(LCD_D4);
     gpio_pad_select_gpio(LCD_D5);
     gpio_pad_select_gpio(LCD_D6);
     gpio_pad_select_gpio(LCD_D7);
     gpio_pad_select_gpio(LCD_D8);
     gpio_pad_select_gpio(LCD_D9);
     gpio_pad_select_gpio(LCD_D10);
     gpio_pad_select_gpio(LCD_D11);
     gpio_pad_select_gpio(LCD_D12);
     gpio_pad_select_gpio(LCD_D13);
     gpio_pad_select_gpio(LCD_D14);
     gpio_pad_select_gpio(LCD_D15);

     gpio_set_direction (LCD_D0, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D1, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D2, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D3, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D4, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D5, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D6, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D7, GPIO_MODE_OUTPUT);  
     gpio_set_direction (LCD_D8, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D9, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D10, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D11, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D12, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D13, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D14, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_D15, GPIO_MODE_OUTPUT);  


     gpio_pad_select_gpio(LCD_CS);
     gpio_pad_select_gpio(LCD_RD);
     gpio_pad_select_gpio(LCD_RS);
     gpio_pad_select_gpio(LCD_RST);
     gpio_pad_select_gpio(LCD_WR);

     gpio_set_direction (LCD_CS, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_RD, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_RS, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_WR, GPIO_MODE_OUTPUT);
     gpio_set_direction (LCD_RST, GPIO_MODE_OUTPUT);
  
     ////////////////////////////////////////////////////////////////////////////////////////
     ////////////////////////////LOAD_cell///////////////////////////////////////////////////
     ////////////////////////////////////////////////////////////////////////////////////////
   




     ////////////////////////////////////////////////////////////////////////////////////////
     ////////////////////////////////////////////////////////////////////////////////////////
     LCD_INIT ();

     //setRotation (1) ;
     SSD1963_Fill_Rect(0,0,800,480,SSD1963_WHITE);

     setRotation (3) ;
    //  draw_Bitmap(224, 209, walton_text, 352, 66,SSD1963_BLUE);
    //  vTaskDelay(1000/portTICK_RATE_MS);
    
    //  SSD1963_Fill_Rect(0,0,800,480,SSD1963_WHITE);
    //  fillRoundRect(5,4,470,41,6, SSD1963_DARKORANGE);
     //draw_Circular_bar(x_bar_t, y_bar_t);

     

      while (1)
    {

      //  LCD_INIT ();
       
 
      //  setRotation (4) ;
      //  SSD1963_Fill (SSD1963_WHITE);
      //  vTaskDelay(2000/portTICK_RATE_MS);

      //  LCD_command_WRITE(0xBE);   //set PWM for B/L
      //  LCD_data_WRITE(0x06);   //0x06
      //  LCD_data_WRITE(0x96);   //0x96
      //  LCD_data_WRITE(0x01);
      //  LCD_data_WRITE(0xf0);   //0xF0
      //  LCD_data_WRITE(0x00);
      //  LCD_data_WRITE(0x00);
      //  LCD_command_WRITE(0xd0); 
      //  LCD_data_WRITE(0x0d); 

      //  vTaskDelay(2000/portTICK_RATE_MS);

      //  LCD_command_WRITE(0xE5);
      //  vTaskDelay(2000/portTICK_RATE_MS);

          for(float j=0; j<=16;j++)
          {
          char buf[10];
          float fridge = 1.0+j;
          //char buff[20];
          float freezer = -15+j;

             ringMeter(fridge, 1, 17, 100, 150, 100, BLUE2RED);
             ringMeter(freezer, -15, 1, 500, 150, 100, BLUE2RED);

          snprintf (buf, sizeof(buf), "%0.1f", fridge);
          printText (buf, 160, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          

          // snprintf (buff, sizeof(buff), "%0.1f", freezer);
          // printText (buff, 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          // printText ("      ", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);

          if (freezer==0.00)
          {
          printText ("      ", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          printText ("0.00", 565, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
          else if (freezer == 1.00)
          {
          printText ("      ", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          printText ("1.00", 565, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -1.00)
          {
          printText ("-1.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -2.00)
          {
          printText ("-2.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -3.00)
          {
          printText ("-3.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -4.00)
          {
          printText ("-4.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -5.00)
          {
          printText ("-5.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -6.00)
          {
          printText ("-6.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -7.00)
          {
          printText ("-7.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -8.00)
          {
          printText ("-8.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -9.00)
          {
          printText ("-9.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -10.00)
          {
          printText ("-10.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -11.00)
          {
          printText ("-11.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -12.00)
          {
          printText ("-12.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -13.00)
          {
          printText ("-13.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -14.00)
          {
          printText ("-14.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
             else if (freezer == -15.00)
          {
          printText ("-15.0", 555, 235, SSD1963_BLACK, SSD1963_WHITE, 3);
          }
          
          else {}
          
          }
          //vTaskDelay (500/portTICK_RATE_MS);

       }
  

  
    

}