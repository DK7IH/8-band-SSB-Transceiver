///////////////////////////////////////////////////////////////////
//        Control software for 8-Band-short-wave radio           //
///////////////////////////////////////////////////////////////////
//Hardware:                                                      // 
//MCU: STM32F411 ("Black Pill Board")                            //
//LCD: ST7735 Color LCD                                          //
//DDS: AD9951 (400MHz clock)                                     //
//External EEPROM: 24C65                                         //
///////////////////////////////////////////////////////////////////
//  Compiler:         ARM GCC TOOLCHAIN                          //
//  Author:           Peter Baier (DK7IH)                        //
//                    JUL 2022                                   // 
///////////////////////////////////////////////////////////////////

//EEPROM:
//Bytes 
//170:128: Frequency data for 8 bands (7:0) x 2 VFOs x 4 bytes
//171: l.LO.LSB stored as "Band8, VFO0"
//172: l.LO.USB stored as "Band8, VFO1"
//256: Last band used
//257: Last VFO used

//PORTS:
//I²C: PB6(SCK), PB9(SDA)

//DDS: PB15:PB12
//LCD: PA3:PA0
//Band relays: A10:A12

//Inputs:

//ADC
//Keys PA4 (analog)
//VDD: PA5 (analog)
//MTR: PA6 (analog)
//TMP: PA7 (analog)

//TX/RX indicator
//PB3

#include "stm32f4xx.h"
#include <stdlib.h>
#include <math.h>

//Radio defines
//Modes and Bands
#define MAXMODES 2
#define MAXBANDS 8

//SPI AD9951 defines
#define DDS_GPIO GPIOB
#define DDS_IO_UD   12   //yellow
#define DDS_SDIO    13   //white
#define DDS_SCLK    14   //blue
#define DDS_RESET   15   //gray

//24C65
#define EEPROM_ADR 0xA0
#define EEPROMSIZE 8192

//Defines for Si5351
#define SI5351_ADR 0xC0     //Check individual module for correct address setting. IDs may vary!
#define FXTAL          25000000 //Hz
#define PLLRATIO       32       //FXTAL * PLLRATIO = f.VCO

//Set of Si5351A relevant register addresses
#define CLK_ENABLE_CONTROL          3
#define PLLX_SRC				   15
#define CLK0_CONTROL               16 
#define CLK1_CONTROL               17
#define CLK2_CONTROL               18
#define SYNTH_PLL_A                26
#define SYNTH_PLL_B                34
#define SYNTH_MS_0                 42
#define SYNTH_MS_1                 50
#define SYNTH_MS_2                 58
#define SPREAD_SPECTRUM_PARAMETERS 149
#define PLL_RESET                  177
#define XTAL_LOAD_CAP              183

//SPI ST7735 defines
#define LCD_GPIO GPIOA
#define CLK    0 //yellow
#define DATA   1 //freen 
#define DC_AO  2 //white
#define RST    3 //gray

//LCD ST7735 contants
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09
#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E
#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD
#define ST7735_PWCTR6  0xFC
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1
//Some sample colors
//USeful website http://www.barth-dev.de/online/rgb565-color-picker/
#define WHITE        0xFFFF
#define BLACK        0x0000
#define GRAY         0x94B2
#define LIGHTGRAY    0xC5D7

#define LIGHTBLUE    0x755C
#define BLUE         0x3C19
#define DARKBLUE     0x0A73
#define DARKBLUE2    0x20AA

#define LIGHTRED     0xFA60 //0xE882 // 0xEB2D //0xDAAB
#define RED          0xF803 //0xB1A7
#define DARKRED      0x80C3

#define LIGHTGREEN   0x27E0 //0x6E84
#define GREEN        0x07EA //0x6505
#define DARKGREEN    0x3B04

#define LIGHTVIOLET  0xAC19
#define LIGHTVIOLET2 0x9BD9
#define VIOLET       0x71B6
#define DARKVIOLET   0x48AF

#define DARKYELLOW   0xB483
#define YELLOW       0xFF00 //0xE746  0xFD40
#define YELLOW2      0xFEC0
#define LIGHTYELLOW  0xF7E0 //0xF752  //0xF7AF

#define LIGHTBROWN   0xF64F
#define BROWN        0x9323
#define DARKBROWN    0x6222


//Font
#define FONTWIDTH 8 
#define FONTHEIGHT 12

#define CHAROFFSET 0x20
const unsigned char xchar[][FONTHEIGHT] ={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x0C,0x1E,0x1E,0x1E,0x0C,0x0C,0x00,0x0C,0x0C,0x00,0x00},	// 0x21
{0x00,0x66,0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x22
{0x00,0x36,0x36,0x7F,0x36,0x36,0x36,0x7F,0x36,0x36,0x00,0x00},	// 0x23
{0x0C,0x0C,0x3E,0x03,0x03,0x1E,0x30,0x30,0x1F,0x0C,0x0C,0x00},	// 0x24
{0x00,0x00,0x00,0x23,0x33,0x18,0x0C,0x06,0x33,0x31,0x00,0x00},	// 0x25
{0x00,0x0E,0x1B,0x1B,0x0E,0x5F,0x7B,0x33,0x3B,0x6E,0x00,0x00},	// 0x26
{0x00,0x0C,0x0C,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x27
{0x00,0x30,0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x30,0x00,0x00},	// 0x28
{0x00,0x06,0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x06,0x00,0x00},	// 0x29
{0x00,0x00,0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00},	// 0x2A
{0x00,0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00},	// 0x2B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x06,0x00},	// 0x2C
{0x00,0x00,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2D
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00},	// 0x2E
{0x00,0x00,0x40,0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00,0x00},	// 0x2F
{0x00,0x3E,0x63,0x63,0x63,0x6B,0x63,0x63,0x63,0x3E,0x00,0x00},	// 0x30
{0x00,0x08,0x0C,0x0F,0x0C,0x0C,0x0C,0x0C,0x0C,0x3F,0x00,0x00},	// 0x31
{0x00,0x1E,0x33,0x33,0x30,0x18,0x0C,0x06,0x33,0x3F,0x00,0x00},	// 0x32
{0x00,0x1E,0x33,0x30,0x30,0x1C,0x30,0x30,0x33,0x1E,0x00,0x00},	// 0x33
{0x00,0x30,0x38,0x3C,0x36,0x33,0x7F,0x30,0x30,0x78,0x00,0x00},	// 0x34
{0x00,0x3F,0x03,0x03,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00,0x00},	// 0x35
{0x00,0x1C,0x06,0x03,0x03,0x1F,0x33,0x33,0x33,0x1E,0x00,0x00},	// 0x36
{0x00,0x7F,0x63,0x63,0x60,0x30,0x18,0x0C,0x0C,0x0C,0x00,0x00},	// 0x37
{0x00,0x1E,0x33,0x33,0x33,0x1E,0x33,0x33,0x33,0x1E,0x00,0x00},	// 0x38
{0x00,0x1E,0x33,0x33,0x33,0x3E,0x18,0x18,0x0C,0x0E,0x00,0x00},	// 0x39
{0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00},	// 0x3A
{0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x1C,0x1C,0x18,0x0C,0x00},	// 0x3B
{0x00,0x30,0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x30,0x00,0x00},	// 0x3C
{0x00,0x00,0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,0x00,0x00},	// 0x3D
{0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00},	// 0x3E
{0x00,0x1E,0x33,0x30,0x18,0x0C,0x0C,0x00,0x0C,0x0C,0x00,0x00},	// 0x3F
{0x00,0x3E,0x63,0x63,0x7B,0x7B,0x7B,0x03,0x03,0x3E,0x00,0x00},	// 0x40
{0x00,0x0C,0x1E,0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00,0x00},	// 0x41
{0x00,0x3F,0x66,0x66,0x66,0x3E,0x66,0x66,0x66,0x3F,0x00,0x00},	// 0x42
{0x00,0x3C,0x66,0x63,0x03,0x03,0x03,0x63,0x66,0x3C,0x00,0x00},	// 0x43
{0x00,0x1F,0x36,0x66,0x66,0x66,0x66,0x66,0x36,0x1F,0x00,0x00},	// 0x44
{0x00,0x7F,0x46,0x06,0x26,0x3E,0x26,0x06,0x46,0x7F,0x00,0x00},	// 0x45
{0x00,0x7F,0x66,0x46,0x26,0x3E,0x26,0x06,0x06,0x0F,0x00,0x00},	// 0x46
{0x00,0x3C,0x66,0x63,0x03,0x03,0x73,0x63,0x66,0x7C,0x00,0x00},	// 0x47
{0x00,0x33,0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x33,0x00,0x00},	// 0x48
{0x00,0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00,0x00},	// 0x49
{0x00,0x78,0x30,0x30,0x30,0x30,0x33,0x33,0x33,0x1E,0x00,0x00},	// 0x4A
{0x00,0x67,0x66,0x36,0x36,0x1E,0x36,0x36,0x66,0x67,0x00,0x00},	// 0x4B
{0x00,0x0F,0x06,0x06,0x06,0x06,0x46,0x66,0x66,0x7F,0x00,0x00},	// 0x4C
{0x00,0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x63,0x63,0x00,0x00},	// 0x4D
{0x00,0x63,0x63,0x67,0x6F,0x7F,0x7B,0x73,0x63,0x63,0x00,0x00},	// 0x4E
{0x00,0x1C,0x36,0x63,0x63,0x63,0x63,0x63,0x36,0x1C,0x00,0x00},	// 0x4F
{0x00,0x3F,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x0F,0x00,0x00},	// 0x50
{0x00,0x1C,0x36,0x63,0x63,0x63,0x73,0x7B,0x3E,0x30,0x78,0x00},	// 0x51
{0x00,0x3F,0x66,0x66,0x66,0x3E,0x36,0x66,0x66,0x67,0x00,0x00},	// 0x52
{0x00,0x1E,0x33,0x33,0x03,0x0E,0x18,0x33,0x33,0x1E,0x00,0x00},	// 0x53
{0x00,0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00,0x00},	// 0x54
{0x00,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x1E,0x00,0x00},	// 0x55
{0x00,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00,0x00},	// 0x56
{0x00,0x63,0x63,0x63,0x63,0x6B,0x6B,0x36,0x36,0x36,0x00,0x00},	// 0x57
{0x00,0x33,0x33,0x33,0x1E,0x0C,0x1E,0x33,0x33,0x33,0x00,0x00},	// 0x58
{0x00,0x33,0x33,0x33,0x33,0x1E,0x0C,0x0C,0x0C,0x1E,0x00,0x00},	// 0x59
{0x00,0x7F,0x73,0x19,0x18,0x0C,0x06,0x46,0x63,0x7F,0x00,0x00},	// 0x5A
{0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00},	// 0x5B
{0x00,0x00,0x01,0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00,0x00},	// 0x5C
{0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00},	// 0x5D
{0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x5E
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00},	// 0x5F
{0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x60
{0x00,0x00,0x00,0x00,0x1E,0x30,0x3E,0x33,0x33,0x6E,0x00,0x00},	// 0x61
{0x00,0x07,0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x3B,0x00,0x00},	// 0x62
{0x00,0x00,0x00,0x00,0x1E,0x33,0x03,0x03,0x33,0x1E,0x00,0x00},	// 0x63
{0x00,0x38,0x30,0x30,0x3E,0x33,0x33,0x33,0x33,0x6E,0x00,0x00},	// 0x64
{0x00,0x00,0x00,0x00,0x1E,0x33,0x3F,0x03,0x33,0x1E,0x00,0x00},	// 0x65
{0x00,0x1C,0x36,0x06,0x06,0x1F,0x06,0x06,0x06,0x0F,0x00,0x00},	// 0x66
{0x00,0x00,0x00,0x00,0x6E,0x33,0x33,0x33,0x3E,0x30,0x33,0x1E},	// 0x67
{0x00,0x07,0x06,0x06,0x36,0x6E,0x66,0x66,0x66,0x67,0x00,0x00},	// 0x68
{0x00,0x18,0x18,0x00,0x1E,0x18,0x18,0x18,0x18,0x7E,0x00,0x00},	// 0x69
{0x00,0x30,0x30,0x00,0x3C,0x30,0x30,0x30,0x30,0x33,0x33,0x1E},	// 0x6A
{0x00,0x07,0x06,0x06,0x66,0x36,0x1E,0x36,0x66,0x67,0x00,0x00},	// 0x6B
{0x00,0x1E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00},	// 0x6C
{0x00,0x00,0x00,0x00,0x3F,0x6B,0x6B,0x6B,0x6B,0x63,0x00,0x00},	// 0x6D
{0x00,0x00,0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x33,0x00,0x00},	// 0x6E
{0x00,0x00,0x00,0x00,0x1E,0x33,0x33,0x33,0x33,0x1E,0x00,0x00},	// 0x6F
{0x00,0x00,0x00,0x00,0x3B,0x66,0x66,0x66,0x66,0x3E,0x06,0x0F},	// 0x70
{0x00,0x00,0x00,0x00,0x6E,0x33,0x33,0x33,0x33,0x3E,0x30,0x78},	// 0x71
{0x00,0x00,0x00,0x00,0x37,0x76,0x6E,0x06,0x06,0x0F,0x00,0x00},	// 0x72
{0x00,0x00,0x00,0x00,0x1E,0x33,0x06,0x18,0x33,0x1E,0x00,0x00},	// 0x73
{0x00,0x00,0x04,0x06,0x3F,0x06,0x06,0x06,0x36,0x1C,0x00,0x00},	// 0x74
{0x00,0x00,0x00,0x00,0x33,0x33,0x33,0x33,0x33,0x6E,0x00,0x00},	// 0x75
{0x00,0x00,0x00,0x00,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00,0x00},	// 0x76
{0x00,0x00,0x00,0x00,0x63,0x63,0x6B,0x6B,0x36,0x36,0x00,0x00},	// 0x77
{0x00,0x00,0x00,0x00,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00,0x00},	// 0x78
{0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x30,0x18,0x0F},	// 0x79
{0x00,0x00,0x00,0x00,0x3F,0x31,0x18,0x06,0x23,0x3F,0x00,0x00},	// 0x7A
{0x00,0x38,0x0C,0x0C,0x06,0x03,0x06,0x0C,0x0C,0x38,0x00,0x00},	// 0x7B
{0x00,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x18,0x00,0x00},	// 0x7C
{0x00,0x07,0x0C,0x0C,0x18,0x30,0x18,0x0C,0x0C,0x07,0x00,0x00},	// 0x7D
{0x00,0xCE,0x5B,0x73,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x7E
{0x00,0x00,0x00,0x08,0x1C,0x36,0x63,0x63,0x7F,0x00,0x00,0x00},	// 0x7F
{0x00,0x1E,0x33,0x33,0x03,0x03,0x03,0x33,0x33,0x1E,0x0C,0x06},	// 0x80
{0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00},	// 0x81 S-Meter bar block
{0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10},	// 0x82 |
{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00},	// 0x83 -
{0x10,0x10,0x10,0x10,0x10,0x10,0xF0,0x00,0x00,0x00,0x00,0x00},	// 0x84 '-
{0x10,0x10,0x10,0x10,0x10,0x10,0x1F,0x00,0x00,0x00,0x00,0x00},	// 0x85 -'
{0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0x10},	// 0x86 ;-
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x10,0x10,0x10,0x10,0x10},	// 0x87 -;
{0x00,0x08,0x14,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x88 ° 
};

//ADC defines
#define KEYS  1 //ADC channel 4 - PA4
#define VDD   2 //ADC channel 5 - PA5
#define MTR   3 //ADC channel 6 - PA6
#define TMP   4 //ADC channel 7 - PA7

////////////////////////////
// Declarations functions //
////////////////////////////
//MISC
int main(void);
static void delay (unsigned int);
void set_band_relay(int);

//ST7735 LCD
void lcd_reset(void);                                    //Reset LCD
void lcd_write_command(int);                             //Send a command to LCD
void lcd_write_data(int);                                //Send data to LCD
void lcd_setwindow(int, int, int, int);                  //Define output window on LCD
void lcd_setpixel(int, int, unsigned int);               //Set 1 Pixel
void lcd_cls(unsigned int);                              //Clear LCD
unsigned int lcd_16bit_color(int, int, int);             //Define color value (int) from red, green and blue
void lcd_putchar(int, int, unsigned char, unsigned int, unsigned int, int, int); //Write one char to LCD (double size, variable height)
void lcd_putstring(int, int, char*, unsigned int, unsigned int, int, int);       //Write \0 terminated string to LCD (double size, variable height)
int lcd_putnumber(int, int, long, int, int, int, int, int);                     //Write a number (int or long) to LCD (double size, variable height)
void show_msg(char*, int);
void show_txrx(void);

//STRING FUNCTIONS
int int2asc(long, int, char*, int);                                     //Convert an int or long number to a string
int strlen(char *);                                                     //Calculate length of string 

//Radio display functions
void show_frequency1(long, int);
void show_frequency2(long);
void show_band(int, int);
void show_sideband(int, int);
void show_vfo(int, int, int);
void show_voltage(int);
void show_pa_temp(int);
void show_msg(char*);
int calc_xpos(int);
int calc_ypos(int);
void draw_hor_line(int, int, int, int);
void draw_vert_line(int, int, int, int);
void show_meter(int);
void draw_meter_bar(int, int, int);
void draw_meter_scale(int);

//DDS
void set_frequency(unsigned long);
void spi_send_bit(int);

//ADC
int get_adc(int);
int get_keys(void);
int get_pa_temp(void);
int get_vdd(void);
int get_sval(void);
int get_txpwr(void);
int get_txrx(void);


//I²C
void i2c_start(void);
void i2c_stop(void); 
void i2c_write_byte1(uint8_t, uint8_t, int);
void i2c_write_byte2(uint8_t*, uint8_t, int); 
int16_t i2c_read(uint8_t); 

//Si5351
void si5351_start(void);
void si5351_set_freq(int, long);
int set_lo(int);

//Interrupt handlers
extern "C" void EXTI0_IRQHandler(void);
extern "C" void TIM2_IRQHandler(void);

//EEPROM
void eeprom_write(uint16_t, uint8_t);
uint8_t eeprom_read(uint16_t);
int is_freq_ok(long, int);
void save_all_vfos(void);
void load_all_vfos(void);
void eeprom_store_frequency(int, int, long);
long eeprom_load_frequency(int, int);

//Variables
//LCD
unsigned int backcolor = DARKBLUE2;

//VFO data & frequencies
int cur_vfo;
int cur_band;
int sideband;
long f_lo[2];
long f_vfo[MAXBANDS][2];
long f_vfo0[MAXBANDS][2] = {{ 1888000,  1961000},	
	                        { 3650000,  3650000}, 
	                        { 7120000,  7120000}, 
	                        {14200000, 14280000}, 
	                        {18080000, 18150000}, 
	                        {21290000, 21390000},
	                        {24910000, 24912000},
	                        {28500000, 28590000}};
int pref_sideband[] = {0, 0, 0, 1, 1, 1, 1, 1}; //Preferred sideband for each ham band
long f_cntr[] =  {1840000, 3650000, 7120000, 14180000, 18100000, 21290000, 24931000, 28500000};  //Center freq
long band_f0[] = {1810000, 3500000, 7000000, 14000000, 18065000, 21000000, 24890000, 28000000};  //Band start
long band_f1[] = {2000000, 3800000, 7200000, 14350000, 18165000, 21465000, 24990000, 29700000};  //Band end

#define INTERFREQUENCY 10000000

//Tuning & seconds counting
int tuning = 0;
long pulses = 0;
long runsecs = 0;
long runsecs_msg = 0;
long runsecs_smax = 0;

//S-Meter
#define METERY 86 //Vertical position for S-Meterbar
int smax = 0;
int sv_old = 0;

/////////////////////////////
 //  Cheap and dirty delay  //
/////////////////////////////
static void delay (unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  /////////////////////////////
 //     INT Handlers        //
/////////////////////////////
//EXTI0
extern "C" void EXTI0_IRQHandler(void)
{
    uint16_t state; 
       
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0))
    {   
		GPIOC->ODR ^= (1 << 13); //Toggle LED
        state = GPIOB->IDR & 0x03; //Read pins
        if(state & 1)
        {
            if(state & 2)
            {
                tuning = 1;
                pulses++;
            }
            else
            {
                tuning = -1;
                pulses++;
            }
        }

        //Clear bit
        EXTI->PR = (1 << 0);
    }
}

//TIM2
extern "C" void TIM2_IRQHandler(void)
{	
	if(TIM2->SR & TIM_SR_UIF)       
    {
		pulses = 0;
		runsecs++;
    }
    TIM2->SR = 0x00;  //Reset status register  
}

  ///////////////////////
 //       L C D       //
///////////////////////    
//Perform hardware reset to LCD
void lcd_reset(void)
{
	LCD_GPIO->ODR |= (1 << RST);  
	delay(100);
	LCD_GPIO->ODR &= ~((1 << RST));  
	delay(100);
	LCD_GPIO->ODR |= (1 << RST);  
	delay(100);
}	

//Write command to LCD
void lcd_write_command(int cmd)
{
	int t1;
	
	LCD_GPIO->ODR &= ~(1 << DC_AO);  //Command
    	
	for(t1 = 7; t1 >= 0; t1--)
    {
	    LCD_GPIO->ODR &= ~(1 << CLK);  //SCL=0	
	    if(cmd & (1 << t1))
	    {
		    LCD_GPIO->ODR |= (1 << DATA);
	    }
	    else
	    {
		    LCD_GPIO->ODR &= ~(1 << DATA); 
	    }
	    LCD_GPIO->ODR |= (1 << CLK);  //SCL=1		
	}	
}	

//Write data to LCD
void lcd_write_data(int dvalue)
{
	int t1;
	
	LCD_GPIO->ODR |= (1 << DC_AO);     //Data
	for(t1 = 7; t1 >= 0; t1--)
    {
	    LCD_GPIO->ODR &= ~(1 << CLK);  //SCL=0	
	    if(dvalue & (1 << t1))
	    {
		    LCD_GPIO->ODR |= (1 << DATA);
	    }
	    else
	    {
		    LCD_GPIO->ODR &= ~(1 << DATA); 
	    }
	    LCD_GPIO->ODR |= (1 << CLK);  //SCL=1		
	}	
}	

//Init LCD to vertical alignement and 16-bit color mode
void lcd_init(void)
{

	lcd_write_command(ST7735_SWRESET); // software reset
	delay(5);

	lcd_write_command(ST7735_SLPOUT);  // out of sleep mode
	delay(5);

	lcd_write_command(ST7735_COLMOD);  // set color mode
	lcd_write_data(0x05);              // 16-bit color
	delay(10);

	lcd_write_command(ST7735_FRMCTR1); // frame rate control
	lcd_write_data(0x00);              // fastest refresh
	lcd_write_data(0x06);              // 6 lines front porch
	lcd_write_data(0x03);              // 3 lines backporch
	delay(1);

	lcd_write_command(ST7735_MADCTL);  // memory access control (directions)
	lcd_write_data(0xC8);              // row address/col address, bottom to top refresh

	lcd_write_command(ST7735_DISSET5); // display settings #5
	lcd_write_data(0x15);              // 1 clock cycle nonoverlap, 2 cycle gate rise, 3 cycle oscil. equalize
	lcd_write_data(0x02);              // fix on VTL

	lcd_write_command(ST7735_INVCTR);  // display inversion control
	lcd_write_data(0x0);               // line inversion

	lcd_write_command(ST7735_GMCTRP1);
	lcd_write_data(0x09);
	lcd_write_data(0x16);
	lcd_write_data(0x09);
	lcd_write_data(0x20);
	lcd_write_data(0x21);
	lcd_write_data(0x1B);
	lcd_write_data(0x13);
	lcd_write_data(0x19);
	lcd_write_data(0x17);
	lcd_write_data(0x15);
	lcd_write_data(0x1E);
	lcd_write_data(0x2B);
	lcd_write_data(0x04);
	lcd_write_data(0x05);
	lcd_write_data(0x02);
	lcd_write_data(0x0E);

	lcd_write_command(ST7735_GMCTRN1);
	lcd_write_data(0x0B);
	lcd_write_data(0x14);
	lcd_write_data(0x08);
	lcd_write_data(0x1E);
	lcd_write_data(0x22);
	lcd_write_data(0x1D);
	lcd_write_data(0x18);
	lcd_write_data(0x1E);
	lcd_write_data(0x1B);
	lcd_write_data(0x1A);
	lcd_write_data(0x24);
	lcd_write_data(0x2B);
	lcd_write_data(0x06);
	lcd_write_data(0x06);
	lcd_write_data(0x02);
	lcd_write_data(0x0F);
	delay(10);
	
	lcd_write_command(ST7735_NORON);   // Normal display on
	delay(10);

	lcd_write_command(ST7735_DISPON);  //Display ON
}	

//Define window area for next graphic operation
void lcd_setwindow(int x0, int y0, int x1, int y1)
{
	lcd_write_command(ST7735_CASET);   //Coloumn address set
	lcd_write_data(0x00);
	lcd_write_data(x0);          
	lcd_write_data(0x00);
	lcd_write_data(x1);          

	lcd_write_command(ST7735_RASET);   //Row address set
	lcd_write_data(0x00);
	lcd_write_data(y0);        
	lcd_write_data(0x00);
	lcd_write_data(y1);        
}

//Set a pixel (Not used, just for academic purposes!)
void lcd_setpixel(int x, int y, unsigned int color)
{
	lcd_setwindow(x, y, x, y);
	lcd_write_command(ST7735_RAMWR);		// RAM access set
	lcd_write_data(color >> 8);
	lcd_write_data(color);
}

//Clear full LCD with background color
void lcd_cls0(unsigned int bgcolor)
{
	int t1;
	lcd_setwindow(0, 0, 132, 132);
	lcd_write_command(ST7735_RAMWR);		// RAM access set
		
    for(t1 = 0; t1 <= 17424; t1++)
    {
		lcd_write_data(bgcolor >> 8);
	    lcd_write_data(bgcolor);
	}    
}	

//Clear part of LCD with background color
void lcd_cls1(int x0, int y0, int x1, int y1, unsigned int bgcolor)
{
	int t1, sz = (x1 - x0) * (y1 - y0);
	lcd_setwindow(x0, y0, x1, y1);
	lcd_write_command(ST7735_RAMWR);		// RAM access set
		
    for(t1 = 0; t1 <= sz; t1++)
    {
		lcd_write_data(bgcolor >> 8);
	    lcd_write_data(bgcolor);
	}    
}	

//Print one character to given coordinates to the screen
//sx and sy define "stretch factor"
void lcd_putchar(int x0, int y0, unsigned char ch0, unsigned int fcol, unsigned int bcol, int sx, int sy)
{
	int x, y, t1, t2;
	unsigned char ch;
	
    lcd_setwindow(x0 + 2, y0 + 2, x0 + FONTWIDTH * sx + 1, y0 + FONTHEIGHT * sy);
	lcd_write_command(ST7735_RAMWR);
	
	for(y = 0; y < FONTHEIGHT - 1; y++)
	{
		ch = xchar[ch0 - CHAROFFSET][y]; 
	    for(t1 = 0; t1 < sy; t1++)
	    {
	        for(x = 0; x < FONTWIDTH; x++)
	        {
		        if((1 << x) & ch)
		        {
					for(t2 = 0; t2 < sx; t2++)
					{
			            lcd_write_data(fcol >> 8);
			            lcd_write_data(fcol);
			        }    
			    }
	   	        else	
		        {
					for(t2 = 0; t2 < sx; t2++)
					{
			            lcd_write_data(bcol >> 8);
			            lcd_write_data(bcol);
			        }    
			    }   
		    }
	    }	
	}
}	

//Print one \0 terminated string to given coordinates to the screen
//xf and yf define "stretch factor"
void lcd_putstring(int x0, int y0, char *s, unsigned int fcol, unsigned int bcol, int xf, int yf)
{
	int x = 0;
	
	while(*s)
	{
		lcd_putchar(x + x0, y0, *(s++), fcol, bcol, xf, yf);
		x += (FONTWIDTH * xf);
	}	
}


//Print a number
//xf and yf define "stretch factor"
int lcd_putnumber(int col, int row, long num, int dec, int fcolor, int bcolor, int xf, int yf)
{
    char *s = (char*) malloc(16);
    int slen = 0;
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(col, row, s, fcolor, bcolor, xf, yf);
	    slen = strlen(s);
	    free(s);
	}	
	return slen;
}

//////////////////////
// STRING FUNCTIONS //
//////////////////////
//INT 2 ASC: Put a number to the screen (with decimal separator if needed)
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < buflen; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//STRLEN
int strlen(char *s)
{
   int t1 = 0;

   while(*(s + t1++));

   return (t1 - 1);
}

//////////////////////////////////
//
//  RADIO DISPLAY FUNCTIONS
//
//////////////////////////////////
//FREQUENCY
void show_frequency1(long f, int csize)
{
	int x;
	int y = calc_ypos(3);
	int fcolor;

    fcolor = WHITE;
	
	if(f < 10000000)
	{
		x = 128 - FONTWIDTH * 12 - 5;
	}
	else
	{
		x = 128 - FONTWIDTH * 14 - 5;
	}	
	
	if(f == 0)
	{
	    lcd_putstring(0, y, (char*)"       ", backcolor, backcolor, csize, csize);
	}
	else
	{
		if(csize == 1)
		{
		    lcd_putnumber(x, y, f, 3, fcolor, backcolor, csize, csize);
		}    
		else
		{
		    lcd_putnumber(x, y, f / 100, 1, fcolor, backcolor, csize, csize);
		}
	}	    

}

//(alternative) FREQUENCY SMALL
void show_frequency2(long f)
{
	int xpos, ypos = calc_ypos(2);
	
	if(f < 10000000)
	{
		xpos = calc_xpos(10);
	}
	else
	{
		xpos = calc_xpos(9);
	}	
	
	lcd_putstring(9 * FONTWIDTH, ypos, (char*)"       ", WHITE, backcolor, 1, 1);
	lcd_putnumber(xpos, ypos, f / 100, 1, WHITE, backcolor, 1, 1);
}

//BAND
void show_band(int band, int invert)
{
	char *band_str[MAXBANDS] = {(char*)"160m", (char*)"80m ", (char*)"40m ", (char*)"20m ", (char*)"17m ", (char*)"15m ", (char*)"12m ", (char*)"10m "};
	int xpos = calc_xpos(0), ypos = calc_ypos(0);	  
	int forecolor = WHITE;
	
	switch(band)
	{
		case 0: forecolor = LIGHTBLUE;
		        break;
		case 1: forecolor = LIGHTBROWN;
		        break;
		case 2: forecolor = LIGHTGREEN;
		        break;
		case 3: forecolor = LIGHTGRAY;
		        break;
		case 4: forecolor = LIGHTVIOLET2;
		        break;
		case 5: forecolor = YELLOW;
		        break;
		case 6: forecolor = LIGHTYELLOW;
		        break;
		case 7: forecolor = WHITE;
		        break;
	}	        
	
	if(invert)
	{	
	     lcd_putstring(xpos, ypos, band_str[band], backcolor, forecolor, 1, 1);
	}
	else     
	{	
	     lcd_putstring(xpos, ypos, band_str[band], forecolor, backcolor, 1, 1);
	}
}

//SIDEBAND
void show_sideband(int sb, int invert)
{
	int xpos = calc_xpos(7), ypos = calc_ypos(0);
	int forecolor;
	char *sidebandstr[MAXMODES + 1] = {(char*)"LSB", (char*)"USB"};
	
	if(!sideband)
	{
		forecolor = LIGHTRED;
	}
	else
	{
		forecolor = LIGHTBLUE;
	}	
	
	if(invert)
	{
		//Write string to position
	    lcd_putstring(xpos, ypos, sidebandstr[sb], backcolor, forecolor, 1, 1);
	}
	else
	{
		//Write string to position
	    lcd_putstring(xpos, ypos, sidebandstr[sb], forecolor, backcolor, 1, 1);
	}
	   
}

//VFO
void show_vfo(int cvfo, int cband, int invert)
{
	int xpos = calc_xpos(12), ypos = calc_ypos(0);
	int forecolor;
	char *vfostr[] = {(char*)"VFOA", (char*)"VFOB"};
	
	if(!cvfo)
	{
		forecolor = WHITE;
	}
	else
	{
		forecolor = YELLOW;
	}	
	//Show frequency of other VFO in d
	if(!cvfo)	
	{
		show_frequency2(f_vfo[cband][1]);
	}
	else	
	{
		show_frequency2(f_vfo[cband][0]);
	}
	
	//Write string to position
	if(!invert)
	{
	    lcd_putstring(xpos, ypos, vfostr[cvfo], forecolor, backcolor, 1, 1);
	}   
	else
	{
	    lcd_putstring(xpos, ypos, vfostr[cvfo], DARKBLUE, forecolor, 1, 1);
	}   
}

//VDD
void show_voltage(int v1)
{
    char *buffer;
	int t1, p;
	int xpos = calc_xpos(0), ypos = calc_ypos(1);
	int fcolor;
		
	buffer= (char*)malloc(0x10);
	//Init buffer string
	for(t1 = 0; t1 < 0x10; t1++)
	{
	    *(buffer + t1) = 0;
	}
    
    p = int2asc(v1, 1, buffer, 6) * FONTWIDTH + xpos;
    
    if(v1 < 10)
    {
		fcolor = RED;
	}
	
	if(v1 >= 10 && v1 < 11)
    {
		fcolor = LIGHTRED;
	}	
	
	if(v1 >= 11 && v1 < 13)
    {
		fcolor = GREEN;
	}	
	
	if(v1 >= 13)
    {
		fcolor = LIGHTGREEN;
	}	
	
    lcd_putstring(xpos, ypos, buffer, fcolor, backcolor, 1, 1);
	lcd_putstring(p, ypos, (char*)"V ", fcolor, backcolor, 1, 1);
	free(buffer);
}

//PA TEMP
void show_pa_temp(int tmp)
{
	int xpos = calc_xpos(12);
	int ypos = calc_ypos(1);
	int fcolor = LIGHTGREEN;
	
	if(tmp > 40)
	{
		fcolor = LIGHTYELLOW;
	}	
	
	if(tmp > 60)
	{
		fcolor = LIGHTRED;
	}	
	
	xpos = calc_xpos(12 + lcd_putnumber(xpos, ypos, tmp, -1, fcolor, backcolor, 1, 1));
	lcd_putchar(xpos, ypos, 0x88, fcolor, backcolor, 1, 1); //°-sign
	xpos += FONTWIDTH;
	lcd_putchar(xpos, ypos, 'C', fcolor, backcolor, 1, 1); //C
}

//Message
void show_msg(char *msg, int fcolor)
{	
	int xpos = calc_xpos(0), ypos = calc_ypos(6);
	lcd_putstring(xpos, ypos, (char*)"                ", fcolor, backcolor, 1, 1);
	lcd_putstring(xpos, ypos, msg, fcolor, backcolor, 1, 1);
}	


//Meter
void show_meter(int sv0)
{
    int sv = sv0;
    
    if(sv > 120)
    {
		sv = 120;
	}	
				    
	//Clear bar graph
	draw_meter_bar(sv, smax, backcolor);
	
	//Draw new bar graph in different colors
	if(sv < 50)
	{
	    draw_meter_bar(0, sv, GREEN);
	}
	if(sv >= 65 && sv < 89)
	{
	    draw_meter_bar(0, 65, GREEN);
	    draw_meter_bar(66, sv, LIGHTYELLOW);
	}

	if(sv >= 89)
	{
	    draw_meter_bar(0, 65, GREEN);
	    draw_meter_bar(66, 89, LIGHTYELLOW);
	    draw_meter_bar(89, sv, LIGHTRED);
	}
    
	if(sv > smax)
	{
		smax = sv;
		runsecs_smax = runsecs;
	}	
	
	sv_old = sv;   
}

//S-Meter bargraph 
void draw_meter_bar(int x0, int x1, int fcol)
{
	int t1;
	
	lcd_setwindow(x0 + 2, METERY, x1 + 2, 94);
	lcd_write_command(ST7735_RAMWR);
	for(t1 = 0; t1 < ((x1 - x0) << 2) + 4; t1++)
	{
		lcd_write_data(fcol >> 8);
	    lcd_write_data(fcol);
	}
}	

//Scale for meter
void draw_meter_scale(int meter_type)
{
	int y = calc_ypos(5); //8 * FONTHEIGHT + 12;
	lcd_putstring(0, y, (char*)"               ", LIGHTYELLOW, backcolor, 1, 1);
	if(!meter_type)
    {
        lcd_putstring(0, y, (char*)"S1 3 5 7 9", LIGHTGREEN, backcolor, 1, 1); //, 1, 1);
        lcd_putstring(65, y, (char*)"+10", LIGHTYELLOW, backcolor, 1, 1); //, 1, 1);
        lcd_putstring(89, y, (char*)"+20dB", LIGHTRED, backcolor, 1, 1); //, 1, 1);
    }
    else
    {
        lcd_putstring(0, y, (char*)"0 2  4  6  8 10W", LIGHTYELLOW, backcolor, 1, 1);
    }
}

//TX/RX status
void show_txrx(void)
{
	int xpos = calc_xpos(7), ypos = calc_ypos(1);
	if(!get_txrx())
	{
		lcd_putstring(xpos, ypos, (char*)"TX", BLACK, LIGHTRED, 1, 1);
	}
	else	
    {
		lcd_putstring(xpos, ypos, (char*)"RX", backcolor, LIGHTGREEN, 1, 1);
	}
}	

//Get X and Y position for row and coloumn in text mode
int calc_xpos(int col)
{
	return col * FONTWIDTH;
}	

int calc_ypos(int row)
{
	return row * (FONTHEIGHT + 5) + 5;
}	

//LINES
void draw_hor_line(int x0, int x1, int y, int color)
{
	int t1;
    for(t1 = x0; t1 < x1; t1++)
	{
		lcd_setpixel(t1, y, color);
	}
}
void draw_vert_line(int x, int y0, int y1, int color)
{
	int t1;
    for(t1 = y0; t1 < y1; t1++)
	{
		lcd_setpixel(x, t1, color);
	}
}

///////////////////////
//    A   D   C      //     
///////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	int adc_val = 0;
	ADC1->SQR3 &= ~(0x3FFFFFFF);
	
	switch(adc_channel)
	{
		case KEYS:  ADC1->SQR3 |= (4 << 0);	//Connect to ADC channel 4 (PA4)
		            break;
		case VDD:   ADC1->SQR3 |= (5 << 0);	//Connect to ADC channel 5 (PA5)
		            break;  
		case MTR:   ADC1->SQR3 |= (6 << 0);	//Connect to ADC channel 6 (PA6)
		            break;              
		case TMP:   ADC1->SQR3 |= (7 << 0);	//Connect to ADC channel 7 (PA7)
		            break;                          
	}
	
    ADC1->CR2 |= (1 << 30);         //Start conversion SWSTART bit 
    while(!(ADC1->SR & (1 << 1)));  //Wait until conversion is complete
    adc_val = ADC1->DR;             //Read value from reg	           
			
	return adc_val;
}	

//Read keys on ADC channel 5
int get_keys(void)
{

    int key_value[] = {370, 735, 1320, 2462, 1863, 3135};
    int t1;
    int adcval0 = get_adc(1);
    long adcval1 = 0;
    int adcvalcnt = 0;
    long secs0 = runsecs;
    
    //lcd_putstring(calc_xpos(0), calc_ypos(3), (char*)"    ", WHITE, backcolor, 1, 1);        
	//lcd_putnumber(calc_xpos(0), calc_ypos(3), adcval0, -1, YELLOW, backcolor, 1, 1);
    
   	if(adcval0 > 4000) //NO key pressed
   	{
		return -1;
	}
		
	while(get_adc(1) <= 4000) //Key pressed
    {
		adcval1 += get_adc(1);
		adcvalcnt++;
	    lcd_putnumber(calc_xpos(0), calc_ypos(5), runsecs - secs0, -1, YELLOW, backcolor, 1, 1);
	    //lcd_putstring(calc_xpos(8), calc_ypos(5), (char*)"    ", backcolor, backcolor, 1, 1);        
	    //lcd_putnumber(calc_xpos(8), calc_ypos(5), get_adc(1), -1, YELLOW, backcolor, 1, 1);
   	}
   	
   	adcval0 = adcval1 / adcvalcnt; //Calc average of adcval
   		
    for(t1 = 0; t1 < 6; t1++)
    {
		if((adcval0 > (key_value[t1] - 100)) && (adcval0 < (key_value[t1] + 100)))
		{
			lcd_putstring(calc_xpos(0), calc_ypos(6), (char*)"KEY:    ", WHITE, backcolor, 1, 1);        
			runsecs_msg = runsecs;
			
			if((runsecs - secs0) < 2)
            {
				
	            lcd_putnumber(calc_xpos(4), calc_ypos(6), t1, -1, YELLOW, backcolor, 1, 1);
				return t1;
				
		    }
		    else
		    {
				lcd_putnumber(calc_xpos(4), calc_ypos(6), t1 + 6, -1, YELLOW, backcolor, 1, 1);
				return t1 + 6;
				
		    }	            
		}    
    }
    
    return -1;
}

//Measure voltage
int get_vdd(void)
{
    double v1;
    v1 = (double) get_adc(VDD) * 0.088623046875; //3.3 / 1024 * 11 / 1 * 10;
	return (int) v1;	

}

//Get adc value for S-meter
int get_sval(void)
{
    int adcval =  get_adc(MTR) >> 4;
	return adcval;	
}

//Get adc value for PWR-meter
int get_txpwr(void)
{
    int adcval =  get_adc(MTR) >> 4;
	return adcval;	
}

//KTY81-210	analog read
int get_pa_temp(void)
{
	int adc = get_adc(TMP);
	double ux = (double) (3.3 * adc) / 4096;
    double rx = 1000 / (3.3 / ux - 1);  //1k to KTY81-210 voltage divider
	double temp = (rx - 1630) / 17.62;  //slope and y0 for KTY81-210
	
	return (int) temp;
}	

int get_txrx(void)
{
    int pin_input = ~GPIOB->IDR; //"0" means "pressed"!
	if(pin_input & (1 << 3))
	{
        return 1;
    }	
    else
	{
        return 0;
    }	    
}	

  /////////////////
 //  SPI  DDS   // 
/////////////////
void spi_send_byte(unsigned int sbyte)
{
    int t1, x = (1 << 7);
	
	for(t1 = 0; t1 < 8; t1++)
	{
	    DDS_GPIO->ODR &= ~(1 << DDS_SCLK);  //DDS_SCLK lo
    	
        //Bit set or erase
	    if(sbyte & x)
	    {
		    DDS_GPIO->ODR |= (1 << DDS_SDIO);  
	    }
	    else
	    {
		    DDS_GPIO->ODR &= ~(1 << DDS_SDIO);  
	    }
	
        DDS_GPIO->ODR |= (1 << DDS_SCLK); //DDS_SCLK hi
		x >>= 1;
	}	
}

//Set frequency for AD9951 DDS
void set_frequency(unsigned long frequency)
{
    //unsigned long interfreq = 10E06; //Interfrequency of radio in Hz
    unsigned long f;
    unsigned long fword;
    int t1, shiftbyte = 24, resultbyte;
    unsigned long comparebyte = 0xFF000000;
	
	f = frequency; //Offset because of inaccuracy of crystal oscillator
	
    //Calculate frequency word
    //2³² / fClk = ----
    
    //Clock rate =  75MHz
    //fword = (unsigned long) f * 57.266230613;
    
    //Clock rate =  100MHz
    //fword = (unsigned long) f * 42.94967296;
        
    //Clock rate =  110MHz
    //fword = (unsigned long) f * 39.045157236;
    
    //Clock rate =  125MHz
    //fword = (unsigned long) f * 34.358675; 
        	
	//Clock rate =  200MHz
    //fword = (unsigned long) f * 21.47478;  //..36448
    
    //Clock rate =  300MHz
    //fword = (unsigned long) f * 14.316557653;
    
    //Clock rate =  400MHz
    fword = (unsigned long) (f + INTERFREQUENCY) * 10.73741824;
		
	
    //Start transfer to DDS
    DDS_GPIO->ODR &= ~(1 << DDS_IO_UD); //DDS_IO_UD lo
    
	//Send instruction bit to set fequency by frequency tuning word
	spi_send_byte(0x04);	
	
    //Calculate and transfer the 4 bytes of the tuning word to DDS
    //Start with msb
    for(t1 = 0; t1 < 4; t1++)
    {
        resultbyte = (fword & comparebyte) >> shiftbyte;
        comparebyte >>= 8;
        shiftbyte -= 8;       
        spi_send_byte(resultbyte);	
    }    
	
	//End transfer sequence
    DDS_GPIO->ODR |= (1 << DDS_IO_UD); //DDS_IO_UD hi 
}

  //////////////////////
 //   I2C commands   //
//////////////////////
void i2c_start(void) 
{
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void i2c_stop(void) 
{
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}

void i2c_write_byte1(uint8_t regaddr, uint8_t data, int i2c_adr) 
{
    //Start signal
    i2c_start();

    //Send chipaddr to be targeted
    I2C1->DR = i2c_adr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)); //Wait until transfer done
    //Perform one dummy read to clear register flags etc.
    (void)I2C1->SR2; //Clear addr reg

    //Send operation type/register 
    I2C1->DR = regaddr;
    while(!(I2C1->SR1 & I2C_SR1_BTF)); //Wait until transfer done

    //Send data
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_BTF));  //Wait

    //Send stop signal
    i2c_stop();
}

//Write multiple number of bytes (>1)
void i2c_write_byte2(uint8_t *data, uint8_t n, int i2c_adr) 
{
	int t1 = 0;
	    
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    for(t1 = 0; t1 < n; t1++)
    {
		//Send data
        I2C1->DR = data[t1];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }    
    
    //Send stop signal
    i2c_stop();
}

int16_t i2c_read(uint8_t regaddr, int i2c_adr) 
{
    int16_t reg;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = i2c_adr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}


int16_t i2c_read2(uint16_t regaddr, int i2c_adr) 
{
    int16_t reg;
    int16_t r_msb, r_lsb;
    
    r_msb = (regaddr & 0xFF00) >> 8;
    r_lsb = regaddr & 0x00FF;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register MSB
    I2C1->DR = r_msb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send operation type/register LSB
    I2C1->DR = r_lsb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = i2c_adr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}

  //////////////////////
 // Si5351A commands //
//////////////////////
//Set PLLA (VCO) to internal clock rate of 900 MHz
//In this example PLLB is not used
//Equation fVCO = fXTAL * (a+b/c) => see AN619 p.3
void si5351_start(void)
{
  unsigned long a, b, c;
  unsigned long p1, p2;
    
  //Init
  i2c_write_byte1(PLLX_SRC, 0, SI5351_ADR);              //Select XTAL as clock source for si5351C
  i2c_write_byte1(SPREAD_SPECTRUM_PARAMETERS, 0, SI5351_ADR); //Spread spectrum diasble (Si5351 A or B only!
  i2c_write_byte1(XTAL_LOAD_CAP, 0xD2, SI5351_ADR);      // Set crystal load capacitor to 10pF (default), 
                                       // for bits 5:0 see also AN619 p. 60
  i2c_write_byte1(CLK_ENABLE_CONTROL, 0x00, SI5351_ADR); // Enable all outputs
  i2c_write_byte1(CLK0_CONTROL, 0x0E, SI5351_ADR);       // Set PLLA to CLK0, 8 mA output
  i2c_write_byte1(CLK1_CONTROL, 0x0E, SI5351_ADR);       // Set PLLA to CLK1, 8 mA output
  i2c_write_byte1(CLK2_CONTROL, 0x0E, SI5351_ADR);       // Set PLLA to CLK2, 8 mA output
  i2c_write_byte1(PLL_RESET, (1 << 5), SI5351_ADR);          // Reset PLLA and PLLB

  //Set VCO of PLLA to 864MHz
  a = PLLRATIO;     // Division factor 864/27 MHz
  b = 0;            // Numerator, sets b/c=0, See AN169 p.3!
  c = 0xFFFFF;      // Max. resolution, but irrelevant in this case as b=0. See AN169 p.3!

  //Formula for splitting up the numbers to register data, see AN619
  p1 = 128 * a + (unsigned long) floor(128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) floor(128 * b / c);
    
  //Write data to registers of PLLA so that VCO is set to 864MHz internal freq
  i2c_write_byte1(SYNTH_PLL_A, 0xFF, SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 1, 0xFF, SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16, SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8, SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 4, (p1 & 0x000000FF), SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16), SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8, SI5351_ADR);
  i2c_write_byte1(SYNTH_PLL_A + 7, (p2 & 0x000000FF), SI5351_ADR);

}

void si5351_set_freq(int synth, long freq)
{
  unsigned long  a, b, c = 0xFFFFF; 
  unsigned long f_xtal = FXTAL;
  double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
  double rm; //remaining
  long p1, p2, p3;
  
  a = (unsigned long) fdiv;
  rm = fdiv - a;  //(equiv. to fractional part b/c)
  b = (unsigned long) (rm * c);
  p1  = 128 * a + (unsigned long) floor(128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) floor(128 * b / c);
  p3 = c;
      
  //Write data to multisynth registers of synth
  i2c_write_byte1(synth + 0, (p3 & 0xFF00) >> 8, SI5351_ADR);  
  i2c_write_byte1(synth + 1, p3 &  0xFF, SI5351_ADR);      
  i2c_write_byte1(synth + 2, (p1 >> 16) & 0x03, SI5351_ADR);
  i2c_write_byte1(synth + 3, (p1 & 0xFF00) >> 8, SI5351_ADR);
  i2c_write_byte1(synth + 4, (p1 & 0xFF), SI5351_ADR);
  i2c_write_byte1(synth + 5, (p3 & 0xF0) | ((p2 >> 16) & 0x0F), SI5351_ADR);
  i2c_write_byte1(synth + 6, (p2 & 0x0000FF00) >> 8, SI5351_ADR);
  i2c_write_byte1(synth + 7, (p2 & 0x000000FF), SI5351_ADR);
}


  ///////////////////////////
 //   EEPROM 24C65        //
///////////////////////////
void eeprom_write(uint16_t mem_address, uint8_t value)
{
   uint8_t data[3]; 
   data[0] = mem_address >> 8;   //Address of byte in 24C65 MSB
   data[1] = mem_address & 0xFF; //                         LSB
   data[2] = value;
   i2c_write_byte2(data, 3, EEPROM_ADR);
   delay(5);
}	

uint8_t eeprom_read(uint16_t mem_address)
{
   uint8_t r = i2c_read2(mem_address, EEPROM_ADR);
   delay(5);
   
   return r;
}	
  ////////////////////////////////////
 //   EEPROM & Mem functions       //
////////////////////////////////////
//Check if freq is in respective band
int is_freq_ok(long f, int cband)
{
	
	if(f >= band_f0[cband] && f <= band_f1[cband])
	{
	    return 1;
	}	
	return 0;		
}

long eeprom_load_frequency(int band, int vfo)
{
    long rf;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = vfo * 4 + band * 8 + 128;
		
    hmsb = eeprom_read(start_adr);
    delay(2);
    hlsb = eeprom_read(start_adr + 1);
    delay(2);
    lmsb = eeprom_read(start_adr + 2);
    delay(2);
    llsb = eeprom_read(start_adr + 3);
    		
    rf = (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
		
	return rf;
}

void eeprom_store_frequency(int band, int vfo, long f)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
    int start_adr = vfo * 4 + band * 8 + 128;
    	
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    eeprom_write(start_adr, hmsb);
    eeprom_write(start_adr + 1, hlsb);
    eeprom_write(start_adr + 2, lmsb);
    eeprom_write(start_adr + 3, llsb);
}

void save_all_vfos(void)
{
	int t0, t1;
	
	for(t0 = 0; t0 < MAXBANDS; t0++)
	{
		for(t1 = 0; t1 < 2; t1++)
		{
			eeprom_store_frequency(t0, t1, f_vfo[t0][t1]);
		}
	}
	eeprom_write(257, cur_vfo); //Last VFO in use
}			

void load_all_vfos(void)
{
	int t0, t1;
	
	for(t0 = 0; t0 < MAXBANDS; t0++)
	{
		for(t1 = 0; t1 < 2; t1++)
		{
			f_vfo[t0][t1] = eeprom_load_frequency(t0, t1);
			if(!is_freq_ok(f_vfo[t0][t1], t0))
			{
				f_vfo[t0][t1] = f_vfo0[t0][t1];
			}	 
		}
	}
}			

///////////////////////
//      LO SET       //     
///////////////////////
int set_lo(int sb)
{
	long f_lo_tmp = f_lo[sb]; //LSB=0, USB=1
	int key;
		
	show_sideband(sb, 1);
	show_frequency1(0, 2);
	show_frequency1(f_lo_tmp, 2);
	
	while(get_keys() != -1);
	
	key = get_keys();
	while((key != 6) && (key != 7))
	{
	    if(tuning)
		{
			f_lo_tmp += pulses * pulses *  tuning;
			si5351_set_freq(SYNTH_MS_0, f_lo_tmp);
			show_frequency1(0, 2);
			show_frequency1(f_lo_tmp, 2);
			tuning = 0;
		}
		key = get_keys();
	}	
	switch(key)
	{
		case 6: si5351_set_freq(SYNTH_MS_0, f_lo[sb]); //Abort operation
		        show_msg((char*)"Aborted.", LIGHTRED);
		        runsecs_msg = runsecs;
		        break;
		case 7: f_lo[sb] = f_lo_tmp;		
		        eeprom_store_frequency(8, sb, f_lo[sb]); //Set new frequency for LO
		        show_msg((char*)"Stored.", LIGHTGREEN);
		        runsecs_msg = runsecs;
		        break;
	}
	return key;
}

///////////////////////
//   BAND RELAY SET  //     
///////////////////////
void set_band_relay(int b)
{
	int t1;
	
	for(t1 = 0; t1 < 3; t1++)
	{
		if((1 << t1) & b)
		{
			GPIOA->ODR |= (1 << (10 + t1));
		}
		else	
		{
			GPIOA->ODR  &= ~(1 << (10 + t1));
		}
	}
    
    //Set LO to preferred sideband of new ham band
    si5351_set_freq(SYNTH_MS_0, f_lo[pref_sideband[b]]);
    show_sideband(pref_sideband[b], 0);
}	


int main(void)
{
	
	int key;
	int t1;
	long runsecs_meas = 0;
	int tx_stat_old = 0;
		
    //GPIOA  power up for DDS (PA15:PA12) and LCD (PA4:PA0)
    RCC->AHB1ENR |= (1 << 0);
    
    //GPIOB power up for rotary encoder (PB1:PB0)
    RCC->AHB1ENR |= (1 << 1);                           
    
    //Read TX/RX status on PB3
    GPIOB->MODER  &= ~(3 << (3 << 1));
    GPIOB->PUPDR  &= ~(3 << (3 << 1));
        
    //GPIOC power up for onboard LED (PC13)
    RCC->AHB1ENR |= (1 << 2);  
    GPIOC->MODER |= (1 << (13 << 1));                         
            
    //A8:A10 as output ports for band relay BCD
    for(t1 = 10; t1 < 13; t1++)
    {
		GPIOA->MODER |= (1 << (t1 << 1));
	}
	
	/*	        
	for(t1 = 8; t1 < 11; t1++)	        
	{
		GPIOA->ODR &= ~(1 << t1);
	}	
		
    //Set PA8 as MCO pin to check master clock frequency
    
    //Turn on GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    RCC->CFGR |= (0b11 << 21);          //MCO1: Microcontroller clock output 1 11: PLL clock selected
    //RCC->CFGR |= (0b100 <<24);          //Divide f.out by 2
    GPIOA->MODER |= (2 << (8 << 1));	//PA8 as AF
    GPIOA->OSPEEDR |= (3 << (8 << 1));	//HiSpeed
    GPIOA->AFR[1] = 0;                  //0b0000   
    */
            
    /////////////////////////////////////////////////
    // Set SystemClock to 167 MHz with 25 MHz HSE  //
    /////////////////////////////////////////////////
    FLASH->ACR |= 0b010;                         //2 wait state for 100 MHz
    RCC->CR |= (1 << 16);                        //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & (1 << 17)) == 0);          //Wait until HSE is ready
    
    //Configuration of PLL
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
                                                //Set PLLM
    RCC->PLLCFGR &= ~0x3F;                      //1st Reset bits
    RCC->PLLCFGR |= 15;                         //2nd define VCO input frequency = PLL input clock frequency (f.HSE) / PLLM with 2 ≤ PLLM ≤ 63 
                                                //-> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                                                                
                                                //Set PLLN: PPLLN defines VCO out frequency
    RCC->PLLCFGR &= ~0x7FC0;                    //1st Reset bits 14:6
    RCC->PLLCFGR |= (200 << 6);                 //2nd define f.VCO.out = f.VCO.in * 210 = 350MHz
     
                                                //Set PLLP: Main PLL (PLL) division factor for main system clock; Reset Bits 17:16
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset bits 17:16
    RCC->PLLCFGR |= (0x00 << 16);               //f.PLL.output.clock = f.VCO.out / 2 = 175MHz
                                                
                                                //Set PLLQ. PLLQ = division factor for USB OTG FS, SDIO and random number generator clocks
    RCC->PLLCFGR &= ~(0b1111 << 24);            //Reset bits 27:24
    RCC->PLLCFGR |= (8 << 24);                  //PLL-Q: f.VCO.out / 8 = 43.75MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL, Bit 24
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready Bit 25
    
                                                //Division of clock signal for bus system
    RCC->CFGR |= (0b1001 << 4)                  //AHB divider:  175MHz / 4 = 43.75MHz
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= 0b10;                          //Switching to PLL clock source
    
    /////////////////////////
    //Rotary Encoder Setup //
    /////////////////////////
    //Set PB0, PB1 as input pins
    RCC->AHB1ENR |= (1 << 1);                           //GPIOB power up
    GPIOB->MODER &= ~((3 << (0 << 1))|(3 << (1 << 1))); //PB0 und PB1 for Input
    GPIOB->PUPDR |= (1 << (0 << 1))|(1 << (1 << 1));    //Pullup PB0 und PB1
    
    RCC->APB2ENR |= (1 << 14); //Enable SYSCFG clock (APB2ENR: bit 14)
    
    SYSCFG->EXTICR[0] |= 0x0001;  //Write 0b01 to map PB0 to EXTI0
    EXTI->RTSR |= 0x01;           //Enable rising edge trigger on EXTI0
    EXTI->IMR |= 0x01;            //Mask EXTI0

    //Initialize interrupt controller
    NVIC_SetPriorityGrouping(3);

    NVIC_SetPriority(EXTI0_IRQn, 1); //Set Priority for each interrupt request Priority level 1
    NVIC_EnableIRQ(EXTI0_IRQn);      //Enable EXT0 IRQ from NVIC
    
    /////////////////////////
    //TIMER2 Setup         //
    /////////////////////////
    //Enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    
    //Timer calculation
    //Timer update frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 10000-1;   //Divide system clock (f=100MHz) by 10000 -> update frequency = 10000/s
    TIM2->ARR = 3500;      //Define overrun

    //Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    TIM2->CR1 |= (1 << 0);           //Enable Timer 2 module (CEN, bit0)
    
    NVIC_SetPriority(TIM2_IRQn, 2); //Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);      //Enable TIM2 IRQ from NVIC
    
    /////////////////////////
    //ADC1 Init            //
    /////////////////////////
    //Port config
    for(t1 = 4; t1 < 8; t1++) //PA7:PA4 to analog mode
    {
        GPIOA->MODER |= (3 << (t1 << 1));           //Set PAt1 to analog mode
    }
        
    //ADC config sequence
    RCC->APB2ENR |= (1 << 8);	                    //Enable ADC1 clock (Bit8) 
    ADC1->CR1 &= ~(1 << 8);			                //SCAN mode disabled (Bit8)
	ADC1->CR1 &= ~(3 << 24);				        //12bit resolution (Bit24,25 0b00)
	ADC1->SQR1 &= ~(0x0F << 20);                    //Set number of conversions projected (L[3:0] 0b0000)
	ADC1->SQR3 &= ~(0x3FFFFFFF);	                //Clears whole 1st 30bits in register
	ADC1->SQR3 |= (4 << 0);			                //First conversion in regular sequence: PB4 as ADC1_4 (-> select ADC channel 4 as first (and only) conversion)
    ADC1->CR2 &= ~(1 << 1);			                //Single conversion ADON=1
	ADC1->CR2 &= ~(1 << 11);			            //Right alignment of data bits  bit12....bit0
	ADC1->SMPR2 |= (7 << 0);	 		            //Sampling rate 480 cycles. 16MHz bus clock for ADC. 1/16MHz = 62.5ns. 480*62.5ns=30us
    ADC1->CR2 |= (1 << 0);                          //Switch on ADC1
    //ADC ready for use
    
    /////////////////////////
    //LCD Setup            //
    /////////////////////////
    //Put pin 0..4 in general purpose output mode
    LCD_GPIO->MODER |= (1 << (DATA << 1));	
    LCD_GPIO->MODER |= (1 << (CLK << 1));	
    LCD_GPIO->MODER |= (1 << (DC_AO << 1));	
    LCD_GPIO->MODER |= (1 << (RST << 1));	
            
    //Start ST7735 LCD
    lcd_reset();
    delay(100);
    lcd_init();
    lcd_cls0(backcolor);
        
    draw_hor_line(0, 129, calc_ypos(0) + FONTHEIGHT + 3, LIGHTBLUE);
    draw_hor_line(0, 129, calc_ypos(1) + FONTHEIGHT + 3, LIGHTBLUE);
    draw_vert_line(50, 0, calc_ypos(1) + FONTHEIGHT + 3, LIGHTBLUE);
    draw_vert_line(90, 0, calc_ypos(1) + FONTHEIGHT + 3, LIGHTBLUE);
    draw_hor_line(0, 129, calc_ypos(4) + FONTHEIGHT - 2, LIGHTBLUE);
    draw_hor_line(0, 129, calc_ypos(5) + FONTHEIGHT + 3, LIGHTBLUE);
    draw_hor_line(0, 129, calc_ypos(6) + FONTHEIGHT + 3, LIGHTBLUE);
    
    //Turn on the GPIOB peripheral for DDS SPI interface
    RCC->AHB1ENR |= (1 << 1);
    
    /////////////////////////
    //DDS Setup            //
    /////////////////////////
    //Put pin AB15:B12 to general purpose output mode
    DDS_GPIO->MODER  |=  (1 << (DDS_SCLK << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_IO_UD << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_SDIO << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_RESET << 1));	
    
    delay(100);
    
	//Reset DDS (AD9951)
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	delay(100);
	DDS_GPIO->ODR &= ~(1 << DDS_RESET);  
    delay(100);
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	
	/////////////////
	// Setup I2C   //
	/////////////////
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Enable I2C clock
    RCC->AHB1ENR |= (1 << 1);          //GPIOB power up
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 option for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     // for PB6
    GPIOB->AFR[1] |= (4 << ((9 - 8) << 2)); // for PB9

    //Reset and clear control register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Enable error interrupt
    I2C1->CR2 |= (I2C_CR2_ITERREN); 

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); //10Mhz peripheral clock
    I2C1->CCR |= (50 << 0);
    //Maximum rise time set
    I2C1->TRISE |= (11 << 0); //TRISE=11ns for 100khz
    
    //Enable I2C
    I2C1->CR1 |= I2C_CR1_PE; 
    
    //Init Si5351
    si5351_start();
    
    /////////// ALL MODULES SETUP FINISHED   ////////
    
	//Load values
    cur_band = eeprom_read(256);    	
    if((cur_band < 0) || (cur_band > 7))
    {
		cur_band = 2;
	}	
	
    cur_vfo = eeprom_read(257);    	
    if((cur_vfo < 0) || (cur_vfo > 1))
    {
		cur_vfo = 0;
	}	
	    
    load_all_vfos();
    
	sideband = pref_sideband[cur_band];
	
	for(t1 = 0; t1 < 2; t1++)
	{
		f_lo[t1] = eeprom_load_frequency(8, t1);
		if((f_lo[t1] < INTERFREQUENCY - 3000) || (f_lo[t1] > INTERFREQUENCY + 3000))
		{
			f_lo[t1] = INTERFREQUENCY + 1500 * (t1 * 2 - 1);
		}	
		si5351_set_freq(SYNTH_MS_0, f_lo[t1]);
	}	
			
	//Display data on screen 
	set_band_relay(cur_band);
	show_band(cur_band, 0);
	show_vfo(cur_vfo, cur_band, 0);
    show_frequency1(f_vfo[cur_band][cur_vfo], 2);
    show_sideband(1, 0);
    show_voltage(get_vdd());
    show_pa_temp(get_pa_temp());
    draw_meter_scale(0);
    show_msg((char*)"DK7IH 8-Band-TRX", LIGHTBLUE);    
        
    for(;;) 
	{
		if(tuning)
		{
			f_vfo[cur_band][cur_vfo] += pulses * pulses *  tuning;
			set_frequency(f_vfo[cur_band][cur_vfo]);
			show_frequency1(f_vfo[cur_band][cur_vfo], 2);
			tuning = 0;
		}		
		        
        key = get_keys();
        
        switch(key)
        {
			case 0: if(cur_band < (MAXBANDS - 1))
			        {
						cur_band++;
						set_frequency(f_vfo[cur_band][cur_vfo]);
						show_frequency1(0, 2);
						show_frequency1(f_vfo[cur_band][cur_vfo], 2);
						show_band(cur_band, 0);
						set_band_relay(cur_band);
						eeprom_write(256, cur_band);
					}
					break;
					
			case 1: sideband = !sideband;
			        show_sideband(sideband, 0);
			        break;
			        
			case 2: cur_vfo = !cur_vfo;
			        eeprom_write(257, cur_vfo);
			        show_vfo(cur_vfo, cur_band, 0);
			        set_frequency(f_vfo[cur_band][cur_vfo]);
					show_frequency1(f_vfo[cur_band][cur_vfo], 2);
					break;        
					
			case 3: if(cur_band > 0)
			        {
						cur_band--;
						set_frequency(f_vfo[cur_band][cur_vfo]);
						show_frequency1(0, 2);
						show_frequency1(f_vfo[cur_band][cur_vfo], 2);
						show_band(cur_band, 0);
						set_band_relay(cur_band);
						eeprom_write(256, cur_band);
					}		
					break;
					
			case 4: save_all_vfos();
			        show_msg((char*)"Saved.", LIGHTGREEN);
			        runsecs_msg = runsecs;
			        break;		
			        
			case 6: set_lo(0);
			        show_frequency1(f_vfo[cur_band][cur_vfo], 2);
			        show_sideband(sideband, 0);
			        break;    
			          
			case 7: set_lo(1);
			        show_frequency1(f_vfo[cur_band][cur_vfo], 2); 
			        show_sideband(sideband, 0);
			        break;        
		}	
		
		show_meter(get_sval());
		
		//Clear message line
		if((runsecs > runsecs_msg + 3) && runsecs_msg)
		{
			show_msg((char*)"DK7IH 8-Band-TRX", LIGHTBLUE);    
			runsecs_msg = 0;		
		}	

        if(get_txrx() != tx_stat_old)
        {
            show_txrx();
            tx_stat_old = get_txrx();
        }    
            
					
		//Show VDD, PATMP evry 3 secs
		if(runsecs > runsecs_meas + 3)
		{
			show_pa_temp(get_pa_temp());
		    show_voltage(get_vdd());	
			runsecs_meas = runsecs;		
		}	
	}
	return 0;
}
 
