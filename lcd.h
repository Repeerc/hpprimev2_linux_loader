
#include <stdint.h>

#define UINT32 uint32_t

#define LCD_REG_PA_BASE                    0x4C800000           

typedef struct {
    UINT32 rVIDCON0;             // 0x00
    UINT32 rVIDCON1;             // 0x04
    UINT32 rVIDTCON0;             // 0x08
    UINT32 rVIDTCON1;             // 0x0C
    UINT32 rVIDTCON2;             // 0x10
    UINT32 rWINCON0;           // 0x14
    UINT32 rWINCON1;           // 0x18
    UINT32 rPAD[3];     

    UINT32 rVIDOSD0A;           // 0x28
    UINT32 rVIDOSD0B;              // 0x2C
    UINT32 rVIDOSD0C;            // 0x30
    UINT32 rVIDOSD1A;             // 0x34
    UINT32 rVIDOSD1B;             // 0x38
    UINT32 rVIDOSD1C;             // 0x3C
    
    UINT32 rPAD1[9];              // 0x40 ~ 0x60 // PAD

    UINT32 rVIDW00ADD0B0;             // 0x64
    UINT32 rVIDW00ADD0B1;             // 0x68
    UINT32 rVIDW01ADD0;               // 0x6C

    UINT32 rPAD2[3];              // 0x70 ~ 0x78 // PAD

    UINT32 rVIDW00ADD1B0;             // 0x7C
    UINT32 rVIDW00ADD1B1;             // 0x80
    UINT32 rVIDW01ADD1;               // 0x84

    UINT32 rPAD3[3];              // 0x88 ~ 0x90 // PAD

    UINT32 rVIDW00ADD2B0;             // 0x94
    UINT32 rVIDW00ADD2B1;             // 0x98
    UINT32 rVIDW01ADD2;               // 0x9C

    UINT32 rPAD4[3];              // 0xA0 ~ 0xA8 // PAD
    
    UINT32 rVIDINTCON;                // 0xAC
    UINT32 rW1KEYCON0;                // 0xB0
    UINT32 rW1KEYCON1;                // 0xB4
    UINT32 rW2KEYCON0;                // 0xB8
    UINT32 rW2KEYCON1;                // 0xBC
    UINT32 rW3KEYCON0;                // 0xC0
    UINT32 rW3KEYCON1;                // 0xC4
    UINT32 rW4KEYCON0;                // 0xC8
    UINT32 rW4KEYCON1;                // 0xCC
    UINT32 rWIN0MAP;                  // 0xD0
    UINT32 rWIN1MAP;                  // 0xD4

    UINT32 rPAD5[3];              // 0xD8 ~ 0xE0 // PAD

    UINT32 rWPALCON;                  // 0xE4

    UINT32 rPAD6[18];              // 0xE8 ~ 0x12C // PAD

    UINT32 rSYSIFCON0;                // 0x130
    UINT32 rSYSIFCON1;                // 0x134
    UINT32 rDITHMODE;                 // 0x138

} LCD_REG;


//------------------------------------------------------------------------------
//  Define: LCD_TYPE_XXX
//
//  Enumerates the types of LCD displays available.
//

#define    LCD_TYPE_STN8BPP            (1)
#define    LCD_TYPE_TFT16BPP           (2)

//------------------------------------------------------------------------------
//  Define: LCD_TYPE
//
//  Defines the active LCD type from above choices.
//

#define    LCD_TYPE                    LCD_TYPE_TFT16BPP

//------------------------------------------------------------------------------
//  Define: LCD_MODE_XXX
//
//  Defines the LCD mode.
//

#define    LCD_MODE_STN_1BIT       (1)
#define    LCD_MODE_STN_2BIT       (2)
#define    LCD_MODE_STN_4BIT       (4)
#define    LCD_MODE_CSTN_8BIT      (108)
#define    LCD_MODE_CSTN_12BIT     (112)
#define    LCD_MODE_TFT_1BIT       (201)
#define    LCD_MODE_TFT_2BIT       (202)
#define    LCD_MODE_TFT_4BIT       (204)
#define    LCD_MODE_TFT_8BIT       (208)
#define    LCD_MODE_TFT_16BIT      (216)

//------------------------------------------------------------------------------
//  Define: LCD_SCR_XXX
//
//  Screen size definitions.
//



#define    LCD_MVAL                (13)
#define    LCD_MVAL_USED           (0)

// STN/CSTN timing parameter for LCBHBT161M(NANYA)

#define    LCD_WLH                 (3)
#define    LCD_WDLY                (3)
#define    LCD_LINEBLANK           ((1)&0xff)

// TFT timing parameter for V16C6448AB(PRIME VIEW) 
/*
#define    LCD_VBPD                ((1)&0xff)
#define    LCD_VFPD                ((2)&0xff)
#define    LCD_VSPW                ((1)&0x3f)
#define    LCD_HBPD                ((6)&0x7f)
#define    LCD_HFPD                ((2)&0xff)
#define    LCD_HSPW                ((4)&0xff)
*/

// AT070TN83
#define    LCD_VBPD                ((29)&0xff)
#define    LCD_VFPD                ((13)&0xff)
#define    LCD_VSPW                ((3)&0x3f)
#define    LCD_HBPD                ((40)&0x7f)
#define    LCD_HFPD                ((40)&0xff)
#define    LCD_HSPW                ((48)&0xff)

/*
// Hitachi  7"   (800*480)
#define    LCD_VBPD                ((14)&0xff)
#define    LCD_VFPD                ((6)&0xff)
#define    LCD_VSPW                ((3)&0x3f)
#define    LCD_HBPD                ((37)&0x7f)
#define    LCD_HFPD                ((19)&0xff)
#define    LCD_HSPW                ((29)&0xff)
*/
/*
// Sharp  4.3" (480*272)
#define    LCD_VBPD                ((2)&0xff)
#define    LCD_VFPD                ((2)&0xff)
#define    LCD_VSPW                ((10)&0x3f)
#define    LCD_HBPD                ((2)&0x7f)
#define    LCD_HFPD                ((2)&0xff)
#define    LCD_HSPW                ((41)&0xff)
*/
// TFT Video Main Control 1 Register Status Field (read only)
#define		LCD_LINECNT						(0x3ff <<(16))
#define		LCD_VSTATUS						(0x3 << (13))
#define		LCD_HSTATUS						(0x3 << (11))

//------------------------------------------------------------------------------
//  Define: LCD_CLKVAL_XXX
//
//  Clock values
//

#define     CLKVAL_STN_MONO         (22)    

// 69.14hz @60Mhz,WLH=16clk,WDLY=16clk,LINEBLANK=1*8,VD=4 

#define     CLKVAL_STN_GRAY         (12)    

//124hz @60Mhz,WLH=16clk,WDLY=16clk,LINEBLANK=1*8,VD=4  

#define     CLKVAL_CSTN             (8)     

//135hz @60Mhz,WLH=16clk,WDLY=16clk,LINEBLANK=1*8,VD=8  

#define     CLKVAL_TFT              (6)

// NOTE: 1)SDRAM should have 32-bit bus width. 
//      2)HBPD,HFPD,HSPW should be optimized. 
// 44.6hz @75Mhz
// VSYNC,HSYNC should be inverted
// HBPD=48VCLK,HFPD=16VCLK,HSPW=96VCLK
// VBPD=33HSYNC,VFPD=10HSYNC,VSPW=2HSYNC

#define     M5D(n)                  ((n)&0x1fffff)

