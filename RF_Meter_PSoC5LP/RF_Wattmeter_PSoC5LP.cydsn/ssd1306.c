/*

SSD1306 OLED - I2C driver

Derk Steggewentz, 3/2015 

This is a I2C driver for SSD1306 OLED displays including graphics library.


The graphics library part (gfx_ functions) is based on the Adafruit graphics library.


No license restrictions on my part, do whatever you want as long as you follow the inherited
license requirements (applying to the Adafruit graphics library part).


===== derived license (for graphics library) ======

Copyright (c) 2013 Adafruit Industries. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/


#include <project.h>
#include <stdlib.h>

#include "font.h"
#include "ssd1306.h"


#define DISPLAYWIDTH 128
#define DISPLAYHEIGHT 64
    
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E


// I2C result status
#define TRANSFER_CMPLT    (0x00u)
#define TRANSFER_ERROR    (0xFFu)


static uint32 display_write_buf( uint8* buf, uint16_t size );

void gfx_init( int16_t width, int16_t height );

static uint8 _i2caddr;

// display memory buffer ( === MUST INCLUDE === the preceding I2C 0x40 control byte for the display)
static uint8_t SSD1306_buffer[DISPLAYHEIGHT * DISPLAYWIDTH / 8 + 1] = { 0x40 };
// pointer to actual display memory buffer
static uint8_t* _displaybuf = SSD1306_buffer+1;
static uint16_t _displaybuf_size = sizeof(SSD1306_buffer) - 1;

// see data sheet page 25 for Graphic Display Data RAM organization
// 8 pages, each page a row of DISPLAYWIDTH bytes
// start address of of row: y/8*DISPLAYWIDTH
// x pos in row: == x 
#define GDDRAM_ADDRESS(X,Y) ((_displaybuf)+((Y)/8)*(DISPLAYWIDTH)+(X))

// lower 3 bit of y determine vertical pixel position (pos 0...7) in GDDRAM byte
// (y&0x07) == position of pixel on row (page). LSB is top, MSB bottom
#define GDDRAM_PIXMASK(Y) (1 << ((Y)&0x07))

#define PIXEL_ON(X,Y) (*GDDRAM_ADDRESS(x,y) |= GDDRAM_PIXMASK(y))
#define PIXEL_OFF(X,Y) (*GDDRAM_ADDRESS(x,y) &= ~GDDRAM_PIXMASK(y))
#define PIXEL_TOGGLE(X,Y) (*GDDRAM_ADDRESS(x,y) ^= GDDRAM_PIXMASK(y)) 



// call before first use of other functions
void display_init( uint8 i2caddr ){
    
    _i2caddr = i2caddr;
    gfx_init( DISPLAYWIDTH, DISPLAYHEIGHT );
    
    uint8 cmdbuf[] = {
        0x00,
        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV,
        0x80,
        SSD1306_SETMULTIPLEX,
        0x3f,
        SSD1306_SETDISPLAYOFFSET,
        0x00,
        SSD1306_SETSTARTLINE | 0x0,
        SSD1306_CHARGEPUMP,
        0x14,
        SSD1306_MEMORYMODE,
        0x00,
        SSD1306_SEGREMAP | 0x1,
        SSD1306_COMSCANDEC,
        SSD1306_SETCOMPINS,
        0x12,
        SSD1306_SETCONTRAST,
        0xcf,
        SSD1306_SETPRECHARGE,
        0xf1,
        SSD1306_SETVCOMDETECT,
        0x40,
        SSD1306_DISPLAYALLON_RESUME,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON
    };
    
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
}


// for submitting command sequences:
//  buf[0] must be 0x00
// for submitting bulk data (writing to display RAM):
//  buf[0] must be 0x40
static uint32 display_write_buf( uint8* buf, uint16_t size ){

    uint32 status = TRANSFER_ERROR;
    I2COLED_MasterSendStart(_i2caddr,I2COLED_WRITE_XFER_MODE);
    
    for(int i=0; i<size; i++){  
        status=I2COLED_MasterWriteByte(buf[i]);
    }
    I2COLED_MasterSendStop();
    return status;
}


// used by gfx_ functions. Needs to be implemented by display_
static void display_setPixel( int16_t x, int16_t y, uint16_t color ){
    
    if( (x < 0) || (x >= DISPLAYWIDTH) || (y < 0) || (y >= DISPLAYHEIGHT) )
        return;

    switch( color ){
        case WHITE: 
            PIXEL_ON(x,y);
            break;
        case BLACK:
            PIXEL_OFF(x,y);
            break;
        case INVERSE: 
            PIXEL_TOGGLE(x,y);
            break;
    }
}


void display_clear(void){
    memset( _displaybuf, 0, _displaybuf_size );
    SSD1306_buffer[0] = 0x40; // to be sure its there
}


// contrast: 0 ...255
void display_contrast( uint8_t contrast ){
        
    uint8 cmdbuf[] = {
        0x00,  
        SSD1306_SETCONTRAST,
        contrast
    };
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
}


// invert <> 0 for inverse display, invert == 0 for normal display
void display_invert( uint8_t invert ){

    uint8 cmdbuf[] = {
        0x00,  
        0
    };
    cmdbuf[1] = invert ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY;
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
}


void display_update(void) {
      
    uint8 cmdbuf[] = {
        0x00,
        SSD1306_COLUMNADDR,
        0,                      // start
        DISPLAYWIDTH-1, // end
        SSD1306_PAGEADDR,
        0,                      // start
        7                       // end
    };
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
    display_write_buf( SSD1306_buffer, sizeof(SSD1306_buffer) );
}


// draws horizontal or vertical line
// Note: no check for valid coords, this needs to be done by caller
// should only be called from gfx_hvline which is doing all validity checking
static void display_line( int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color ){

    if( x1 == x2 ){
        // vertical
        uint8_t* pstart = GDDRAM_ADDRESS(x1,y1);
        uint8_t* pend = GDDRAM_ADDRESS(x2,y2);       
        uint8_t* ptr = pstart;             
        
        while( ptr <= pend ){
            
            uint8_t mask;
            if( ptr == pstart ){
                // top
                uint8_t lbit = y1 % 8;
                // bottom (line can be very short, all inside this one byte)
                uint8_t ubit = lbit + y2 - y1;
                if( ubit >= 7 )
                    ubit = 7;
                mask = ((1 << (ubit-lbit+1)) - 1) << lbit;    
            }else if( ptr == pend ){
                // top is always bit 0, that makes it easy
                // bottom
                mask = (1 << (y2 % 8)) - 1;    
            }

            if( ptr == pstart || ptr == pend ){
                switch( color ){
                    case WHITE:     *ptr |= mask; break;
                    case BLACK:     *ptr &= ~mask; break;
                    case INVERSE:   *ptr ^= mask; break;
                };  
            }else{
                switch( color ){
                    case WHITE:     *ptr = 0xff; break;
                    case BLACK:     *ptr = 0x00; break;
                    case INVERSE:   *ptr ^= 0xff; break;
                };  
            }
            
            ptr += DISPLAYWIDTH;
        }
    }else{
        // horizontal
        uint8_t* pstart = GDDRAM_ADDRESS(x1,y1);
        uint8_t* pend = pstart + x2 - x1;
        uint8_t pixmask = GDDRAM_PIXMASK(y1);    

        uint8_t* ptr = pstart;
        while( ptr <= pend ){
            switch( color ){
                case WHITE:     *ptr |= pixmask; break;
                case BLACK:     *ptr &= ~pixmask; break;
                case INVERSE:   *ptr ^= pixmask; break;
            };
            ptr++;
        }
    }
}



void display_stopscroll(void){

    uint8 cmdbuf[] = {
        0x00,
        SSD1306_DEACTIVATE_SCROLL
    };
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
}

void display_scroll( SCROLL_AREA start, SCROLL_AREA end, SCROLL_DIR dir, SCROLL_SPEED speed ){
   
    uint8 cmdbuf[] = {
        0x00,
        dir,                    // 0x26 or 0x2a
        0x00,                   // dummy byte
        start,                  // start page
        speed,                  // scroll step interval in terms of frame frequency 
        end,                    // end page
        0x00,                   // dummy byte
        0xFF,                   // dummy byte
        SSD1306_ACTIVATE_SCROLL // 0x2F
    };
    display_write_buf( cmdbuf, sizeof(cmdbuf) ); 
}



// ============================================================
// graphics library stuff

int16_t WIDTH, HEIGHT; // This is the 'raw' display w/h - never changes
static int16_t _width, _height; // Display w/h as modified by current rotation
static int16_t cursor_x, cursor_y;
static uint16_t textcolor, textbgcolor;
static uint8_t textsize;
uint8_t rotation;
uint8_t wrap; // If set, 'wrap' text at right edge of display



void gfx_init( int16_t width, int16_t height ){
    WIDTH = width;
    HEIGHT = height;
    _width = WIDTH;
    _height = HEIGHT;
    
    rotation = 0;
    cursor_y = cursor_x = 0;
    textsize = 1;
    textcolor = textbgcolor = 0xFFFF;
    wrap = 1;
}

// Return the size of the display (per current rotation)
int16_t gfx_width(void){
    return _width;
}

int16_t gfx_height(void){
    return _height;
}

uint8_t gfx_rotation(void){
    return rotation;
}

void gfx_setCursor( int16_t x, int16_t y ){
    cursor_x = x;
    cursor_y = y;
}

void gfx_setTextSize( uint8_t size ){
    textsize = (size > 0) ? size : 1;
}

void gfx_setTextColor( uint16_t color ){
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    textcolor = textbgcolor = color;
}

void gfx_setTextBg( uint16_t color ){
    textbgcolor = color;
}

void gfx_setTextWrap( uint8 w ){
    wrap = w;
}

void gfx_setRotation( uint8_t x ){
    
    rotation = (x & 3);
    switch( rotation ){
        case 0:
        case 2:
            _width = WIDTH;
            _height = HEIGHT;
            break;
        case 1:
        case 3:
            _width = HEIGHT;
            _height = WIDTH;
        break;
    }
}

static void gfx_rotation_adjust( int16_t* px, int16_t* py ){

    int16_t y0 = *py;
    
    switch( rotation ){
        case 1:
            *py = *px;
            *px = WIDTH - y0 - 1;
            break;
        case 2:
            *px = WIDTH - *px - 1;
            *py = HEIGHT - *py - 1;
            break;
        case 3:
            *py = HEIGHT - *px - 1;
            *px = y0;
            break;
    }
}

void gfx_drawPixel( int16_t x, int16_t y, uint16_t color ){
    
    if( (x < 0) || (x >= _width) || (y < 0) || (y >= _height) )
        return;
    
    gfx_rotation_adjust( &x, &y );

    display_setPixel(x,y,color);
}

// helper function for gfx_drawLine, handles special cases of horizontal and vertical lines
static void gfx_hvLine( int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color ){
    
    if( x1 != x2 && y1 != y2 ){
        // neither vertical nor horizontal
        return;
    }    
    
    // bounds check
    if( rotation == 1 || rotation == 3 ){
        if( x1 < 0 || x1 >= HEIGHT || x2 < 0 || x2 >= HEIGHT )
            return;
        if( y1 < 0 || y1 >= WIDTH || y2 < 0 || y2 >= WIDTH )
            return;
    }else{
        if( y1 < 0 || y1 >= HEIGHT || y2 < 0 || y2 >= HEIGHT )
            return;
        if( x1 < 0 || x1 >= WIDTH || x2 < 0 || x2 >= WIDTH )
            return;
    }
    
    gfx_rotation_adjust( &x1, &y1 );
    gfx_rotation_adjust( &x2, &y2 );
    
    // ensure coords are from left to right and top to bottom
    if( (x1 == x2 && y2 < y1) || (y1 == y2 && x2 < x1) ){
        // swap as needed
        int16_t t = x1; x1 = x2; x2 = t;
        t = y1; y1 = y2; y2 = t;
    }
    
    display_line( x1, y1, x2, y2, color );
}

// always use this function for line drawing
void gfx_drawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color ){
 
    if( x0 == x1 || y0 == y1 ){
        // vertical and horizontal lines can be drawn faster
        gfx_hvLine( x0, y0, x1, y1, color );
        return;
    }
    
    int16_t t;
    
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if( steep ){
        t = x0; x0 = y0; y0 = t;
        t = x1; x1 = y1; y1 = t;
    }
    if( x0 > x1 ){
        t = x0; x0 = x1; x1 = t;
        t = y0; y0 = y1; y1 = t;
    }
    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep;
    if( y0 < y1 ){
        ystep = 1;
    }else{
        ystep = -1;
    }
    for( ; x0<=x1; x0++ ){
        if( steep ){
            gfx_drawPixel( y0, x0, color );
        }else{
            gfx_drawPixel( x0, y0, color );
        }
        err -= dy;
        if( err < 0 ){
            y0 += ystep;
            err += dx;
        }
    }
}

void gfx_drawRect( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color ){
    
    gfx_drawLine( x, y, x+w-1, y, color );
    gfx_drawLine( x, y+h-1, x+w-1, y+h-1, color );
    gfx_drawLine( x, y, x, y+h-1, color );
    gfx_drawLine( x+w-1, y, x+w-1, y+h-1, color );
}

void gfx_fillRect( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color ){
    int16_t i = 0;
    if( h > w ){
        for( i = x ; i < x+w ; i++ ){
            gfx_drawLine( i, y, i, y+h-1, color );
        }
    }else{
        for( i = y ; i < y+h ; i++ ){
            gfx_drawLine( x, i, x+w-1, i, color );
        }
    }
}


// circle outline
void gfx_drawCircle( int16_t x0, int16_t y0, int16_t r,uint16_t color ){

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    gfx_drawPixel( x0 , y0+r, color );
    gfx_drawPixel( x0 , y0-r, color );
    gfx_drawPixel( x0+r, y0 , color );
    gfx_drawPixel( x0-r, y0 , color );
    while( x < y ){
        if( f >= 0 ){
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        gfx_drawPixel( x0 + x, y0 + y, color );
        gfx_drawPixel( x0 - x, y0 + y, color );
        gfx_drawPixel( x0 + x, y0 - y, color );
        gfx_drawPixel( x0 - x, y0 - y, color );
        gfx_drawPixel( x0 + y, y0 + x, color );
        gfx_drawPixel( x0 - y, y0 + x, color );
        gfx_drawPixel( x0 + y, y0 - x, color );
        gfx_drawPixel( x0 - y, y0 - x, color );
    }
}

void gfx_drawTriangle( int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color ){
    
    gfx_drawLine( x0, y0, x1, y1, color );
    gfx_drawLine( x1, y1, x2, y2, color );
    gfx_drawLine( x2, y2, x0, y0, color );
}


// Draw a character
void gfx_drawChar( int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size) {
    if( (x >= _width) || // Clip right
        (y >= _height) || // Clip bottom
        ((x + 6 * size - 1) < 0) || // Clip left
        ((y + 8 * size - 1) < 0)) // Clip top
        return;

    int8_t i = 0;
    for( i = 0 ; i < 6 ; i++ ){
        uint8_t line;
        if( i == 5 )
            line = 0x0;
        else
           line = font[(c*5)+i];
        int8_t j = 0;
        for( j = 0; j < 8 ; j++ ){
            if( line & 0x1 ){
                if( size == 1 ) // default size
                    gfx_drawPixel( x+i, y+j, color );
                else { // big size
                    gfx_fillRect( x+(i*size), y+(j*size), size, size, color );
                }
            } else if( bg != color ){
                if( size == 1 ) // default size
                    gfx_drawPixel( x+i, y+j, bg );
                else { // big size
                    gfx_fillRect( x+i*size, y+j*size, size, size, bg );
                }
            }
            line >>= 1;
        }
    }
}

void gfx_write( uint8_t ch ){
    if( ch == '\n' ){
        cursor_y += textsize*8;
        cursor_x = 0;
    }else if( ch == '\r' ){
        // skip em
    }else{
        gfx_drawChar(cursor_x, cursor_y, ch, textcolor, textbgcolor, textsize);
        cursor_x += textsize*6;
        if( wrap && (cursor_x > (_width - textsize*6)) ){
            cursor_y += textsize*8;
            cursor_x = 0;
        }
    }
}

void gfx_print( const char* s ){
    
    unsigned int len = strlen( s );
    unsigned int i = 0; 
    for( i = 0 ; i < len ; i++ ){
        gfx_write( s[i] );
    }
}

void gfx_println( const char* s ){ 
    gfx_print( s ); 
    gfx_write( '\n' );
}
/* [] END OF FILE */
