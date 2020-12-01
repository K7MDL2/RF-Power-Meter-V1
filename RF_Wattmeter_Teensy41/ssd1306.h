/* ========================================
 *
 *  SSD1306.H
 *
 * ========================================
*/

#include "util\my_types.h"
//#ifdef SSD1306_OLED
#ifndef _SSD1306_H
#define _SSD1306_H
    
#define BLACK 0
#define WHITE 1
#define INVERSE 2   

typedef enum{
    SCROLL_RIGHT = 0x26,
    SCROLL_LEFT = 0x2A
}SCROLL_DIR;

typedef enum{
    SCROLL_SPEED_0 = 0x03,  // slowest
    SCROLL_SPEED_1 = 0x02,
    SCROLL_SPEED_2 = 0x01,
    SCROLL_SPEED_3 = 0x06,
    SCROLL_SPEED_4 = 0x00,
    SCROLL_SPEED_5 = 0x05,
    SCROLL_SPEED_6 = 0x04,
    SCROLL_SPEED_7 = 0x07   // fastest
}SCROLL_SPEED;

typedef enum{
    SCROLL_PAGE_0 = 0,
    SCROLL_PAGE_1,
    SCROLL_PAGE_2,
    SCROLL_PAGE_3,
    SCROLL_PAGE_4,
    SCROLL_PAGE_5,
    SCROLL_PAGE_6,
    SCROLL_PAGE_7   
}SCROLL_AREA;

void display_init( unsigned char i2caddr );
void display_update(void);
void display_clear(void);
void display_stopscroll(void);
void display_scroll( SCROLL_AREA start, SCROLL_AREA end, SCROLL_DIR dir, SCROLL_SPEED speed );
void display_contrast( unsigned char contrast );
void display_invert( unsigned char invert );

void gfx_drawPixel(int16_t x, int16_t y, uint16_t color);
void gfx_drawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color );
void gfx_setCursor( int16_t x, int16_t y );
void gfx_setTextSize( unsigned char size );
void gfx_setTextColor( uint16_t color );
void gfx_setTextBg( uint16_t background );
void gfx_write( unsigned char ch );
int16_t gfx_width(void);
int16_t gfx_height(void);
void gfx_print( const char* s );
void gfx_println( const char* s );
void gfx_drawRect( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color );
void gfx_fillRect( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color );
void gfx_drawCircle( int16_t x0, int16_t y0, int16_t r,uint16_t color );
void gfx_drawTriangle( int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color );
void gfx_setRotation( unsigned char x );

#endif	// _SSD1306_H
//#endif // SSD1306_OLED

/* [] END OF FILE */
