/*
 * Author - Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
 
// Serial LCD module's SCLK(clock) and SI(data) must be connected
// to Atmega's PC4 and PC5, respectively.

#define OUT_C_LCD_SCL OUT_C_LCD_RnW     
#define OUT_C_LCD_SI  OUT_C_LCD_E       

// SI     PC5 40
// SCK    PC4 39
// A0 DC  PC3 38 
// RESET  PC2 37
// CS     PC1 36



#include "er9x.h"
#include <stdlib.h>

#define DBL_FONT_SMALL	1

uint8_t lcd_lastPos;

uint8_t displayBuf[DISPLAY_W*DISPLAY_H/8];
#define DISPLAY_END (displayBuf+sizeof(displayBuf))


const prog_uchar APM font[] = {
#include "font.lbm"
};

#define font_5x8_x20_x7f (font)

const prog_uchar APM font_dblsize[] = {
#include "font_dblsize.lbm"
};

#define font_10x16_x20_x7f (font_dblsize)

#define delay_1us() _delay_us(1)
#define delay_2us() _delay_us(2)
static void delay_1_5us(int ms)
{
  for(int i=0; i<ms; i++) delay_1us();
}


void lcd_clear()
{
  //for(unsigned i=0; i<sizeof(displayBuf); i++) displayBuf[i]=0;
  memset(displayBuf, 0, sizeof(displayBuf));
}


void lcd_img(uint8_t i_x,uint8_t i_y,const prog_uchar * imgdat,uint8_t idx/*,uint8_t mode*/)
{
  const prog_uchar  *q = imgdat;
  uint8_t w    = pgm_read_byte(q++);
  uint8_t hb   = (pgm_read_byte(q++)+7)/8;
  uint8_t sze1 = pgm_read_byte(q++);
  q += idx*sze1;
//  bool    inv  = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);
  for(uint8_t yb = 0; yb < hb; yb++){
    uint8_t   *p = &displayBuf[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
    for(uint8_t x=0; x < w; x++){
      uint8_t b = pgm_read_byte(q++);
      *p++ = b;
      //*p++ = inv ? ~b : b;
    }
  }
}

uint8_t lcd_putc(uint8_t x,uint8_t y,const char c)
{
  return lcd_putcAtt(x,y,c,0);
}

/// invers: 0 no 1=yes 2=blink
uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode)
{
    uint8_t *p    = &displayBuf[ y / 8 * DISPLAY_W + x ];
    //uint8_t *pmax = &displayBuf[ DISPLAY_H/8 * DISPLAY_W ];
		if ( c < 22 )		// Move to specific x position (c)*FW
		{
			x = c * FW ;
//  		if(mode&DBLSIZE)
//			{
//				x += x ;
//			}
			return x ;
		}
		x += FW ;
    const prog_uchar    *q = &font_5x8_x20_x7f[(c-0x20)*5];
    bool         inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);
    if(mode&DBLSIZE)
    {
			if ( (c!=0x2E)) x+=FW; //check for decimal point
	/* each letter consists of ten top bytes followed by
	 * five bottom by ten bottom bytes (20 bytes per 
	 * char) */

		  unsigned char c_mapped ;

#ifdef DBL_FONT_SMALL
			if ( c >= ',' && c <= ':' )
			{
				c_mapped = c - ',' + 1 ;		
			}
  		else if (c>='A' && c<='Z')
			{
				c_mapped = c - 'A' + 0x10 ;
			}
  		else if (c>='a' && c<='z')
			{
				c_mapped = c - 'a' + 0x2B ;
			}
  		else if (c=='_' )
			{
				c_mapped = 0x2A ;
			}
			else
			{
				c_mapped = 0 ;
			}
#else
			c_mapped = c - 0x20 ;
#endif
        q = &font_10x16_x20_x7f[(c_mapped)*20] ;// + ((c-0x20)/16)*160];
        for(char i=11; i>=0; i--){
	    /*top byte*/
            uint8_t b1 = i>1 ? pgm_read_byte(q) : 0;
	    /*bottom byte*/
            uint8_t b3 = i>1 ? pgm_read_byte(10+q) : 0;
	    /*top byte*/
//            uint8_t b2 = i>0 ? pgm_read_byte(++q) : 0;
	    /*bottom byte*/
//            uint8_t b4 = i>0 ? pgm_read_byte(10+q) : 0;
            q++;
            if(inv) {
                b1=~b1;
//                b2=~b2;
                b3=~b3;
//                b4=~b4;
            }

            if(&p[DISPLAY_W+1] < DISPLAY_END){
                p[0]=b1;
//                p[1]=b2;
                p[DISPLAY_W] = b3;
//                p[DISPLAY_W+1] = b4;
                p+=1;
            }
        }
//        q = &dbl_font[(c-0x20)*20];
//        for(char i=0; i<10; i++){
//            uint8_t b = pgm_read_byte(q++);
//            if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~b : b;
//            b = pgm_read_byte(q++);
//            if(p<DISPLAY_END) *p = inv ? ~b : b;
//            p++;
//        }
//        if(p<DISPLAY_END) *p = inv ? ~0 : 0;
//        if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~0 : 0;
    }
    else
    {
        uint8_t condense=0;

        if (mode & CONDENSED) {
            *p = inv ? ~0 : 0;
						p += 1 ;
            condense=1;
	    	x += FWNUM-FW ;
	}

        for(char i=5; i!=0; i--){
            uint8_t b = pgm_read_byte(q++);
    	    if (condense && i==4) {
                /*condense the letter by skipping column 4 */
                continue;
            }
            if(p<DISPLAY_END) {*p = inv ? ~b : b; p += 1 ; }
        }
        if(p<DISPLAY_END) *p++ = inv ? ~0 : 0;
    }
		return x ;
}

// Puts sub-string from string options
// First byte of string is sub-string length
// idx is index into string (in length units)
// Output length characters
void lcd_putsAttIdx(uint8_t x,uint8_t y,const prog_char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = pgm_read_byte(s++) ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}


//uint8_t lcd_putsnAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t len,uint8_t mode)
void lcd_putsnAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t len,uint8_t mode)
{
	uint8_t source ;
//	uint8_t size ;
	source = mode & BSS ;
//	size = mode & DBLSIZE ;
  while(len!=0) {
    char c = (source) ? *s++ : pgm_read_byte(s++);
    x = lcd_putcAtt(x,y,c,mode);
//    x+=FW;
//		if ((size)&& (c!=0x2E)) x+=FW; //check for decimal point
    len--;
  }
}
void lcd_putsn_P(uint8_t x,uint8_t y,const prog_char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}

//uint8_t lcd_puts2Att(uint8_t x,uint8_t y,const prog_char * s,const prog_char * t ,uint8_t mode)
//{
//	x = lcd_putsAtt( x, y, s, mode ) ;
//	return lcd_putsAtt( x, y, t, mode ) ;	
//}

uint8_t lcd_putsAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t mode)
{
	uint8_t source ;
//	uint8_t size ;
	source = mode & BSS ;
//	size = mode & DBLSIZE ;
  //while(char c=pgm_read_byte(s++)) {
  while(1) {
    char c = (source) ? *s++ : pgm_read_byte(s++);
    if(!c) break;
    x = lcd_putcAtt(x,y,c,mode);
//    x+=FW;
//		if ((size)&& (c!=0x2E)) x+=FW; //check for decimal point
  }
  return x;
}

void lcd_puts_Pleft(uint8_t y,const prog_char * s)
{
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_puts_P(uint8_t x,uint8_t y,const prog_char * s)
{
  lcd_putsAtt( x, y, s, 0);
}
void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
  x+=FWNUM*4;
  for(int i=0; i<4; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}
void lcd_outdez(uint8_t x,uint8_t y,int16_t val)
{
  lcd_outdezAtt(x,y,val,0);
}

void lcd_outdezAtt(uint8_t x,uint8_t y,int16_t val,uint8_t mode)
{
  lcd_outdezNAtt( x,y,val,mode,5);
}

#define PREC(n) ((n&0x20) ? ((n&0x10) ? 2 : 1) : 0)
uint8_t lcd_outdezNAtt(uint8_t x,uint8_t y,int32_t val,uint8_t mode,int8_t len)
{
  uint8_t fw = FWNUM;
  uint8_t prec = PREC(mode);
	uint8_t negative = 0 ;
  uint8_t xn = 0;
  uint8_t ln = 2;
  char c;
  uint8_t xinc ;
	uint8_t fullwidth = 0 ;

	mode &= ~NO_UNIT ;
	if ( len < 0 )
	{
		fullwidth = 1 ;
		len = -len ;		
	}

  if ( val < 0 )
	{
		val = -val ;
		negative = 1 ;
	}

  if (mode & DBLSIZE)
  {
    fw += FWNUM ;
    xinc = 2*FWNUM;
    lcd_lastPos = 2*FW;
  }
  else
  {
    xinc = FWNUM ;
    lcd_lastPos = FW;
  }

  if (mode & LEFT) {
//    if (val >= 10000)
//      x += fw;
    if(negative)
    {
      x += fw;
    }
    if (val >= 1000)
      x += fw;
    if (val >= 100)
      x += fw;
    if (val >= 10)
      x += fw;
    if ( prec )
    {
      if ( prec == 2 )
      {
        if ( val < 100 )
        {
          x += fw;
        }
      }
      if ( val < 10 )
      {
        x+= fw;
      }
    }
  }
  else
  {
    x -= xinc;
  }
  lcd_lastPos += x ;

  if ( prec == 2 )
  {
    mode -= LEADING0;  // Can't have PREC2 and LEADING0
  }

  for (uint8_t i=1; i<=len; i++)
	{
		div_t qr ;
		qr = div( val, 10 ) ;
    c = (qr.rem) + '0';
    lcd_putcAtt(x, y, c, mode);
    if (prec==i) {
      if (mode & DBLSIZE) {
        xn = x;
        if( c<='3' && c>='1') ln++;
        uint8_t tn = (qr.quot) % 10;
        if(tn==2 || tn==4) {
          if (c=='4') {
            xn++;
          }
          else {
            xn--; ln++;
          }
        }
      }
      else {
        x -= 2;
        if (mode & INVERS)
          lcd_vline(x+1, y, 7);
        else
          lcd_plot(x+1, y+6);
      }
      if (qr.quot)
        prec = 0;
    }
    val = qr.quot ;
    if (!val)
    {
      if (prec)
      {
        if ( prec == 2 )
        {
          if ( i > 1 )
          {
            prec = 0 ;
          }
        }
        else
        {
          prec = 0 ;
        }
      }
      else if (mode & LEADING0)
			{
				if ( fullwidth == 0 )
				{
        	mode -= LEADING0;
				}
			}
      else
        break;
    }
    x-=fw;
  }
  if (xn) {
    lcd_hline(xn, y+2*FH-4, ln);
    lcd_hline(xn, y+2*FH-3, ln);
  }
  if(negative) lcd_putcAtt(x-fw,y,'-',mode);
	return 0 ;		// Stops compiler creating two sets of POPS, saves flash
}

void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent )
{
	uint8_t solid ;
	if ( percent > 100 )
	{
		percent = 100 ;
	}
	solid = (w-2) * percent / 100 ;
	lcd_rect( x, y, w, h ) ;

	if ( solid )
	{
		w = y + h - 1 ;
		y += 1 ;
		x += 1 ;
		while ( y < w )
		{
 			lcd_hline(x, y, solid ) ;
			y += 1 ;			
		}
	}
}

// Reverse video 8 pixels high, w pixels wide
// Vertically on an 8 pixel high boundary
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
	if ( blink && BLINK_ON_PHASE )
	{
		return ;
	}
	uint8_t end = x + w ;
  uint8_t *p = &displayBuf[ y / 8 * DISPLAY_W + x ];

	while ( x < end )
	{
		*p++ ^= 0xFF ;
		x += 1 ;
	}
}

void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
  lcd_vline(x, y, h ) ;
	if ( w > 1 )
	{
  	lcd_vline(x+w-1, y, h ) ;
	}
 	lcd_hline(x+1, y+h-1, w-2 ) ;
 	lcd_hline(x+1, y, w-2 ) ;
}

void lcd_plot(uint8_t x,uint8_t y)
{
  //  if(y>=64)  return;
  //  if(x>=128) return;
  //  displayBuf[ y / 8 * DISPLAY_W + x ] ^= BITMASK(y%8);
  uint8_t *p   = &displayBuf[ y / 8 * DISPLAY_W + x ];
  if(p<DISPLAY_END) *p ^= BITMASK(y%8);
}
void lcd_hlineStip(unsigned char x,unsigned char y, signed char w,uint8_t pat)
{
  if(w<0) {x+=w; w=-w;}
  uint8_t *p  = &displayBuf[ y / 8 * DISPLAY_W + x ];
  uint8_t msk = BITMASK(y%8);
  while(w){
    if ( p>=DISPLAY_END)
    {
      break ;			
    }
    if(pat&1) {
      //lcd_plot(x,y);
      *p ^= msk;
      pat = (pat >> 1) | 0x80;
    }else{
      pat = pat >> 1;
    }
    w--;
    p++;
  }
}

void lcd_hline(uint8_t x,uint8_t y, int8_t w)
{
  lcd_hlineStip(x,y,w,0xff);
}

void lcd_vline(uint8_t x,uint8_t y, int8_t h)
{
//    while ((y+h)>=DISPLAY_H) h--;
		uint8_t y1 ;
    uint8_t *p  = &displayBuf[ y / 8 * DISPLAY_W + x ];
		y1 = y + h ;
    uint8_t *q  = &displayBuf[ y1 / 8 * DISPLAY_W + x ];
    *p ^= ~(BITMASK(y%8)-1);
    while(p<q){
        p  += DISPLAY_W;
        if ( p>=DISPLAY_END)
        {
          break ;			
        }
        *p ^= 0xff;
    }
    if(p<DISPLAY_END) *p ^= ~(BITMASK((y+h)%8)-1);
}

// Supports 4W serial LCD interface and SSD1306 OLED controller
// - Hyun-Taek Chang (flybabo@att.net), Feb 2013

// controller independent options

 #define SERIAL_LCD      1       // parallel=0, 4W_serial=1
 #define ROTATE_SCREEN   0       // don't-rotate-screen=0, rotate-180-degree=1
 #define REVERSE_VIDEO   0       // don't-reverse-video=0, reverse-video=1


// force inline expansion
#define ALWAYS_INLINE   __attribute__((always_inline))

static void lcdSendByte(uint8_t val, uint8_t v0, uint8_t v1) ALWAYS_INLINE;
static void lcdEndSend() ALWAYS_INLINE;

static void lcdEndSend()
{
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_CS1); // disable chip select
}

static void lcdSendBit(uint8_t b, uint8_t v0, uint8_t v1) ALWAYS_INLINE;

static void lcdSendBit(uint8_t b, uint8_t v0, uint8_t v1)
{
  PORTC_LCD_CTRL = v0;                  // out 0x15, r19  ; 1 cycle
  if (b != 0)                           // sbrc r24, 7    ; 1 cycle
    PORTC_LCD_CTRL = v1;                // out 0x15, r18  ; 1 cycle
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_SCL); // sbi 0x15, 4    ; 2 cycle 
}

static void lcdSendByte(uint8_t val, uint8_t v0, uint8_t v1)
{
  lcdSendBit((val & 0x80), v0, v1);
  lcdSendBit((val & 0x40), v0, v1);
  lcdSendBit((val & 0x20), v0, v1);
  lcdSendBit((val & 0x10), v0, v1);
  lcdSendBit((val & 0x08), v0, v1);
  lcdSendBit((val & 0x04), v0, v1);
  lcdSendBit((val & 0x02), v0, v1);
  lcdSendBit((val & 0x01), v0, v1);
}

static void lcdSendCtl(uint8_t val)
{
  uint8_t v0c = 0xC5; // PC7=1,PC6=1,SI=0,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  uint8_t v1c = 0xE5; // PC7=1,PC6=1,SI=1,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  for (uint8_t n = 8; n > 0; n--) {
    lcdSendBit((val & 0x80), v0c, v1c);
    val <<= 1;
  }
  lcdEndSend();
}

static void lcdSendData(uint8_t val)
{
  uint8_t v0d = 0xCD; // PC7=1,PC6=1,SI=0,SCL=0,A0=1,RES=1,CS1=0,PC0=1
  uint8_t v1d = 0xED; // PC7=1,PC6=1,SI=1,SCL=0,A0=1,RES=1,CS1=0,PC0=1
  for (uint8_t n = 8; n > 0; n--) {
    lcdSendBit((val & 0x80), v0d, v1d);
    val <<= 1;
  }
  lcdEndSend();
}

static void lcdSendWord(uint16_t val)
{

	uint8_t data1 = val>>8;
	uint8_t data2 = val&0xff;
	
	lcdSendData(data1);
	lcdSendData(data2);

}

void setCol(uint16_t StartCol,uint16_t EndCol)
{
    lcdSendCtl(0x2A);                                                      /* Column Command address       */
    lcdSendWord(StartCol);
    lcdSendWord(EndCol);
}

void setPage(uint16_t StartPage,uint16_t EndPage)
{
    lcdSendCtl(0x2B);                                                      /* Page Command address       */
    lcdSendWord(StartPage);
    lcdSendWord(EndPage);
}

void clearScreen(void)
{

    setCol(0, 319);
    setPage(0, 239);
    lcdSendCtl(0x2c);                                                  /* start to write to display ra */

    for(uint16_t i=0; i<38400; i++)
    {
	lcdSendWord(0);
	lcdSendWord(0);
	lcdSendWord(0);
	lcdSendWord(0);
	}
    lcdEndSend();
}

static void init_ILI9341() 
{

		lcdSendCtl(0x01);
        delay_1_5us(10000);

        lcdSendCtl(0xCB);  
        lcdSendData(0x39); 
        lcdSendData(0x2C); 
        lcdSendData(0x00); 
        lcdSendData(0x34); 
        lcdSendData(0x02); 

        lcdSendCtl(0xCF);  
        lcdSendData(0x00); 
        lcdSendData(0XC1); 
        lcdSendData(0X30); 

        lcdSendCtl(0xE8);  
        lcdSendData(0x85); 
        lcdSendData(0x00); 
        lcdSendData(0x78); 

        lcdSendCtl(0xEA);  
        lcdSendData(0x00); 
        lcdSendData(0x00); 

        lcdSendCtl(0xED);  
        lcdSendData(0x64); 
        lcdSendData(0x03); 
        lcdSendData(0X12); 
        lcdSendData(0X81); 

        lcdSendCtl(0xF7);  
        lcdSendData(0x20); 

        lcdSendCtl(0xC0);            //Power control 
        lcdSendData(0x23);           //VRH[5:0] 

        lcdSendCtl(0xC1);            //Power control 
        lcdSendData(0x10);           //SAP[2:0];BT[3:0] 

        lcdSendCtl(0xC5);            //VCM control 
        lcdSendData(0x3e);           //Contrast
        lcdSendData(0x28); 

        lcdSendCtl(0xC7);            //VCM control2 
        lcdSendData(0x86);           //--

        lcdSendCtl(0x36);            // Memory Access Control 
        lcdSendData(0x48);          //C8           //48 68绔栧睆//28 E8 妯睆

        lcdSendCtl(0x3A);    
        lcdSendData(0x55); 

        lcdSendCtl(0xB1);    
        lcdSendData(0x00);  
        lcdSendData(0x18); 

        lcdSendCtl(0xB6);            // Display Function Control 
        lcdSendData(0x08); 
        lcdSendData(0x82);
        lcdSendData(0x27);  
 
        lcdSendCtl(0xF2);            // 3Gamma Function Disable 
        lcdSendData(0x00); 
        
        lcdSendCtl(0x36);			// Rotacion
        lcdSendData(0xE8);

        lcdSendCtl(0x55);			// Brillo adaptativo a UI
        lcdSendData(0x01);

        lcdSendCtl(0x11);            //Exit Sleep 
        
        delay_1_5us(50000);
        delay_1_5us(50000);
        delay_1_5us(50000);
        
        lcdSendCtl(0x29);    //Display on 
        lcdSendCtl(0x2c);   
        
        
        lcdSendCtl(0x36);
        lcdSendData(0xE8);
        
		clearScreen();

		
}

void lcd_init() {

  LcdLock = 1 ;            // Lock LCD data lines
  
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_RES);
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000); 
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RES);  //LCD_RES
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000);
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_RES);   //  f524  sbi 0x15, 2 IOADR-PORTC_LCD_CTRL; 21           1
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000);
  delay_1_5us(50000);
  
  init_ILI9341();
  
  g_eeGeneral.contrast = lcd_nomContrast;
  LcdLock = 0 ;            // Free LCD data lines

}

void lcdSetContrast()
{
  return;
  lcdSetRefVolt(g_eeGeneral.contrast);
}

void lcdSetRefVolt(uint8_t val)
{
  return;

  LcdLock = 1 ;            // Lock LCD data lines
  lcdSendCtl(0x81);
  lcdSendCtl(val);
  LcdLock = 0 ;            // Free LCD data lines
}

volatile uint8_t LcdLock ;

uint16_t constrain(uint16_t x, uint16_t a, uint16_t b) 
{
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}


void fillScreen(uint16_t XL, uint16_t XR, uint16_t YU, uint16_t YD, uint16_t color)
{
    unsigned long XY=0;
    unsigned long i=0;
    
    if(XL > XR)
    {
        XL = XL^XR;
        XR = XL^XR;
        XL = XL^XR;
    }
    if(YU > YD)
    {
        YU = YU^YD;
        YD = YU^YD;
        YU = YU^YD;
    }
    XL = constrain(XL, 0,128);
    XR = constrain(XR, 0,128);
    YU = constrain(YU, 0,64);
    YD = constrain(YD, 0,64);

    XY = (XR-XL+1);
    XY = XY*(YD-YU+1);

    setCol(XL,XR);
    setPage(YU, YD);
    lcdSendCtl(0x2c);            
    
    uint8_t Hcolor = color>>8;
    uint8_t Lcolor = color&0xff;                                      
 
    for(i=0; i < XY; i++)
    {
		lcdSendData(Hcolor);
		lcdSendData(Lcolor);
    }

}

void fillRectangle(uint16_t poX, uint16_t poY, uint16_t length, uint16_t width, uint16_t color)
{
    fillScreen(poX, poX+length, poY, poY+width, color);
}

void setPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
    setCol(poX, poX);
    setPage(poY, poY);
    lcdSendCtl(0x2c);
    lcdSendWord(color);
}

void drawChar(uint8_t bloque, uint16_t poX, uint16_t poY,uint16_t size, uint16_t fgcolor)
{

	uint8_t v0d = 0xCD; // PC7=1,PC6=1,SI=0,SCL=0,A0=1,RES=1,CS1=0,PC0=1
	uint8_t v1d = 0xED; // PC7=1,PC6=1,SI=1,SCL=0,A0=1,RES=1,CS1=0,PC0=1
  
	setCol(poX, poX);
    setPage(poY, poY+7);
    lcdSendCtl(0x2c);

    for (int i =0; i<8; i++ ) {

        for(uint8_t f=0;f<8;f++)
        {
            if( ( bloque >> f) & 0x01 )
            {
                lcdSendByte(0xff,v0d, v1d);
                lcdSendByte(0xff,v0d, v1d);
				}
            else
            {
				lcdSendByte(0x00,v0d, v1d);
				lcdSendByte(0x00,v0d, v1d);
                }

        }

    }
    
      lcdEndSend();
}


void refreshDiplay()
{

  LcdLock = 1 ;             // Lock LCD data line
  
  uint8_t *p=displayBuf;
  uint8_t y=0;
  
  while(y< 64) {
 
				for(uint8_t x=0; x<128; x++){
				
						drawChar(*p++,x,y,8,0xffff);
										
				};
				y=y+8;
  }

  lcdEndSend();
  LcdLock = 0 ;            // Free LCD data lines

}




