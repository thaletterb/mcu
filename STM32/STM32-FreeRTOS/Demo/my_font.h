#ifndef MY_FONT_H__
#define MY_FONT_H__

// My basic ascii font
// Each char is 8px tall x 5 px wide
//

#define ASCII_8x5_FONT_WIDTH    5
#define ASCII_8x5_FONT_HEIGHT   8
#define ASCII_8x5_OFFSET        32

#define ASCII_24x40_FONT_WIDTH          24
#define ASCII_24x40_FONT_HEIGHT         40
#define ASCII_24x40_FONT_NUM_CHARS      27  // Num of chars so far in Tahoma24x40[] array
#define ASCII_24x40_FONT_ASCII_OFFSET   32  // First ascii char to appear in Tahoma24x40[] array

extern const unsigned short Tahoma24x40[ASCII_24x40_FONT_NUM_CHARS][120];
extern const char Ascii_8x5_font[97][5];
#endif
