#ifndef LCDi2cR_h
#define LCDi2cR_h


#define _LCDEXPANDED				// If defined turn on advanced functions

#include <inttypes.h>

#include "Print.h"



class LCDi2cR : public Print {

public: 

  LCDi2cR(uint8_t num_lines, uint8_t num_col, uint8_t i2c_address, uint8_t display);
  
  void command(uint8_t value);

  void init();

  void setDelay(int,int);

  virtual void write(uint8_t);

  void clear();
  
  void home();
  
  void on();
  
  void off();
  
  void cursor_on();
  
  void cursor_off();
  
  void blink_on();
  
  void blink_off();
	
  void setCursor(uint8_t Line, uint8_t Col );
	
  void backlight(uint8_t value);
	
	
	// [][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
	// []
	// []	Extended Functions
	// []
	// [][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][]
	
	
	
#ifdef _LCDEXPANDED		
	
	
	

	
	int status();
	
	uint8_t keypad();
	
	void load_custom_character(uint8_t char_num, uint8_t *rows);
	
	
#define LCDi2c_MIN_BRIGHTNESS		0
#define LCDi2c_MAX_BRIGHTNESS		250
#define LCDi2c_VALUE_OUT_OF_RANGE	1
#define LCDi2c_MIN_CONTRAST			0
#define LCDi2c_MAX_CONTRAST			100
  
  //void left();
  
  //void right();
  
  //uint8_t keypad();
 
  	// Overload the Print class function for strings because
	// the base class implementation causes problems by doing
	// an I2C I/O for every character
  //void printstr(const char[]);
	

/*
 
// Values for graphtype in calls to init_bargraph
#define LCDi2c_VERTICAL_BAR_GRAPH    1
#define LCDi2c_HORIZONTAL_BAR_GRAPH  2
#define LCDi2c_HORIZONTAL_LINE_GRAPH 3

  unsigned char init_bargraph(unsigned char graphtype);
  void draw_horizontal_graph(unsigned char row, unsigned char column, unsigned char len,  unsigned char pixel_col_end);
  void draw_vertical_graph(unsigned char row, unsigned char column, unsigned char len,  unsigned char pixel_col_end);

#define LCDi2c_NUM_CUSTOM_CHARS 8
#define LCDi2c_CUSTOM_CHAR_SIZE 8
  void load_custom_character(unsigned char char_num, unsigned char *rows);
  unsigned char set_backlight_brightness(unsigned char new_val);
  unsigned char set_contrast(unsigned char new_val);
  
*/
	
#endif

private:
	

};

#endif

