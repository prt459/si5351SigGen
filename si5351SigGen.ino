/*  
Arduino Nano script for homebrew Signal Generator based on the si5351. 
Written by Paul Taylor, VK3HN (https://vk3hn.wordpress.com/) 
Targets Ashar Farhan VU2ESE's Arduino Nano/si5351 module (Raduino). 
V1.0, 12 Jul 2020 - first version for homebrew Arduino/si5351 VFO
*/

// common libraries
#include <Rotary.h>
#include <si5351.h>     // Etherkit si3531  V2.0.1   https://github.com/etherkit/Si5351Arduino 
#include <Wire.h>
#include <EEPROM.h>

// common #define's that precede other declarations
#define LCD_RS    8  // Register Select is LCD pin 4 
#define LCD_E     9  // Enable/clock LCD pin 6
#define LCD_D4   10  // LCD D4 
#define LCD_D5   11  // LCD D5 
#define LCD_D6   12  // LCD D6 
#define LCD_D7   13  // LCD D7  

#define TUNE_MS 3000 
bool mode_tune = 0;

#define BUTTON_HELD_MS 1000

#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    3 // number of selectable VFOs 

// #define CW_KEYER      // include the CW keyer code

// Arduino Nano digital pin assignments (aligns with Raduino)
//                0     Serial
//                1     Serial
#define ENCODER_B 2  // Encoder pin B
#define ENCODER_A 3  // Encoder pin A

// Arduino Nano analogue pins
#define SWITCH_BANK       A0 // front panel push buttons

#ifdef  CW_KEYER
#define PIN_PADDLE        A1 // paddle on analog pin 1
#define PIN_PUSHBTTN_REAR A2 // keyer memory pushbuttons on analog pin 2
#define PIN_KEYER_SPEED   A3 // speed potentiometer wiper
#endif
//                        A4    SDA
//                        A5    SCL

// i2c devices and addresses:
// si5351  x60

bool message_playing = false;      // true when the keyer is playing a CW message 
#define CW_TONE_HZ       700  // CW tone frequency (Hz)
unsigned int dot_dash_counter = 0;  // total nbr CW chars sent since power-up
unsigned int dot_dash_sent = 0;     // nbr CW chars sent this transmit period, used for timing a refresh to the (interleaved) RF relative power meter
#define PIN_TONE_OUT       6  // digital pin with keyed audio tone on it
bool key_down = false; 
unsigned long char_sent_ms, curr_ms;
bool space_inserted;
#define KEYER_MSG_SPEED 50


#ifdef CW_KEYER
// start of CW Keyer block -------------------------------------------------------------

#define PADDLE_R           1      // value representing analog value for paddle left (dot)
#define PADDLE_L           2      // value representing analog value for paddle right (dash)

#define CW_DASH_LEN        5  // length of dash (in dots)
#define BREAK_IN_DELAY   800  // break-in hang time (mS)
#define SERIAL_LINE_WIDTH 80  // number of morse chars on Serial after which we newline 

// set the CW keyer speed, lower is faster, 60 is 10 w.p.m.
byte  dot_length_ms = 55; 

                     
// morse reference table
struct morse_char_t {
  char ch[7]; 
};

morse_char_t MorseCode[] = {
  {'A', '.', '-',  0,   0,   0,   0},
  {'B', '-', '.', '.', '.',  0,   0},
  {'C', '-', '.', '-', '.',  0,   0},
  {'D', '-', '.', '.',  0,   0,   0},
  {'E', '.',  0,   0,   0,   0,   0},
  {'F', '.', '.', '-', '.',  0,   0},
  {'G', '-', '-', '.',  0,   0,   0},
  {'H', '.', '.', '.', '.',  0,   0},
  {'I', '.', '.',  0,   0,   0,   0},
  {'J', '.', '-', '-', '-',  0,   0},
  {'K', '-', '.', '-',  0,   0,   0},
  {'L', '.', '-', '.', '.',  0,   0},
  {'M', '-', '-',  0,   0,   0,   0},
  {'N', '-', '.',  0,   0,   0,   0},
  {'O', '-', '-', '-',  0,   0,   0},
  {'P', '.', '-', '-', '.',  0,   0},
  {'Q', '-', '-', '.', '-',  0,   0},
  {'R', '.', '-', '.',  0,   0,   0},
  {'S', '.', '.', '.',  0,   0,   0},
  {'T', '-',  0,   0,   0,   0,   0},
  {'U', '.', '.', '-',  0,   0,   0},
  {'V', '.', '.', '.', '-',  0,   0},
  {'W', '.', '-', '-',  0,   0,   0},
  {'X', '-', '.', '.', '-',  0,   0},
  {'Y', '-', '.', '-', '-',  0,   0},
  {'Z', '-', '-', '.', '.',  0,   0},
  {'0', '-', '-', '-', '-', '-',  0},
  {'1', '.', '-', '-', '-', '-',  0},
  {'2', '.', '.', '-', '-', '-',  0},
  {'3', '.', '.', '.', '-', '-',  0},
  {'4', '.', '.', '.', '.', '-',  0},
  {'5', '.', '.', '.', '.', '.',  0},
  {'6', '-', '.', '.', '.', '.',  0},
  {'7', '-', '-', '.', '.', '.',  0},
  {'8', '-', '-', '-', '.', '.',  0},
  {'9', '-', '-', '-', '-', '.',  0},
  {'/', '-', '.', '.', '-', '.',  0},
  {'?', '.', '.', '-', '-', '.', '.'},
  {'.', '.', '-', '.', '-', '.', '-'},
  {',', '-', '-', '.', '.', '-', '-'}
};

String morse_msg[] = {"CQ SOTA VK3HN/P K", "CQ VK3HN/P K", "VK3HN/P K" };
//String morse_msg[] = {"CQ SOTA VK3HN/P K", "VK3HN ", " RST 599 5NN BK" };

byte   curr_msg_nbr;   // index into morse_msg[] array
byte   cw_msg_index;   // index into morse_msg[cw_msg_index] array
#endif
// end CW Keyer block -------------------------------------------------------------


byte curr_line = 0;    // the currently selected filter control line

// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets
byte v;                       // index into VFOSet array (representing the current VFO)
byte v_prev;


Si5351 si5351;                // I2C address defaults to x60 in the NT7S lib
Rotary r = Rotary(ENCODER_A, ENCODER_B);

// variables for transmit-receive control 
bool mode_tx = false; 
bool mode_cw = false; 

// button variables used in main loop for sensing the multiplexed buttons
byte button_nbr; 
byte old_button_nbr = 0; 

bool func_button_pressed = false; // if true, the next button pressed is interpreted as a Function 

// variables for controlling EEPROM writes
unsigned long last_freq_change_ms;
bool eeprom_written_since_last_freq_change; 
bool changed_f = false;


/**************************************/
/* Interrupt service routine for encoder frequency change */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);
}

/**************************************/
/* Change frequency; dir=1 Increment; dir=-1 Decrement
/**************************************/
void set_frequency(short dir)
{
  if(mode_tx) return;  // dial locks in transmit

  if (dir == 1)
  {
      VFOSet[v].vfo += VFOSet[v].radix;
  }
  else 
  {
     if (dir == -1)
       VFOSet[v].vfo -= VFOSet[v].radix; 
  };
  changed_f = 1;
};


int read_analogue_pin(byte p)
{
// Take an averaged reading of analogue pin 'p'  
  int i, val=0, nbr_reads=2; 
  for (i=0; i<nbr_reads; i++)
  {
    val += analogRead(p);
    delay(1); 
  }
  return val/nbr_reads; 
};


#ifdef CW_KEYER
int read_keyer_speed()
{ 
  int n = read_analogue_pin((byte)PIN_KEYER_SPEED);
  //Serial.print("Speed returned=");
  //Serial.println(n);
  dot_length_ms = 60 + (n-183)/5;   // scale to wpm (10 wpm == 60mS dot length)
                                     // '511' should be mid point of returned range
                                     // change '5' to widen/narrow speed range...
                                     // smaller number -> greater range  
  return n;
};
#endif



byte get_front_panel_button()
// Take a reading of the front panel buttons and map it to a button number (0..4)
// Take multiple readings and average them
{
  byte b=0; 
  int z;
  z = read_analogue_pin((byte)SWITCH_BANK);
//  Serial.print("Frnt bttn="); Serial.println(z);


  if     (z > 1021)               b = 0;  // 1023
  else if(z >= 1000 && z <= 1021) b = 3;  // 1015-1021
  else if(z > 940   && z < 1000)  b = 1;  // 966-973
  else if(z >= 895   && z < 940)  b = 6;  // 910
  else if(z > 800   && z < 895)   b = 2;  // 880-886
  else if(z > 700   && z < 800)   b = 5;  // 737 
  else if(z > 400   && z < 480)   b = 4;  // 444 

//  if(b>0){ Serial.print("Front button="); Serial.print(b); Serial.print("   z="); Serial.println(z);}

  b=0;  // temp!!!!!!!!!!!!!!!!!!!!!!!!!!
  return b;  
}


void LCD_diagnostic(char c)
{
  lcd.setCursor(10, 1);
  lcd.print(c);
}


void refresh_LCD() {
// Update the LCD  
  uint16_t f_MHz, f_kHz, f_Hz, f;
  uint32_t vfo_l; 

  lcd.setCursor(0, 0);
  vfo_l = VFOSet[v].vfo;
  lcd.print(char(v + 65));  // convert 0..2 to ASCII 'A' .. 'C'

  lcd.setCursor(2, 0);
  f = vfo_l / 1000000;   
  if(f<900) lcd.print(' ');
  if(f<10) lcd.print(' ');
  lcd.print(f);
  lcd.print(',');

  f_MHz = (vfo_l % 1000000) / 1000;
  if (f_MHz < 100) lcd.print('0');
  if (f_MHz < 10) lcd.print('0');
  lcd.print(f_MHz);
  lcd.print(',');
  
  f_kHz = vfo_l % 1000;
  if (f_kHz < 100) lcd.print('0');
  if (f_kHz < 10) lcd.print('0');
  lcd.print(f_kHz);
  lcd.print('.');

  f_Hz = vfo_l % 10;
  if (f_Hz < 10) lcd.print('0');
  lcd.print(f_Hz);
 
//  lcd.print(" ");

  lcd.setCursor(15, 1);
  if(func_button_pressed) lcd.print("F");    
  else lcd.print(" ");

// line 2
  lcd.setCursor(0, 1);  // start of line 2
  if(message_playing)
  {
#ifdef CW_KEYER
    // write the canned CW message across line 2...
    byte first=0, last=cw_msg_index; 
    if(last>=16) first= last%16 + 1; // implement scrolling
    lcd.print( (morse_msg[curr_msg_nbr-1]).substring(first,last+1) );
#endif 
  }
  else
  {
    // display s-meter in receiver or pwr meter in transmit 
    if(!mode_tx) {
      lcd.print("Line 2         .");
      lcd.print("     "); // clear the rest of line 2 (replace this if right half of line 2 gets used for something) 
    }
    else
    {
      // transmit line 2, in modes other than keyer message playing
      // placeholder for the relative power meter
    }
    
  }

  byte cursor_pos = 14;  
  switch (VFOSet[v].radix)
  {
    case 10:
      lcd.setCursor(cursor_pos, 0);
    break;
      
    case 100:
      lcd.setCursor(cursor_pos-1, 0);
    break;
      
    case 1000:
      lcd.setCursor(cursor_pos-3, 0);
    break;
      
    case 10000:
      lcd.setCursor(cursor_pos-4, 0);
    break;
  };
}



void update_eeprom()
{
  if(abs( millis() - last_freq_change_ms ) > 10000)
  {
    if(eeprom_written_since_last_freq_change == false)
    {
      // do the eeprom write
      // Serial.println("*** eeprom write");
      EEPROM.write(0, v);   // write the band index (v) to the first byte
        
      int element_len = sizeof(VFOset_type);
      for(int i=0; i<NBR_VFOS ; i++)    // write each element of the VFOSet array
      {
        EEPROM.put(1 + (i * element_len), VFOSet[i]);
      }
      eeprom_written_since_last_freq_change = true;
    }
  }
};



void tune()
// puts the carrier on, for TUNE_MS milliseconds
// NOTE - this function relies on some functions inside the CW_KEYER block -- factor these out!
{
  if(!mode_tx)
  {
    // prime CLK1 to the current frequency 
     si5351.set_freq(VFOSet[v].vfo * SI5351_FREQ_MULT, (si5351_clock)v);
     si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); 
     si5351.output_enable(SI5351_CLK1, 0); // turn the CW clock off until keyed
     
     tone(PIN_TONE_OUT, CW_TONE_HZ);
     Serial.println("Tune -- Carrier on"); 
     set_key_state2('D');  
     refresh_LCD();

     delay(TUNE_MS);  // key down for this number of mS
     Serial.println("Tune -- Carrier off"); 
     set_key_state2('U');  

     noTone(PIN_TONE_OUT);
     
  }
  mode_tune = false; 
}


void set_key_state2(char k)
{
// push the morse key down, or let it spring back up
// changes global 'key_state' (bool)

  if(!key_down and k=='D')
  {
    // do whatever you need to key the transmitter
    digitalWrite(13, 1);  // turn the Nano's LED on
    si5351.output_enable(SI5351_CLK1, 1); // turn the (CW clock) on
    key_down = true; 
  };

  if(key_down and k=='U')
  {
    // do whatever you need to un-key the transmitter 
    digitalWrite(13, 0);  // turn the Nano's LED off
    si5351.output_enable(SI5351_CLK1, 0); // turn of (CW freq) clock off

    char_sent_ms = millis();
    space_inserted = false;
    key_down = false; 
  };
}  



#ifdef CW_KEYER
// start of CW Keyer block -------------------------------------------------------------

int morse_lookup(char c)
// returns the index of parameter 'c' in MorseCode array, or -1 if not found
{
  for(int i=0; i<sizeof(MorseCode); i++)
  {
    if(c == MorseCode[i].ch[0])
      return i;
  }
  return -1; 
};


byte check_keyer_pushbutton()
{
// Reads the keyer pushbuttons and returns the button number as a byte; 0 if not pressed  
  byte b=0; 
  int  z;
  z = read_analogue_pin(PIN_PUSHBTTN_REAR);    // read the analog pin

// Serial.print("Kyr pshbtn z="); Serial.println(z);

  // reading is from the rear keyer pushbuttons  

                                      // open (USB power: 840) (LiFePO+7812: 1023)  
  if(z > 300 && z < 990) b = 1;       // L    (USB power: 418) (LiFePO+7812: 712)  
  else if(z > 10 && z <= 300) b = 2;  // R    (USB power:  74) (LiFePO+7812: 122)  
                                      // both:(USB power:  66) (LiFePO+7812: 112)  
  if(b>0){
      Serial.print("Keyer pushbutton="); Serial.print(b); Serial.print(", z="); Serial.println(z);
    }
  return b;
}


byte check_paddle()
{
// Reads the paddle, returns the paddle number as a byte; 0 if not pressed  
  byte b=0; 
  int  z=0;

  z = read_analogue_pin(PIN_PADDLE);    // read the analog pin
  
// Serial.print("Kyr pdl, z="); Serial.println(z);

  if(z < 30) b = 2;        // L 14
  else if(z > 30 && z < 100) b = 1; // R 110 

  if(b>0){
//     Serial.print("Kyr pdl, b="); Serial.print(b); 
//     Serial.print(", z="); Serial.println(z); 
  }
  return b;
};


void activate_state2(char c)
{
// if necessary, activates the receiver or the transmitter 
// 'c' may be either 'T' or 'R' 

  // in  all other cases, do nothing!
}


void send_dot()
{
  delay(dot_length_ms);  // wait for one dot period (space)

  // send a dot and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print(".");
  delay(dot_length_ms);  // key down for one dot period
  noTone(PIN_TONE_OUT);
  
  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_dash()
{
  delay(dot_length_ms);  // wait for one dot period (space)
  // send a dash and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print("-");
  delay(dot_length_ms * CW_DASH_LEN);  // key down for CW_DASH_LEN dot periods
  noTone(PIN_TONE_OUT);

  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_letter_space()
{
  delay(dot_length_ms * 4);  // wait for 3 dot periods
  Serial.print(" ");
}

void send_word_space()
{
  delay(dot_length_ms * 7);  // wait for 6 dot periods
  Serial.print("  ");
}

void send_morse_char(char c)
{
  // 'c' is a '.' or '-' char, so send it 
  if(c == '.') send_dot();
  else if (c == '-') send_dash();
  // ignore anything else, including 0s
}

void play_message(String m, int s)
{
// sends the message in string 'm' as CW, with inter letter and word spacing
// s is the speed to play at; if s == 0, use the current speed  
  byte j, n, old_s; 
  char buff[30];   // magic number, should guard this!

  message_playing = true; 
  Serial.println(m);
//  Serial.println(m.length());

  // use ch = m.charAt(index);
  m.toCharArray(buff, m.length()+1);

  if(s > 0)  // caller has passed in a speed to send message at 
  {
    old_s = dot_length_ms; // preserve the current keyer speed
    dot_length_ms = s;
  }
//  Serial.print("play_message()::dot_length_ms:");   Serial.println(dot_length_ms);

  //activate_state2('T'); 

  for (cw_msg_index=0; cw_msg_index<m.length(); cw_msg_index++)
  {
    if(buff[cw_msg_index] == ' ') 
    {
       send_word_space(); 
    }
    else
    {
      if( (n = morse_lookup(buff[cw_msg_index])) == -1 )
      {
        // char not found, ignore it (but report it on Serial)
//        Serial.print("!! not found <");
//        Serial.print(buff[cw_msg_index]);
//        Serial.println(">");
      }
      else
      {
        // char found, so send it as dots and dashes
        // Serial.println(n);
        refresh_LCD();         // first, refresh the LCD so the current CW char gets displayed

        for(j=1; j<7; j++)
          send_morse_char(MorseCode[n].ch[j]);
        send_letter_space();  // send an inter-letter space
        // if(s==0)read_keyer_speed();  // see if speed has changed mid message ***
      } // else
    } // else 
  } // for  
  Serial.println();
  if(s > 0)  // reset speed back to what it was  
    dot_length_ms = old_s;

  //activate_state2('R'); 
  message_playing = false; 
  curr_msg_nbr = 0;
  cw_msg_index = 0;
} // play_message
// end CW Keyer block -------------------------------------------------------------
#endif


void setup()
{
  Serial.begin(9600);  
  Wire.begin();
  
  Serial.println("si5351 Signal Generator by VK3HN");
  lcd.begin(16, 2);   
  lcd.cursor();
  lcd.noBlink();   
  lcd.setCursor(0,0);
  lcd.print("si5351 SigGen");
  lcd.setCursor(0,1);
  lcd.print("VK3HN 12/07/2020");
  delay(2000); 
  lcd.clear();

// set digital and analogue pins
  pinMode(SWITCH_BANK, INPUT_PULLUP); // switch bank is Pull-up

// start with transmit line low (in receive) 
  mode_tx = false;
  
  PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  
  // load up VFOSet array from EEPROM

  v = EEPROM.read(0);
  Serial.print("setup() eeprom: v=");
  Serial.println(v);
  if(v >= NBR_VFOS) v = 1;  // in case NBR_VFOS has been reduced since the last run (EEPROM update)
  v_prev = v;
  
  int element_len = sizeof(VFOset_type);
  for(int i=0; i < NBR_VFOS; i++)
  {
    EEPROM.get(1 + (i * element_len), VFOSet[i]);
  };

/* // initialise VFOSet array
  for(int n=0; n<NBR_VFOS; n++) VFOSet[n].active = 0;   // make sure all are inactive to start with 
  VFOSet[0] = (VFOset_type){1,  1825000ULL, 1000};
  VFOSet[1] = (VFOset_type){1,  3525000ULL, 1000};
  VFOSet[2] = (VFOset_type){1,  3625000ULL, 1000};
  VFOSet[3] = (VFOset_type){1,  7025000ULL,  100};
  VFOSet[4] = (VFOset_type){1,  7090000ULL, 1000};
  VFOSet[5] = (VFOset_type){1,  7125000ULL, 1000};
  VFOSet[6] = (VFOset_type){1, 10105000ULL,  100};
  VFOSet[7] = (VFOset_type){1, 14060000ULL, 1000};
  VFOSet[8] = (VFOset_type){1, 18068000ULL, 1000};
  VFOSet[9] = (VFOset_type){1, 18068000ULL, 1000};
*/
// dump out the VFOset array for diagnostics
  for(int n=0; n < NBR_VFOS ; n++)
  {
    Serial.print((int)VFOSet[n].active);
    Serial.print(" ");
    Serial.print(VFOSet[n].vfo);
    Serial.print(" ");
    Serial.print((long)VFOSet[n].radix);
    Serial.println();
  }
  
// initialise and start the si5351 clocks

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); // If using 27Mhz xtal, put 27000000 instead of 0 (0 is the default xtal freq of 25Mhz)
  
  si5351.set_correction(15500);    // Library update 26/4/2020: requires destination register address  ... si5351.set_correction(19100, SI5351_PLL_INPUT_XO);
                                      
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
// turn on the oscillators

  for(byte j=0; j<3; j++){
    Serial.print("Osc "); Serial.print(j); Serial.print(" "); Serial.println(VFOSet[j].vfo);  
    si5351.set_freq(VFOSet[j].vfo * SI5351_FREQ_MULT, (si5351_clock)j); 
    si5351.drive_strength((si5351_clock)j, SI5351_DRIVE_2MA); 
    if(VFOSet[j].active) 
      si5351.output_enable((si5351_clock)j, 1);  
    else 
      si5351.output_enable((si5351_clock)j, 0);  
  }
  
  changed_f = true; 
  last_freq_change_ms = millis(); 
  eeprom_written_since_last_freq_change = false; 

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
  key_down = false;  
  char_sent_ms = millis();
  space_inserted = false;
  message_playing = false;
  curr_msg_nbr = 0;
  cw_msg_index = 0;
  dot_dash_counter = 0;

  // dump out the MorseCode table for diagnostic
/*  for(int i=0; i<40; i++)
  {
    Serial.print(MorseCode[i].ch[0]);
    Serial.print(' ');
    for(int j=1; j<7; j++)
      Serial.print(MorseCode[i].ch[j]);
    Serial.println();
  }

  // play the two messages as a diagnostic
  //  play_message(morse_msg[0], 0);
  //  delay(2000);
  //  play_message(morse_msg[1], 0);
  //  delay(2000);
  */
#endif
// end of CW Keyer block -------------------------------------------------------------

}


void loop()
{
  update_eeprom(); 
  refresh_LCD();  
   
  // Update the display if the frequency has been changed (and we are not transmitting)
  if (!mode_tx and changed_f)
  {
    volatile uint32_t f; 
    f = VFOSet[v].vfo; 
    si5351.set_freq(f * SI5351_FREQ_MULT, (si5351_clock)v);  
    
    //refresh LCD 
    changed_f = 0;
    last_freq_change_ms = millis(); 
    eeprom_written_since_last_freq_change = false;
  } // endif changed_f

  //------------------------------------------------------------------------
  // if any of the buttons have been pressed...
  old_button_nbr = (byte)0;  // clear the last command 
  bool button_held = false; 
  unsigned long button_pressed_ms = millis();
  
  button_nbr = get_front_panel_button();
  byte first_button_nbr = 0;
  while (button_nbr > 0)
  {
  // one of the multiplexed switches is being held down
    delay(5);  // was 20
    if(first_button_nbr == 0) first_button_nbr = button_nbr;
    old_button_nbr = button_nbr;
    button_nbr = get_front_panel_button();
  }

 //button_nbr = old_button_nbr;  // accepts the final reading, which can be spurious due to button release
  button_nbr = first_button_nbr; // experiment to accept the first reading
  
  if((millis() - button_pressed_ms) >= BUTTON_HELD_MS) button_held = true;
    
 // if one of the buttons was pressed (and is now released) act on it...
 
  if (button_nbr == 1)
  {
      if(!func_button_pressed)
      {
          Serial.println("<B1>VFO down");
          if(v == 0) v = (NBR_VFOS-1);
          else v--;
      }
      else
      {
          Serial.println("<F><B1>N/A");
          func_button_pressed = false;
      }
    changed_f = 1;
  }

  if (button_nbr == 2) 
  {
    // Button 2:  
  }; 

  if (button_nbr == 3) {};  

  if (button_nbr == 4)
  {
      if(!func_button_pressed)
      {
             if(button_held) 
             {
               Serial.println("<B4>held -- Tune !"); 
               button_held = false;
               mode_tune = true;
               tune();               
             }
             else
             {
               Serial.println("<B4>VFO up");
               if(v == (NBR_VFOS-1)) v = 0;
               else v++;
             }
      }
      else
      {

       }
    changed_f = 1;
  }

  if (button_nbr == 5) 
  {
    Serial.println("<B5>Fn tgl");
    func_button_pressed = !func_button_pressed;   
    if(func_button_pressed) Serial.println("Function...");
    changed_f = 1;
  }

  if (button_nbr == 6)
  // Button 6: change frequency step up
  // Fn: tbd
  {
    if(!func_button_pressed)
       Serial.println("<B6>f step r");
    else
       Serial.println("<F><B6>f step l");

    if(button_held)
    {
      Serial.println("<B6>held -- toggle IF filters"); 
    }
    else
    {

      // default radfix increment/decrement behaviour...
      switch (VFOSet[v].radix)
      {
        case 10:
        {
            if(!func_button_pressed)
            {
                // change radix up
                VFOSet[v].radix = 10000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
            else
            {
               func_button_pressed = false;
               // change radix down
               VFOSet[v].radix = 100;
               // clear residual < 100Hz frequency component from the active VFO         
               uint16_t f = VFOSet[v].vfo % 100;
               VFOSet[v].vfo -= f;
            }
        }
        break;
  
        case 100:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 10;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 1000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
        }
        break;
        
        case 1000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 100;
            }
            else
            {
               func_button_pressed = false;
               VFOSet[v].radix = 10000;            
            }
            break;
        }
      
        case 10000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 1000;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 10;
            }
            break;
        }
      }
    } // else
    changed_f = 1;
  }

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
//  read_keyer_speed(); // read the speed control 
// see if a memory button has been pushed
  if((curr_msg_nbr = check_keyer_pushbutton()) > 0)
  {
    mode_cw = true; 
    activate_state2('T'); 
    refresh_LCD();  // to show 'CW' on the display  
    byte msg_speed=0;
    play_message(morse_msg[curr_msg_nbr-1], msg_speed);  
    delay(5); 
    activate_state2('R');  
  };

  // see if the paddle is being pressed
  byte j = check_paddle();
  if((j==PADDLE_L) or (j==PADDLE_R)) 
  {
    mode_cw = true;
    activate_state2('T'); 
    if(j==PADDLE_L) send_dot();
    if(j==PADDLE_R) send_dash();
 //   Serial.print("loop()::dot_length_ms:");   Serial.println(dot_length_ms);
  };

  curr_ms = millis();
  // if paddle has been idle for BREAK_IN_DELAY drop out of transmit 
  if(mode_cw and mode_tx and ((curr_ms - char_sent_ms) > BREAK_IN_DELAY))
  {
//    Serial.print("curr_ms="); Serial.print(curr_ms); 
//    Serial.print("  char_sent_ms="); Serial.print(char_sent_ms); 
//    Serial.print("  curr_ms - char_sent_ms="); Serial.println(curr_ms - char_sent_ms); 
    // drop back to receive to implement break-in
    activate_state2('R');  
  }
// end of CW Keyer block -------------------------------------------------------------
#endif

}
