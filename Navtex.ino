/*
 * Simple Navtex receiver - https://youtu.be/SwL_ZQ_iBIM
 *
 * Initial port by Martin Kuettner <berry@fmode.de> 03/2016
 * Revisions 2021, rszemeti
 *
 *
 * port of the 80C51 Assembler Program from "A NAVTEX Receiver for the DXer":
 *
 * Klaus Betke, Am Entengrund 7, D-26160 Bad Zwischenahn, Email: betke@itap.de
 * 11-AUG-00/01-OCT-00
   
 *
 * The port in C can be run on an ATmega32U4 (Arduino Mirco). 
 * Other Arduino models, which are based on the ATmega32U4 should also be able to process the program.
 * Please pay attention to any other configuration of the output pins.
 *
 * Development environment: Arduino v1.67
 *
 * Removes: clock display
 * Since the ATmega32U4 has no real-time clock and the evaluation happens on a Raspberry PI with GPS module
 * , no clock on the ATmega32U4 is required.
 *
 * The program is optimizable, I have written it so that I have the source code too
 * understand without comments. Therefore, it was also written in some places (if ((byte & 0b111100)> 0))
 * or on shortening, like i + = 5 omitted!
 *
 * Who would like to optimize all this :)
 *
 */


// to copy and paste for debugging
/*
  digitalWrite (DebugPin, HIGH);
  delay microseconds (100);
  digitalWrite (DebugPin, LOW);
*/

/*
    tobinstr (sirawdata, 8, UART);
    sprintf (UART, "% s \ n", UART);
    Serial.write (UART);
*/

#include <EEPROM.h>

// pin definitions
const byte DebugPin = 13;       // Pin with built-in LED on the Arduino
const byte DataPin = 7;         // Data pin from the receiver
const byte ErrorLEDPin = 12;    // if error is set
const byte SyncLEDPin = 11;     // on data received
const byte Do518kHzLEDPin = 10; // when received at 518kHz (default)
const byte Do518kHzSetPin = 8;  // output for divider after the PLL
const byte DataLEDPin = 9;      // Data LED .. to blink ...

const byte UpperLowerInPin = 4; // Input pin for output large or small
const byte FrequencyInPin = 6;  // Input pin switching 490kHz / 518kHz

// data port - internal register
bool InPort; // internal data variable for transfer

// workers
byte AA; // work register for external requests
unsigned int AB; // work register for DataPin interrupt
byte AC; // working register for bitholes in buffer
byte AD; // working register for bit shifting for byte fetching
bool debug = LOW;
bool FF = LOW;
bool InWait;

// Sync & Clock
byte loopadjust = 255; // Counter to clock slide-trigger
byte loopgain = 128;   // Regulator, how often should be readjusted
byte clock3200 = 0b00100000; // clock for 10ms clock (3,2kHz / 32)

// run variables
byte i; // Runner for loops

// Dates
byte sirawdata; // raw data fetched in timer interrupt
byte bitsavailable; // Count how many bits are available (max 8)
byte error; // error counter for reception
byte RingBuffer [3]; // 3-byte ring buffer for error pre-correction
byte RXByte; // Received byte on the RX channel
byte CharRingBuffer; // character, which comes from the ring buffer (for error correction)
byte CharRX; // character, which was received
char ReceivedChar; // sign, what is output to the UART

// constants, control characters from the SITOR code (everything where "1" is in the LUT)
// these characters are not output, are for internal control only
const byte ALPHA = 0x0F;
const byte ALPHAUP = 0x7F;
const byte REP = 0x66;
const byte LRTS = 0x5A;
const byte FIGS = 0x36;
const byte LF = 0x6c;
const byte CR = 0x78;
const byte CHAR32 = 0x6A;
const byte SPACE = 0x5C;
const byte BETA = 0x33;
const byte BEL = 0x17;

const char errsymbol = '~'; // error sign ("0" in the LUT)

// status bits
bool Shifted = LOW; // use numbers or letters LUT?
bool Frequency = HIGH; // Abort variable when frequency is switched
bool UpperLower = HIGH; // Abort variable when LUT is toggled
bool sync = LOW; //  whether valid sequence ALPHA-REP-ALHPA was received

bool NotLostSync = HIGH;

// UART buffer
char UART[255]; // output buffer variable for UART

// There are different mappings - this is the European variant (except 0xd3 - I left that $)
// LUT - in lowercase letters
static const uint8_t lutLOWER[256] =
//0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f
{
  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1, // 0
  0,    0,    0,    0,    0,    0,    0,  'j',    0,    0,    0,  'f',    0,  'c',  'k',    0, // 1
  0,    0,    0,    0,    0,    0,    0,  'w',    0,    0,    0,  'y',    0,  'p',  'q',    0, // 2
  0,    0,    0,    1,    0,  'g',    1,    0,    0,  'm',  'x',    0,  'v',    0,    0,    0, // 3
  0,    0,    0,    0,    0,    0,    0,  'a',    0,    0,    0,  's',    0,  'i',  'u',    0, // 4
  0,    0,    0,  'd',    0,  'r',  'e',    0,    0,  'n',    1,    0,  ' ',    0,    0,    0, // 5
  0,    0,    0,  'z',    0,  'l',    1,    0,    0,  'h',    1,    0,    1,    0,    0,    0, // 6
  0,  'o',  'b',    0,  't',    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0, // 7
  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1, // 8
  0,    0,    0,    0,    0,    0,    0,    1,    0,    0,    0,  '!',    0,  ':',  '(',    0, // 9
  0,    0,    0,    0,    0,    0,    0,  '2',    0,    0,    0,  '6',    0,  '0',  '1',    0, // a
  0,    0,    0,    1,    0,  '&',    1,    0,    0,  '.',  '/',    0,  '=',    0,    0,    0, // b
  0,    0,    0,    0,    0,    0,    0,  '-',    0,    0,    0, '\'',    0,  '8',  '7',    0, // c
  0,    0,    0,  '$',    0,  '4',  '3',    0,    0,  ',',    1,    0,  ' ',    0,    0,    0, // d
  0,    0,    0,  '+',    0,  ')',    1,    0,    0,  '#',    1,    0,    1,    0,    0,    0, // e
  0,  '9',  '?',    0,  '5',    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0  // f
};
// LUT - in Grossbuchstaben
static const uint8_t lutUPPER[256] =
//0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f
{
  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1, // 0
  0,    0,    0,    0,    0,    0,    0,  'J',    0,    0,    0,  'F',    0,  'C',  'K',    0, // 1
  0,    0,    0,    0,    0,    0,    0,  'W',    0,    0,    0,  'Y',    0,  'P',  'Q',    0, // 2
  0,    0,    0,    1,    0,  'G',    1,    0,    0,  'M',  'X',    0,  'V',    0,    0,    0, // 3
  0,    0,    0,    0,    0,    0,    0,  'A',    0,    0,    0,  'S',    0,  'I',  'U',    0, // 4
  0,    0,    0,  'D',    0,  'R',  'E',    0,    0,  'N',    1,    0,  ' ',    0,    0,    0, // 5
  0,    0,    0,  'Z',    0,  'L',    1,    0,    0,  'H',    1,    0,    1,    0,    0,    0, // 6
  0,  'O',  'B',    0,  'T',    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0, // 7
  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1, // 8
  0,    0,    0,    0,    0,    0,    0,    1,    0,    0,    0,  '!',    0,  ':',  '(',    0, // 9
  0,    0,    0,    0,    0,    0,    0,  '2',    0,    0,    0,  '6',    0,  '0',  '1',    0, // a
  0,    0,    0,    1,    0,  '&',    1,    0,    0,  '.',  '/',    0,  '=',    0,    0,    0, // b
  0,    0,    0,    0,    0,    0,    0,  '-',    0,    0,    0, '\'',    0,  '8',  '7',    0, // c
  0,    0,    0,  '$',    0,  '4',  '3',    0,    0,  ',',    1,    0,  ' ',    0,    0,    0, // d
  0,    0,    0,  '+',    0,  ')',    1,    0,    0,  '#',    1,    0,    1,    0,    0,    0, // e
  0,  '9',  '?',    0,  '5',    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0  // f
};
// Initial setup of the registers
void setup () {
  // Pin as interrupt, falling edge on the data line
  attachInterrupt (digitalPinToInterrupt (DataPin), EventOnInput, FALLING);

  // pin 13 (the one with LED) as output (for debug)
  pinMode (DebugPin, OUTPUT);

  // 2 input pins for frequency and character set
  pinMode (UpperLowerInPin, INPUT);
  pinMode (FrequencyInPin, INPUT);

  // configure further pins as output 
  pinMode (ErrorLEDPin, OUTPUT);
  pinMode (SyncLEDPin, OUTPUT);
  pinMode (Do518kHzLEDPin, OUTPUT);
  pinMode (Do518kHzSetPin, OUTPUT);
  pinMode (DataLEDPin, OUTPUT);

  // boot sequence ...
  for (i = 0; i <5; i ++) {
    digitalWrite (Do518kHzLEDPin, HIGH);
    delay (50);
    digitalWrite (DataLEDPin, HIGH);
    delay (50);
    digitalWrite (SyncLEDPin, HIGH);
    delay (50);
    digitalWrite (ErrorLEDPin, HIGH);
    delay (50);
    digitalWrite (Do518kHzLEDPin, LOW);
    delay (50);
    digitalWrite (DataLEDPin, LOW);
    delay (50);
    digitalWrite (SyncLEDPin, LOW);
    delay (50);
    digitalWrite (ErrorLEDPin, LOW);
    delay (50);
  }

  // UART - 9600 BAUD
  Serial.begin(9600);


  // get last settings from the EEProm
  Frequency = EEPROM.read (0);
  UpperLower = EEPROM.read (1);

  if (UpperLower == LOW) {
    Serial.write ( "--- Lower Cased ---\n");
  } else {
    Serial.write ( "--- Upper Cased --- \n");
  }
  if (Frequency == LOW) {  
    Serial.write ( "--- 490 kHz --- \n");
    digitalWrite (Do518kHzLEDPin, LOW);
    digitalWrite (Do518kHzSetPin, LOW);
  } else {
    Serial.write ( "--- 518kHz --- \n");
    digitalWrite (Do518kHzLEDPin, HIGH);
    digitalWrite (Do518kHzSetPin, HIGH);
  }

  // Interrupt Timer Setup
  DDRC |= (bit (7) | bit (6));
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10); // timer 1 without prescale (runs at 16MHz)
  OCR1A = 5000; // Trigger after 5000 clock cycles (ie all 312.5us | 16MHz) interrupt
  TIMSK1 |= (1 << OCIE1A);             
}


// main program
void loop () {

  Serial.write ("--- Loop Start --- \n");

  // if change to frequency or character set pin 
  if (AA> 0) {
    if ((AA & 0b01) == 0b01) {// Switch character set
      if (UpperLower == LOW) {
        Serial.write ( "--- Lower Cased \n");
        EEPROM.write (1, 0);
      } else {
        Serial.write ( "--- Upper Cased --- \n");
        EEPROM.write (1, 1);
      }
    }
    if ((AA & 0b10) == 0b10) {// switch to the frequency to be received
      if (Frequency == LOW) {  
        Serial.write ( "--- 490 kHz --- \n");
        digitalWrite (Do518kHzLEDPin, LOW);
        digitalWrite (Do518kHzSetPin, LOW);
        EEPROM.write (0, 0);
      } else {
        Serial.write ( "--- 518kHz --- \n");
        digitalWrite (Do518kHzLEDPin, HIGH);
        digitalWrite (Do518kHzSetPin, HIGH);
        EEPROM.write (0, 1);
      }
      NotLostSync = HIGH; // Prevent Error LED
    }
    AA = 0;
  }
  
  /* 
   * Error LED switch if no EOT was received.
   * stays on until the next sync is reached. 
   * maybe rebuild and run over bitclock an int16 counter, 
   * then turns off the LED after a while ...
  */
     
  if ((error> 16) || (NotLostSync == LOW)) {
    digitalWrite (ErrorLEDPin, HIGH);
  } else {
    digitalWrite (ErrorLEDPin, LOW);
  }
  digitalWrite (DataLEDPin, LOW);

  loopgain = 96; // as long as no sync - often readjust!
  loopadjust = 255; // Reset counter
  Shifted = LOW; // Set to Letters LUT
  NotLostSync = LOW;
  sync = LOW; // no sync
  InWait = HIGH;
  while (sync != HIGH) {// uC stays in this loop until ALPHA-REP-ALPHA comes
    while (GetOneBit () != ALPHA) {// Bitwise to see if ALPHA was received
      CheckInput(); // Check if something has changed on charset / frequency pin
      
      if (AA> 0) {
        break;
      }
    }
    InWait = LOW;
    if (AA == 0) {// skip over if config should change!
      sync = HIGH; // sync high, turns low if the next characters do not match
      if (Get7Bits () != REP) {// pick 7 bits and check for REP
        sync = LOW;
      } else {
        RingBuffer [0] = REP; // If true, fill ring buffers
      }
      if (Get7Bits () != ALPHA) {// pick 7 bits and check for ALPHA
        sync = LOW;
      }
    }
    if (AA> 0) {// stop while config changes!
      break;
    }
  }

  if (AA == 0) {
    Serial.write ( "\n --- InSync --- \n"); // debug string
 
    loopgain = 8; // Sync da -> disable regulation
    error = 0; // error count to 0

    digitalWrite (ErrorLEDPin, LOW); // error off
    digitalWrite (SyncLEDPin, HIGH); // Sync LED on

    RingBuffer [1] = 0; // Empty the ring buffer [1] so that you can not cancel the old character transfer
 
  /*
    // debug - just output the bit strings - but the lower while part has to be commented out!
    tobinstr (CharRX, 7, Get7Bits (););
    sprintf (UART, "% s \ n", CharRX);
    Serial.write (UART);
    // debug the end
  */
 
    // always pick up 2 characters from here
    while (error <16) {// loop goes to logoff, or error> = 32
      RingBuffer [2] = RingBuffer [1];
      RingBuffer [1] = RingBuffer [0];
      RingBuffer [0] = Get7Bits (); // Move the ring buffer and fill it with a new character
      RXByte = Get7Bits (); // RX Fill buffer with new character
 
    /*
     * The reception works like this:
     * It is sent repeatedly offset by 3 characters.
     * Thus one can see that as 2 in one another multiplexed channels
     * If you break that up, it looks like this:
     * - start sequence -
     * ALPHA ALPHA ALPHA ALPHA KUTT _ IS   
     * REP REP REP KUTT _ IS
     * The lower channel is the ring buffer, the upper one is the receive channel
     * There is always a character from the top and bottom fetched and looked, which is 3 characters
     * was previously received on the other channel  
     */
   
      if (RXByte != ALPHA) {// As long as ALPHA is on the RXbyte, synchronization will still be sent
 
        ReceivedChar = ByteToASCII (); // convert bytes to char
        if ((ReceivedChar != 0) && (ReceivedChar != 1)) {// if valid ..
          Serial.write (ReceivedChar); // give it to UART

         if (FF == LOW) {
            digitalWrite (DataLEDPin, LOW);
            FF = HIGH;
         } else {
            digitalWrite (DataLEDPin, HIGH);
            FF = LOW;
         }    
        }
        if (RingBuffer [2] == ALPHA) {// Termination condition when transmission is over
          Serial.write ( "\n --- EOT --- \n");

          digitalWrite (ErrorLEDPin, LOW);
          NotLostSync = HIGH;
          error = 0;

          break;
        }
      }
    }
    Serial.write ( "\n --- OutSync --- \n");
    digitalWrite (SyncLEDPin, LOW);
    
  }
}


// Convert a byte to char incl. Error correction and evaluation of control characters
char ByteToASCII () {
  if (UpperLower == LOW) {
    CharRX = lutLOWER [RXByte | (Shifted << 7)]; // Convert RX buffers to char, taking into account the character set to be used
    CharRingBuffer = lutLOWER [RingBuffer [2] | (Shifted << 7)]; // Convert RingBuffer to Char observing the character set to be used  
  } else {
    CharRX = lutUPPER [RXByte | (Shifted << 7)]; // Convert RX buffers to char, taking into account the character set to be used
    CharRingBuffer = lutUPPER [RingBuffer [2] | (Shifted << 7)]; // Convert RingBuffer to Char observing the character set to be used  
  }
 
  if ((CharRX == 0) && (CharRingBuffer == 0)) {// both invalid: /
    error ++;
    return errsymbol; // raise error and return error
  } else if ((CharRX == 0) && (CharRingBuffer != 0)) {// RX invalid but RingBuffer OK
    CharRX = CharRingBuffer; // copy characters
    RXByte = RingBuffer [2]; // RXByte umkopieren for switch query ...
    if (error> 0) {
      error--; // decrement error counters by one
    }
  }
  if (CharRX == 1) {// if out of the LUT 1 = control character
    switch (RXByte) {
      case LRTS: // use characters
        Shifted = LOW;
        break;
      case FIGS: // use numbers (offset 0x80h)
        Shifted = HIGH;
        break;
      case BETA:
      case ALPHA:
      case ALPHAUP:
      case REP:
      case CR:
      case BEL:
        break; // Characters for output irrelevant
      case LF:
        return '\n'; // New line
        break;
      case CHAR32:
      case SPACE:
        return ' '; // spaces
      default:
        sprintf (UART, "Unknown: 0x% x \n", RXByte);
        Serial.write (UART);
        break; // character unknown = Error
      return 0;
    }
  } else {
    return CharRX;
  }
}


byte GetOneBit () {
  while (bitsavailable == 0) {
    delay (1);
  }
  if (AA == 0) {
    switch (bitsavailable) {// select characters according to the size of the buffer and return them
      case 7:
        AC = AC << 1;
        AC = AC | ((sirawdata & 0b00000001) << 6);
        bitsavailable--;
        break;
      case 6:
        AC = AC << 1;
        AC = AC | ((sirawdata & 0b00000010) << 5);
        bitsavailable--;
        break;
      case 5:
        AC = AC << 1;
        AC = AC | ((sirawdata & 0b00000100) << 4);
        bitsavailable--;
        break;
      case 4:
        AC = AC << 1;
        AC = AC | ((sirawdata & 0b00001000) << 3);
        bitsavailable--;
        break;
      case 3:
        AC = AC << 1;
        AC = AC | ((sirawdata & 0b00010000) << 2);
        bitsavailable--;
        break;
      case 2:
        AC = AC >> 1;
        AC = AC | ((sirawdata & 0b00100000) << 1);
        bitsavailable--;
        break;
      case 1:
        AC = AC >> 1;
        AC = AC | (sirawdata & 0b01000000);
        bitsavailable--;
        break;
      case 0:
        AC = 0;
        break;
      default:
        bitsavailable = 0; // if buffer overflow clears buffers and returns error on UART
        AC = 0;
        Serial.write("ERROR! Bitsavailable overflow! \n");
        break;
    }
    AC = AC & 0b01111111; // top BIT away-and-en, just in case it's set
    return AC;
  }
}

// Get 7 bits in one go and return
byte Get7Bits () {
  i = 7;
  AD = 0;
  while (i> 0) {
    AD = GetOneBit ();
    i--;
  }
  return AD;
}

// Interrupt if falling edge on DataPin
void EventOnInput ()
{
  AB = loopadjust + loopgain; // Add reinforcements
  loopadjust = AB & 0x00FF;   // delete upper 8 bits and write back (due to lack of a carry flag)
  if ((AB & 0xFF00)> 0) { // if over 255, then readjust, otherwise not
    if (clock3200 <16) { // if the clock is too slow
        clock3200 ++; // Precheck meter
    } else if (clock3200> 16) {
        clock3200--; // otherwise rules
    }
  }

}

// 3200kHz timer interrupt
ISR (TIMER1_COMPA_vect)
{
  clock3200--; // Decrease clock .. factor 32 is oversampled

  if (clock3200 == 0) {
    InPort = digitalRead (DataPin);

  /*
   * if at 490kHz, then 1 and 0 are reversed 
   * This is due to the reference frequency, which at 518kHz with 518.4kHz slightly over, and 
   * at 490kHz with 489.6kHz is slightly below the centre frequency.
   * This will invert the signal (which could be done by another LUT) 
   * also works with InPort = InPort ^ 1; ... what the hell ...
   */
    
    if (Frequency == LOW) {
      if (InPort == LOW) {
        InPort = HIGH;                   
      } else {
        InPort = LOW;                    
      }
    }
    
    clock3200 = 0b00100000; // Reset counter

    // Move data to ring buffer and increase counters. 8 bits can be stored temporarily (80ms)
    sirawdata = ((sirawdata >> 1) | (InPort << 7)) & 0b11111111;
    bitsavailable ++;
  }
}


// debug routine to convert INT to binaer-String
void tobinstr (int value, int bitsCount, char * output)
{
    int i;
    output [bitsCount] = '\ 0';
    for (i = bitsCount - 1; i >= 0; --i, value >>= 1)
    {
        output [i] = (value & 1) + '0';
    }
}

// check if something has happened on the frequency or character set pin
void CheckInput () {
  AA = 0;
  if (digitalRead (UpperLowerInPin)!= UpperLower) {
    UpperLower = digitalRead (UpperLowerInPin);
    AA = AA | 0b01;
  }
  if (digitalRead (FrequencyInPin)!= Frequency) {
    Frequency = digitalRead (FrequencyInPin);
    AA = AA | 0b10;
  }
}
