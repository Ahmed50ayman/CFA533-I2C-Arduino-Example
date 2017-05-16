//============================================================================
// This is an example use of the Crystalfontz CFA533 I2C 2x16 LCD with
// keypad and the CFA533 I2C Arduino library.
//
// http://www.crystalfontz.com
//
//
//============================================================================
/*

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>

*/
//============================================================================

#include <Wire.h>
#include <stdarg.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
//============================================================================
// ref http://playground.arduino.cc/Main/Printf
void SerPrintFF(const __FlashStringHelper *fmt, ... )
  {
  char
    tmp[128]; // resulting string limited to 128 chars
  va_list
    args;
  va_start(args, fmt );
  vsnprintf_P(tmp, 128, (const char *)fmt, args);
  va_end (args);
  Serial.print(tmp);
  }
//----------------------------------------------------------------------------
// ref http://scott.dd.com.au/wiki/Arduino_Static_Strings
void SerialPrint_P(const char flash_string[])
  {
  uint8_t
    c;
  for(;0x00 != (c = pgm_read_byte(flash_string)); flash_string++)
    {
    Serial.write(c);
    }
  }  
//============================================================================
int freeRam(void)
  {
  extern int
    __heap_start;
  extern int
    *__brkval; 
  int
    v;
  return((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval)); 
  }
//============================================================================
// Waits until a switch is closed before booting (debug)
#define led_pin 3
#define switch_pin 2
//----------------------------------------------------------------------------
void BootStall(void)
  {
  int
    count;
  //wait for switch to be pressed
  count=0;
  while(count<3)
    {
    if(digitalRead(switch_pin))
      {
      //released
      count=0;
      digitalWrite(led_pin, HIGH);
      delay(50);
      digitalWrite(led_pin, LOW);
      }
    else
      {
      //pressed
      count++;
      digitalWrite(led_pin, HIGH);
      }
    delay(50);
    }

  //wait for switch to be released
  count=0;
  while(count<3)
    {
    if(digitalRead(switch_pin))
      {
      //pressed
      count++;
      digitalWrite(led_pin, HIGH);
      }
    else
      {
      //released
      count=0;
      digitalWrite(led_pin, HIGH);
      delay(50);
      digitalWrite(led_pin, LOW);
      }
    delay(50);
    }
  }
//============================================================================
typedef struct
  {
  uint8_t   command;
  uint8_t   length;
  uint8_t   data[24];
  uint16_t  crc;
  } CFPacket_t;
//----------------------------------------------------------------------------
class CrystalfontzI2CPacketLCD
  {
  public:
    //vars
    //functions
    CrystalfontzI2CPacketLCD(uint8_t address);
    uint8_t sendPacket_getReply(
              CFPacket_t *packet_sent,
              CFPacket_t *packet_received,
              uint8_t print_errors);
    uint8_t dumpPacket(CFPacket_t *packet_to_send);
    uint8_t Search_I2C_Adresses(void);
    void Set_I2C_Adress(uint8_t address);
    void writeText(uint8_t x, uint8_t y, char *text, uint8_t length);
    void clearScreen(void);
    uint8_t getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases);
    void setUpBar(uint8_t spec_char,uint8_t vert_mask);
    void drawBar(uint8_t col,uint8_t row,uint8_t chars_wide,
                 uint8_t px_length,uint8_t spec_char_solid,
                 uint8_t spec_char_variable,uint8_t vert_mask);
  private:
    //vars
    uint8_t  i2c_address;
    //functions
    uint16_t CRC(uint8_t *ptr, uint16_t len);
  };
//----------------------------------------------------------------------------
CrystalfontzI2CPacketLCD::CrystalfontzI2CPacketLCD(uint8_t address)
  {
  i2c_address=address;
  }
//----------------------------------------------------------------------------
uint16_t CrystalfontzI2CPacketLCD::CRC(uint8_t *data, uint16_t length)
  {
  //calculate the CRC for the packet data
  uint16_t crc = 0xFFFF;
  while(length--)
    crc = _crc_ccitt_update(crc, *data++);
  return ~crc;
  }
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::sendPacket_getReply(
        CFPacket_t *packet_to_send,
        CFPacket_t *packet_received,
        uint8_t print_errors)
  {
  uint8_t
    bytes_received;
  uint8_t
    i;
  //Valid commands are from 0-35.
  //The maximum received length is known for each packet from the data sheet,
  //so this table will allow us to optimize read performance/minimize I2C
  //traffic by reading only the number of bytes that are significant.
  //0xFF is a magic value for invalid commands
  //Storing data in flash in the Arduino is really obtuse.
  static const uint8_t receive_packet_length[36] PROGMEM= {
    1+1+16+2,  //  0 = Ping Command (variable, 16 is max)
    1+1+16+2,  //  1 = Get Hardware & Firmware Version
    1+1+ 0+2,  //  2 = Write User Flash Area
    1+1+16+2,  //  3 = Read User Flash Area
    1+1+ 0+2,  //  4 = Store Current State As Boot State
    1+1+ 0+2,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
    1+1+ 0+2,  //  6 = Clear LCD Screen
    1+1+ 0+2,  //  7 = Set LCD Contents, Line 1
    1+1+ 0+2,  //  8 = Set LCD Contents, Line 2
    1+1+ 0+2,  //  9 = Set LCD Special Character Data
    1+1+ 9+2,  // 10 = Read 8 Bytes of LCD Memory
    1+1+ 0+2,  // 11 = Set LCD Cursor Position
    1+1+ 0+2,  // 12 = Set LCD Cursor Style
    1+1+ 0+2,  // 13 = Set LCD Contrast
    1+1+ 0+2,  // 14 = Set LCD & Keypad Backlight
    1+1+ 4+2,  // 15 = Read Temperature
    0xFF,      // 16 = (reserved)
    0xFF,      // 17 = (reserved)
    1+1+ 9+2,  // 18 = Read DOW Device Information
    0xFF,      // 19 = (reserved)
    1+1+16+2,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
    1+1+ 7+2,  // 21 = Setup Live Temperature Display (2 or max of 7)
    1+1+ 0+2,  // 22 = Send Command Directly to the LCD Controller
    0xFF,      // 23 = (reserved)
    1+1+ 3+2,  // 24 = Read Keypad, Polled Mode
    0xFF,      // 25 = (reserved)
    0xFF,      // 26 = (reserved)
    0xFF,      // 27 = (reserved)
    1+1+ 0+2,  // 28 = Set ATX Switch Functionality
    1+1+ 0+2,  // 29 = Enable/Feed Host Watchdog Reset
    1+1+15+2,  // 30 = Read Reporting/ATX/Watchdog (debug)
    1+1+ 0+2,  // 31 = Send data to LCD
    1+1+ 1+2,  // 33 = Set I2C slave address
    1+1+ 0+2,  // 34 = Set/Configure GPIO
    1+1+ 4+2}; // 35 = Read GPIO & Configuration

  //Table of times to delay in order to assure that the reply is valid.
  //These cheat/optimize a bit from the data sheet, since I have access
  //to the module firmware ;)
  static const uint16_t command_execution_delay[36] PROGMEM = {
       3,  //  0 = Ping Command (variable, 16 is max)
       2,  //  1 = Get Hardware & Firmware Version
      20,  //  2 = Write User Flash Area
       2,  //  3 = Read User Flash Area
      30,  //  4 = Store Current State As Boot State
    1500,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
       2,  //  6 = Clear LCD Screen
       3,  //  7 = Set LCD Contents, Line 1
       3,  //  8 = Set LCD Contents, Line 2
       2,  //  9 = Set LCD Special Character Data
       2,  // 10 = Read 8 Bytes of LCD Memory
       1,  // 11 = Set LCD Cursor Position
       1,  // 12 = Set LCD Cursor Style
       1,  // 13 = Set LCD Contrast
      50,  // 14 = Set LCD & Keypad Backlight
       2,  // 15 = Read Temperature
       0,  // 16 = (reserved)
       0,  // 17 = (reserved)
       2,  // 18 = Read DOW Device Information
       0,  // 19 = (reserved)
      50,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
       3,  // 21 = Setup Live Temperature Display (2 or max of 7)
       2,  // 22 = Send Command Directly to the LCD Controller
       0,  // 23 = (reserved)
       2,  // 24 = Read Keypad, Polled Mode
       0,  // 25 = (reserved)
       0,  // 26 = (reserved)
       0,  // 27 = (reserved)
       2,  // 28 = Set ATX Switch Functionality
       2,  // 29 = Enable/Feed Host Watchdog Reset
       3,  // 30 = Read Reporting/ATX/Watchdog (debug)
       4,  // 31 = Send data to LCD
       2,  // 33 = Set I2C slave address
       2,  // 34 = Set/Configure GPIO
       3}; // 35 = Read GPIO & Configuration
   
  //Validate the command
  if(35 < packet_to_send->command)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->command out of range %d requested, max is 35\n"),
                 packet_to_send->command);
      }
    return(1);
    }
  if(0xFF == receive_packet_length[packet_to_send->command])
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->command number %d is invalid/reserved\n"),
                 packet_to_send->command);
      }
    return(2);
    }
  //Validate the data length
  if(18 < packet_to_send->length)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->length out of range %d requested, max is 18\n"),
                 packet_to_send->length);
      }
    return(3);
    }
  //Start the I2C transaction    
  Wire.beginTransmission(i2c_address);
  //Send the command byte
  Wire.write(packet_to_send->command);
  //Send the significant data length (will match the I2C since we are in
  //control of it here)
  Wire.write(packet_to_send->length);
  //Send the data[]
  for(i=0;i<packet_to_send->length;i++)
    {
    Wire.write(packet_to_send->data[i]);
    }
  //Calculate the crc
  packet_to_send->crc = CRC((uint8_t*)packet_to_send, packet_to_send->length+2);
  //Low byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[0]))));
  //High byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[1]))));
  //Stop the I2C transaction
  Wire.endTransmission();
  
  //Now we need to wait for the command to complete, based on the
  //delay table.
  //Even with all the crazy cariable type macros above, this still does not work:
  //delay(command_execution_delay[packet_to_send->command]);
#define EXECUTION_DELAY (pgm_read_word_near(&command_execution_delay[packet_to_send->command]))
  delay(EXECUTION_DELAY);

#define EXPECTED_BYTES (pgm_read_byte_near(&receive_packet_length[packet_to_send->command]))
  //Now it is safe to read the response packet back from the CFA533.
  bytes_received=Wire.requestFrom(i2c_address,EXPECTED_BYTES);

  //If the bytes received does not agree, throw a warning.
  if(bytes_received != EXPECTED_BYTES)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: Wire.requestFrom fail. Expected %d bytes got %d bytes\n"),
                 EXPECTED_BYTES,
                 bytes_received);
      }
    }

  if(1<=bytes_received)
    {
    //Get the command byte of the respose
    packet_received->command=Wire.read();

    //Verify the low 6 bits of the Rx vs Tx command. They should match.
    if((packet_received->command & 0x3F ) != (packet_to_send->command & 0x3F ))
      {
      if(print_errors)
        {
        SerPrintFF(F("sendPacket_getReply: Received unexpected response of %3d (0x%02X) to command %3d (0x%02X).\n"),
                  packet_received->command&0x3F,packet_received->command&0x3F,
                  packet_to_send->command&0x3F, packet_to_send->command&0x3F);
        }
      }
    //Verify the top 2 bits of the Rx command. 
    if((packet_received->command & 0xC0 ) != (0x40))
      {
      if(print_errors)
        {
        SerPrintFF(F("Received unexpected response type of 0x%02X to command %3d (0x%02X). Expect 0x40.\n"),
                  packet_received->command & 0xC0,
                  packet_to_send->command&0x3F,   packet_to_send->command&0x3F);
        }
      }
    }
  else
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: No command byte returned.\n"));
      }
    return(4);
    }

  if(2<=bytes_received)
    {
    //Find out how may bytes of the transfer the CFA533 thinks are significant.
    packet_received->length=Wire.read();
    
    //Range check the length. There should not be more than 18 (max data) + 2 (crc), and
    //we should have at least response->length still available.
    if(((18 + 2) < packet_received->length)||
       (Wire.available() < packet_received->length))
      {
      if(print_errors)
        {
        SerPrintFF(F("Invalid length of %d in response to command %d. Truncating to %d.\n"),
                   packet_received->length,packet_to_send->command,Wire.available());
        }
      //Attempt to gracefully continue: Override the length
      packet_received->length = Wire.available();
      }
    }
  else
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: No length byte returned.\n"));
      }
    return(5);
    }
    
  //Transfer over the data
  for(i=0;i<packet_received->length;i++)
    {
    packet_received->data[i]=Wire.read();
    }
    
  //Check the CRC of the incoming packet.
  uint16_t
    calculated_crc;
  calculated_crc = CRC((uint8_t*)packet_received, packet_received->length+2);
    
  //Low byte of CRC
#define LOW_CRC  (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[0]))))
  //High byte of CRC
#define HIGH_CRC (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[1]))))

  uint8_t
    crc_low;  
  uint8_t
    crc_high;  
  crc_low=Wire.read();
  crc_high=Wire.read();
    
  if((crc_low != LOW_CRC) || (crc_high != HIGH_CRC))
    {
    if(print_errors)
      {
      SerPrintFF(F("calculated CRC hi:low (0x%02X:%02X) %3d : %3d\n"),
                 HIGH_CRC,LOW_CRC,
                 HIGH_CRC,LOW_CRC);
      SerPrintFF(F("received CRC hi:low (0x%02X:%02X) %3d : %3d\n"),
                 crc_high,crc_low,
                 crc_high,crc_low);
      }
    return(6);
    }
  //All good.
  return(0);
  }
//----------------------------------------------------------------------------
  const char cmd_Str_00[] PROGMEM = " 0 = Ping Command";
  const char cmd_Str_01[] PROGMEM = " 1 = Get Hardware & Firmware Version";
  const char cmd_Str_02[] PROGMEM = " 2 = Write User Flash Area";
  const char cmd_Str_03[] PROGMEM = " 3 = Read User Flash Area";
  const char cmd_Str_04[] PROGMEM = " 4 = Store Current State As Boot State";
  const char cmd_Str_05[] PROGMEM = " 5 = Reboot CFA-533, Reset Host, or Power Off Host";
  const char cmd_Str_06[] PROGMEM = " 6 = Clear LCD Screen";
  const char cmd_Str_07[] PROGMEM = " 7 = Set LCD Contents, Line 1";
  const char cmd_Str_08[] PROGMEM = " 8 = Set LCD Contents, Line 2";
  const char cmd_Str_09[] PROGMEM = " 9 = Set LCD Special Character Data";
  const char cmd_Str_10[] PROGMEM = "10 = Read 8 Bytes of LCD Memory";
  const char cmd_Str_11[] PROGMEM = "11 = Set LCD Cursor Position";
  const char cmd_Str_12[] PROGMEM = "12 = Set LCD Cursor Style";
  const char cmd_Str_13[] PROGMEM = "13 = Set LCD Contrast";
  const char cmd_Str_14[] PROGMEM = "14 = Set LCD & Keypad Backlight";
  const char cmd_Str_15[] PROGMEM = "15 = Read Temperature";
  const char cmd_Str_16[] PROGMEM = "16 = (reserved)";
  const char cmd_Str_17[] PROGMEM = "17 = (reserved)";
  const char cmd_Str_18[] PROGMEM = "18 = Read DOW Device Information";
  const char cmd_Str_19[] PROGMEM = "19 = (reserved)";
  const char cmd_Str_20[] PROGMEM = "20 = Arbitrary DOW Transaction (variable, 16 is max)";
  const char cmd_Str_21[] PROGMEM = "21 = Setup Live Temperature Display (2 or max of 7)";
  const char cmd_Str_22[] PROGMEM = "22 = Send Command Directly to the LCD Controller";
  const char cmd_Str_23[] PROGMEM = "23 = (reserved)";
  const char cmd_Str_24[] PROGMEM = "24 = Read Keypad, Polled Mode";
  const char cmd_Str_25[] PROGMEM = "25 = (reserved)";
  const char cmd_Str_26[] PROGMEM = "26 = (reserved)";
  const char cmd_Str_27[] PROGMEM = "27 = (reserved)";
  const char cmd_Str_28[] PROGMEM = "28 = Set ATX Switch Functionality";
  const char cmd_Str_29[] PROGMEM = "29 = Enable/Feed Host Watchdog Reset";
  const char cmd_Str_30[] PROGMEM = "30 = Read Reporting/ATX/Watchdog (debug)";
  const char cmd_Str_31[] PROGMEM = "31 = Send data to LCD";
  const char cmd_Str_32[] PROGMEM = "32 = (reserved)";
  const char cmd_Str_33[] PROGMEM = "33 = Set I2C slave address";
  const char cmd_Str_34[] PROGMEM = "34 = Set/Configure GPIO";
  const char cmd_Str_35[] PROGMEM = "35 = Read GPIO & Configuration";  
  //Should be able to put this array into PROGMEM, but it breaks:
  //    SerialPrint_P(Command_Strings[packet->command]);
  //even thought it works for
  //    SerialPrint_P(Command_Strings[3]);
  //weird as hell.
  //const PROGMEM char * const PROGMEM Command_Strings[36] = {
  const char *  Command_Strings[36] = {
    cmd_Str_00,cmd_Str_01,cmd_Str_02,cmd_Str_03,cmd_Str_04,cmd_Str_05,cmd_Str_06,
    cmd_Str_07,cmd_Str_08,cmd_Str_09,cmd_Str_10,cmd_Str_11,cmd_Str_12,cmd_Str_13,
    cmd_Str_14,cmd_Str_15,cmd_Str_16,cmd_Str_17,cmd_Str_18,cmd_Str_19,cmd_Str_20,
    cmd_Str_21,cmd_Str_22,cmd_Str_23,cmd_Str_24,cmd_Str_25,cmd_Str_26,cmd_Str_27,
    cmd_Str_28,cmd_Str_29,cmd_Str_30,cmd_Str_31,cmd_Str_32,cmd_Str_33,cmd_Str_34,
    cmd_Str_35};

uint8_t CrystalfontzI2CPacketLCD::dumpPacket(CFPacket_t *packet)
  {
  SerPrintFF(F("Decode %s%s packet: %2d ( 0x%02x ): \""),
             packet->command&0x80?"ERROR ":"",
             packet->command&0x40?"response":"command",
             packet->command&0x3F,
             packet->command&0x3F);
  SerialPrint_P(Command_Strings[(packet->command)&0x3F]);
  SerPrintFF(F("\"\n"));
  SerPrintFF(F("  Length: %2d ( 0x%02x )"),
             packet->length,
             packet->length);

  //Special decode for some packets
  if(0x01 == (packet->command&0x3F))
    {
    // 1 = Get Hardware & Firmware Version
    SerPrintFF(F(" Data: \""));
    uint8_t
      i;
    for(i=0;i<packet->length;i++)        
      {
      SerPrintFF(F("%c"),isprint(packet->data[i])?packet->data[i]:' ');
      }
    SerPrintFF(F("\" "));
    }
  else
    {
    // default byte-by byte dump
    SerPrintFF(F("\n"));
    uint8_t
      i;
    for(i=0;i<packet->length;i++)        
      {
      SerPrintFF(F("    Data[%2d]: %3d ( 0x%02x ) \'%c\'\n"),
                 i,
                 packet->data[i],
                 packet->data[i],
                 isprint(packet->data[i])?packet->data[i]:' ');
      }
    }
  SerPrintFF(F("  CRC: 0x%04X\n"),
             packet->crc,
             packet->length);
  }
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::Search_I2C_Adresses(void)
  {
  CFPacket_t
    address_command;
  CFPacket_t
    address_response;
  uint8_t
    original_address;
  uint8_t
    device_found_at_address;

  original_address=i2c_address;
  device_found_at_address=0xFF;
    
  //Set up the packet
  address_command.command = 1;
  address_command.length = 0;
  
  for(i2c_address=0;i2c_address<=127;i2c_address++)
    {
    address_response.command = 0xFF;
    address_response.length = 0;        
    uint8_t
       response;
    response=sendPacket_getReply(&address_command,&address_response,0);
   
    if(0x00 == response)
      {
      if(0xFF == device_found_at_address)
        {
        device_found_at_address=i2c_address;
        }
      SerPrintFF(F("Device found at addreess %3d\n"),i2c_address);
      dumpPacket(&address_response);
      }
    }
    
  i2c_address=original_address;
  return(device_found_at_address);
  }
  
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::Set_I2C_Adress(uint8_t address)
  {
  i2c_address=address;
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::writeText(uint8_t x, uint8_t y, char *text, uint8_t length)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  
  //Set up the packet
  command.command = 31;
  command.length = length + 2;
  command.data[0] = x;
  command.data[1] = y;
  memcpy(command.data + 2, text, length);
  
  //send the packet
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::clearScreen(void)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  
  //Set up the packet
  command.command = 6;
  command.length = 0;

  //send the packet
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
#define KP_UP     0x01
#define KP_ENTER  0x02
#define KP_CANCEL 0x04
#define KP_LEFT   0x08
#define KP_RIGHT  0x10
#define KP_DOWN   0x20
uint8_t CrystalfontzI2CPacketLCD::getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 24; //24 (0x18): Read Keypad, Polled Mode
  command.length = 0;
  //Send the packet, get the response
  sendPacket_getReply(&command,&response,0);
  // type: 0x40 | 0x18 = 0x58 = 8810
  // data_length: 3
  // data[0] = bit mask showing the keys currently pressed
  // data[1] = bit mask showing the keys that have been pressed since the last poll
  // data[2] = bit mask showing the keys that have been released since the last poll  
  //Pull the goodies out of the response, into the users varaibles.
  *down=response.data[0];
  *presses=response.data[1];
  *releases=response.data[2];
  
  if(*down || *presses || *releases)
    {
  //  dumpPacket(&command);
  //  dumpPacket(&response);
    return(1);
    }
  return(0);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::setUpBar(uint8_t spec_char,
                                        uint8_t vert_mask)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  //Second special character is always solid
  command.command = 9; // 9 (0x09): Set LCD Special Character Data
  command.length = 9;
  command.data[0]=spec_char; //Index of special character for solid
  command.data[1]=vert_mask & 0x80 ? 0x3F : 0; //Top Line
  command.data[2]=vert_mask & 0x40 ? 0x3F : 0;
  command.data[3]=vert_mask & 0x20 ? 0x3F : 0;
  command.data[4]=vert_mask & 0x10 ? 0x3F : 0;
  command.data[5]=vert_mask & 0x08 ? 0x3F : 0;
  command.data[6]=vert_mask & 0x04 ? 0x3F : 0;
  command.data[7]=vert_mask & 0x02 ? 0x3F : 0;
  command.data[8]=vert_mask & 0x01 ? 0x3F : 0; //Bottom Line

  //send the packet do define the always solid character
  sendPacket_getReply(&command,&response,0);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::drawBar(uint8_t col,
                                       uint8_t row,
                                       uint8_t chars_wide,
                                       uint8_t px_length,
                                       uint8_t spec_char_solid,
                                       uint8_t spec_char_variable,
                                       uint8_t vert_mask)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  uint8_t
    i;
    
    
  //Set up the text command
  command.command = 31; // 31 (0x1F): Send Data to LCD
  command.length = 1 + 1 + chars_wide;  // X + Y + chars_wide
  command.data[0]=col;    // X
  command.data[1]=row;    // Y
  //Fill in the solid characters.
  for(i=0;i<(px_length/5);i++)
    command.data[2+i]=spec_char_solid;   //Second special character (solid);
  //put in the partial character
  command.data[2+i]=spec_char_variable; //Special character (variable);
  for(i++;i<16;i++)
    command.data[2+i]=' '; //Rest are blanks
  //Send the characters to the display
  sendPacket_getReply(&command,&response,0);
    
  //Remap the spec_char_variable to the remainder
  command.command = 9; // 9 (0x09): Set LCD Special Character Data
  command.length = 9;
  
  uint8_t
    horizontal_mask;
  horizontal_mask = (0x1F << (5-(px_length%5)) & 0x1F);
  
  command.data[0]=spec_char_variable;  //Index of special character
  command.data[1]=vert_mask & 0x80 ? horizontal_mask : 0; //Top Line
  command.data[2]=vert_mask & 0x40 ? horizontal_mask : 0;
  command.data[3]=vert_mask & 0x20 ? horizontal_mask : 0;
  command.data[4]=vert_mask & 0x10 ? horizontal_mask : 0;
  command.data[5]=vert_mask & 0x08 ? horizontal_mask : 0;
  command.data[6]=vert_mask & 0x04 ? horizontal_mask : 0;
  command.data[7]=vert_mask & 0x02 ? horizontal_mask : 0;
  command.data[8]=vert_mask & 0x01 ? horizontal_mask : 0; //Bottom Line
  //Send the packet, get the response
  sendPacket_getReply(&command,&response,0);
  }
//============================================================================
CrystalfontzI2CPacketLCD   *cfPacket;  
//============================================================================
void Text_Scroll_Demo(void)
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;    
  while(1)
    {
    cfPacket->writeText(0,0, "Hello World     ", 16);
    cfPacket->writeText(0,1, "     Hello World", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, " Hello World    ", 16);
    cfPacket->writeText(0,1, "    Hello World ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "  Hello World   ", 16);
    cfPacket->writeText(0,1, "   Hello World  ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "   Hello World  ", 16);
    cfPacket->writeText(0,1, "  Hello World   ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "    Hello World ", 16);
    cfPacket->writeText(0,1, " Hello World    ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "     Hello World", 16);
    cfPacket->writeText(0,1, "Hello World     ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "d     Hello Worl", 16);
    cfPacket->writeText(0,1, "ello World     H", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "ld     Hello Wor", 16);
    cfPacket->writeText(0,1, "llo World     He", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "rld     Hello Wo", 16);
    cfPacket->writeText(0,1, "lo World     Hel", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "orld     Hello W", 16);
    cfPacket->writeText(0,1, "o World     Hell", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "World     Hello ", 16);
    cfPacket->writeText(0,1, " World     Hello", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, " World     Hello", 16);
    cfPacket->writeText(0,1, "World     Hello ", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "o World     Hell", 16);
    cfPacket->writeText(0,1, "orld     Hello W", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "lo World     Hel", 16);
    cfPacket->writeText(0,1, "rld     Hello Wo", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "llo World     He", 16);
    cfPacket->writeText(0,1, "ld     Hello Wor", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    cfPacket->writeText(0,0, "ello World     H", 16);
    cfPacket->writeText(0,1, "d     Hello Worl", 16);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;
    delay(100);
    }
  }
//============================================================================
uint8_t Get_LCD_Key_Character_For_Key_Demo(
          uint8_t key,
          uint8_t keys_down,
          uint8_t key_presses,
          uint8_t key_releases)
  {
  uint8_t
    return_value;
  return_value='-';
  if(keys_down & key)
    return_value=4;  //Dot
  if(key_presses & key)
    return_value=3;  //Down Arrow
  if(key_releases & key)
    return_value=2;  //Up Arrow
  if((key_presses & key)&&(key_releases & key))
    return_value=7;  //Up+Down Arrow
  return(return_value);
  }
//----------------------------------------------------------------------------
void Key_Poll_Demo_Forever(void)
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;
  //Set up the screen
  // 0123456789012345
  // 0000000000111111
  // CFA533-KC UECLRD
  // I2C Keys: ******
  cfPacket->writeText(0,0, "CFA533-KC UECLRD", 16);
  cfPacket->writeText(0,1, "I2C Keys:       ", 16);
  while(1)
    {
    //Ask the module what is happening with the keys
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    //Prep a string to show to the LCD:
    //   Up      arrow is special character 2
    //   Down    arrow is special character 3
    //   Up+Down arrow is special character 7
    //   dot     for held down, special character 4
    //   -       for no activity
    char
      keys_string[6];
    //Up Arrow Key
    keys_string[0]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_UP,keys_down,key_presses,key_releases);
    keys_string[1]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_ENTER,keys_down,key_presses,key_releases);
    keys_string[2]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_CANCEL,keys_down,key_presses,key_releases);
    keys_string[3]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_LEFT,keys_down,key_presses,key_releases);
    keys_string[4]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_RIGHT,keys_down,key_presses,key_releases);
    keys_string[5]=Get_LCD_Key_Character_For_Key_Demo(
                     KP_DOWN,keys_down,key_presses,key_releases);
    //Now print the string to the LCD
    cfPacket->writeText(10,1, keys_string, 6);
    //Wait a bit before the next poll. This is not necessary, but it shows
    //the transitions to the display. Typically a program would only react
    //releases, but you have full flexibility with this polling command.
    delay(100);
    }
  }
//============================================================================
void Bar_Graph_Demo(void)
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;
    
  //Set up the screen       0123456789012345
  cfPacket->writeText(0,0, "533I2C Bar Graph", 16);
  //Prep the second special character  
  cfPacket->setUpBar(4,     //uint8_t spec_char,
                     0xBD); //uint8_t vert_mask)

  uint8_t
    bar_target;

  while(1)
    {
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;      
    for(bar_target=0;bar_target<16*5;bar_target++)
      {
      cfPacket->drawBar(0,          // uint8_t col,
                        1,          // uint8_t row,
                        16,         // uint8_t chars_wide,
                        bar_target, // uint8_t px_length,
                        4,          // uint8_t spec_char_solid,
                        6,          // uint8_t spec_char_variable,
                        0xBD);      // uint8_t vert_mask)
      }
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    if(0x00 != key_releases)
      return;      
    for(;bar_target;bar_target--)
      {
      cfPacket->drawBar(0,          // uint8_t col,
                        1,          // uint8_t row,
                        16,         // uint8_t chars_wide,
                        bar_target, // uint8_t px_length,
                        4,          // uint8_t spec_char_solid,
                        6,          // uint8_t spec_char_variable,
                        0xBD);      // uint8_t vert_mask)
      }
      
    }
  }
//============================================================================
void Keys_and_Bar_Graph_Demo(void)
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;
    
  //Set up the screen       0123456789012345\\\
  cfPacket->writeText(0,0, "\1 or \0 \5 to exit", 16);
  
  //Prep the second special character  
  cfPacket->setUpBar(4,     //uint8_t spec_char,
                     0xBD); //uint8_t vert_mask)

  uint8_t
    bar_target;

  bar_target=40;

  while(1)
    {
    cfPacket->drawBar(0,          // uint8_t col,
                      1,          // uint8_t row,
                      16,         // uint8_t chars_wide,
                      bar_target, // uint8_t px_length,
                      4,          // uint8_t spec_char_solid,
                      6,          // uint8_t spec_char_variable,
                      0xBD);      // uint8_t vert_mask)
     
    while(0x00 == cfPacket->getKeys(&keys_down,&key_presses,&key_releases));
    if(key_releases & KP_ENTER)
      return;
    if(key_presses & KP_LEFT)
      if(0x00 != bar_target)
        bar_target--;
    if(key_presses & KP_RIGHT)
      if(bar_target < 80)
        bar_target++;
    if(key_presses & KP_DOWN)
      if(5 < bar_target)
        bar_target-=5;
      else
        bar_target=0;
    if(key_presses & KP_UP)
      if(bar_target < 75)
        bar_target+=5;
      else
        bar_target=80;
    }
  }
//============================================================================
void setup()
  {
  pinMode(led_pin, OUTPUT);
  pinMode(switch_pin, INPUT_PULLUP);
  Serial.begin(115200);  // start serial for output
  SerPrintFF(F("Free Memory: %d\n"),freeRam);

  Wire.begin(); // join i2c bus (address optional for master)
  //If you know the address:
  //  cfPacket = new CrystalfontzI2CPacketLCD(42);
  //If you do not know the address, this will search for the module.
  cfPacket = new CrystalfontzI2CPacketLCD(0xFF);
  cfPacket->Set_I2C_Adress(cfPacket->Search_I2C_Adresses());
  }
//============================================================================
void loop()
  {
  SerPrintFF(F("============================================================================\n"));
  //SerPrintFF(F("Waiting for permission to boot . . . "));
  //BootStall();
  SerPrintFF(F("booting now.\n"));
//  SerPrintFF(F("1"));
  
  while(1)
    {
//  SerPrintFF(F("1"));
    Keys_and_Bar_Graph_Demo();
//  SerPrintFF(F("2"));
    Text_Scroll_Demo();
//  SerPrintFF(F("3"));
    //Key_Poll_Demo_Forever();
    Bar_Graph_Demo();
//  SerPrintFF(F("4\n"));
    }
  }
//============================================================================

