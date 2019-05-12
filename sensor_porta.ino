/*===============================================================================================
  LoRa HalfDuplex communication wth callback

  SLAVE

  Listen to the Master, and uses callback   for new incoming messages. 
  Not implements callback for transmission (yet!!)
  
  Formats:

  enq = x05 - 1 byte
  ack = x06 - 1 byte

  msg { 
   header  = 1byte (xFF)
   payloadlen = 1byte (0..xFF)
   payload  = array byte [255]
   crc      = ? bytes  //  add in the LoRa link layer  https://www.libelium.com/forum/viewtopic.php?t=24813 - INVESTIGAR - ESTA ALTERANDO O PAYLOD !!!
  }

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

  created by Evaldo Nascimento and Paulo Abreu in 05/29, 2018
  based on the original version by Tom Igoe
 =============================================================================================== */


#define DEBUG           1   // 0 = no debug;  1 = partial debug;   2 = full debug

#define OLED            1   // controls if we will use the OLED display for messages

 
// include libraries
#include <SPI.h>            // comunication with radio LoRa 
// vem por padrão nas bibliotecas da IDE do Arduino

#include <LoRa.h>           // radio LoRa sx1276 lib
// Fonte: https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series <-- essa biblioteca exige 2 parâmetros na chamada da função LoRa.begin()
// Fonte: https://github.com/sandeepmistry/arduino-LoRa       <-- Usar esta. Exige apenas 1 parâmetro na chamada da função LoRa.begin()

#include <Wire.h>           // comunication i2c
// vem por padrão nas bibliotecas da IDE do Arduino

#if OLED > 0
#include "SSD1306.h"        // comunication with display OLED SSD1306
// Instale a partir da IDE do Arduino. Biblioteca dos autores Daniel e Fabrice
#endif
#include <OneWire.h>        // OneWire protocol used by the DS18x20 temperature sensor 
// Instale a partir da IDE do Arduino. Biblioteca do autor Jim Studt e outros

#include <TimeLib.h>		    // time/clock functions. To synch up with ntp time
#include <time.h>
// ambos os arquivos vem da fonte: https://github.com/PaulStoffregen/Time

#include <Timer.h>
// to schedule callback functions in the future 
// http://blog.rhesoft.com/2014/06/18/tutorial-using-timer-object-interval-and-singleshot-callback-in-arduino/
// download library Timer from https://github.com/aron-bordin/PNG-Arduino-Framework

#include <FastCRC.h>
// fonte: https://github.com/FrankBoesing/FastCRC


FastCRC8 CRC8;


// Protocol command definitions 
#define HEADER     0xFF     // header da mensagem
#define ENQ        0x05     // ASCII ENQ - enquire command
#define ACK        0x06     // ASCII ACK - acnoledge response
//#define GETTIME     0x01    // slave asks ntp time to master

#define ALARM      0x01      // the message to be transitted is an alarm
/*
01 - no_wather
02 - high_current_motor    // no futuro motor_X (X=1..9)
03 - no_current_motor       // no futuro motor_X    (X=1..9)
04 - power_down
05 - software_reset (watch_dog)
10 - door open
20 - during slave setup: gettime (pergunta a hora certa para o master) sempre que houver reset ou 1 vez por mes
outros alarmes: valor da corrente do motor, valor da temperatura do motor
*/
 
#define COMMAND    0x02      // the msg received is a command sent by the master: msgtype = byte 02H - COMMAND
                             // payload has 4 bytes with: COMMAND, GPIOSETUP, GPIO_pin, GPIO_value
#define   GPIOSETUP  0x00    // the command type is to set the gpio up

#define RESP              0x03 // Msgtype = byte 03H - RESP - response
#define   motor_on          0x01 // motor_on            OK|NG    byte   (OK=01, NG=00)
#define   motor_off         0x02 //- motor_off          OK|NG   
#define   water_level       0x03 //- water_level        OK|NG   
#define   water_temperature 0x04 //- water_temperature  XX     byte   (XX temp oC)
#define   current_ac_motor  0x05 //- current-ac_motor   YY.Y      byte   (XX Amps) 
#define   master_reset      0x06 //- master-reset       OK|NG    
#define   load_motor_table  0x07 //- load_motor_table   OK|NG   
#define   show_status       0x08 //- show_status        OK|NG  - transferir arquivo texto “status.txt”

#define NTPTIME   0x09      // payload has 4 bytes with ntp time

// Alarm IDs (types of alarm):
#define NOWATER           0x01  // 
#define NOCLOCK           0x20  // informs master that slave needs to synch its clock
/*02 - high_current_motor    // no futuro motor_X (X=1..9)
03 - no_current_motor       // no futuro motor_X    (X=1..9)
04 - power_down
05 - software_reset (watch_dog)
10 - door open
20 - during slave setup: gettime (per
*/

// Pinout definition
#define SCK     5          // GPIO5  -- SX127x's SCK
#define MISO    19         // GPIO19 -- SX127x's MISO
#define MOSI    27         // GPIO27 -- SX127x's MOSI
#define SS      18         // GPIO18 -- SX127x's CS
#define RST     14         // GPIO14 -- SX127x's RESET
#define DI00    26         // GPIO26 -- SX127x's IRQ(Interrupt Request)

OneWire  ds(17);           // OneWire protocol on pin 17 (a 4.7K pull up resistor is necessary)
#define DOOROPEN 35        // pin to receive information from a reed switch sensor


// LoRa frequency definition
#define BAND    915E6       // Lora Radio frequency -  433E6 for China, 868E6 for Europe, 915E6 for USA, Brazil

// Variables definitions
byte    TxmsgCount =                    0;              // counter of outgoing messages
byte    RxmsgCount =                    0;              // counter of incomming messages
long    lastSendTime, lastSendTime2 =   0;              // last send time
int     interval, interval2 =        2000;              // interval between sends
int     i =                             0;              // general counter

byte    rx_buffer[256];                                 // holds received bytes in callback function
byte    rx_byte;                                        // holds 1 received byte
byte    rx_buffer_head =                0;              // array index of rx_buffer  in callback  onReceive function
byte    rx_buffer_tail =                0;              // array index of rx_buffer  in foreground  
byte    p_rx_msg =                      0;              // array index of RX_msg 
byte    RX_msg [256];                                   // holds received bytes in foreground functions

// test - FF - 10 bytes - ABCDEFGHIJ
byte    tx_test[] =                    {0xFF, 10, 65, 66, 67, 68, 69, 70, 71, 72, 73,74}; 

byte    tx_buffer[256];
byte    tx_byte;                                        // holds 1 transmitted  byte in foreground sendBuffer function
byte    tx_buffer_head =                0;              // array index of tx_buffer  set before calling sendBuffer function
byte    tx_buffer_tail =                0;              // array index of tx_buffer  in "callback" actually: foreground sendBuffer function. Callback for transmission not implemented yet 
byte    msglength=                      0;              // size of received message
byte    crc =                           0;
byte    Status =                        0;              // SSM - Software State Machine controler

// int16_t Temperature;                                    // holds the temperature read from a DS18x20 sensor
byte    tx_temperature[1];

boolean dooropen = false;                               // indicates the door status
boolean ntpReceived = false;                            // controls if ntp time has been received from master

byte    tx_dooralarm[] =              {0xFF, 0x02, 0x01, 0x10}; 
//                                       |     |     |     +--> Alarm_ID = Door Open
//                                       |     |     +--------> Msgtype  = ALARM
//                                       |     +--------------> payload lenght - 2 bytes
//                                       +--------------------> msg header

#if OLED > 0
//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15);          //construtor do objeto que controlaremos o display
#endif

String rssi = "RSSI --";
String packSize = "--";
String packet;

String stringDate;

unsigned long ntpTime;
// CP1 to CP4 will store ntp time, byte by byte
unsigned char CP1 = 0;    // most significant byte
unsigned char CP2 = 0;
unsigned char CP3 = 0;
unsigned char CP4 = 0;    // least significant byte

// will call the callback in the interval of xxx ms - used for ntp setup
//Timer *timer1 = new Timer(5 * 1000);        // 5 sec interval 
Timer *timer1 = new Timer(15 * 1000);        // 15 sec interval 

// used for Temperature sensor
//Timer *timer2 = new Timer(10 * 1000);     // 10 sec interval
Timer *timer2 = new Timer(10 * 60 * 1000);  // 10 min = 10 * 60 s = 600s = 1000 * 600 = 600.000 ms
//Timer *timer2 = new Timer(300*1000);      // 5 min = 5 * 60 s = 300s = 1000 * 300 = 300.000 ms
//Timer *timer2 = new Timer(60*1000);       //  1min = 60s = 60*1000 ms


// Use the internal (adjusted by ntp) clock to create a string
// with the current date and time.
// Tipically used for debugging.
String theDateIs() {
    String dateNow;
    int weekNow = weekday();
    switch(weekNow) {
        case 1:
            dateNow = dateNow + "Domingo ";
            break;
        case 2:
            dateNow = dateNow + "Segunda-feira ";
            break;
        case 3:
            dateNow = dateNow + "Terça-feira ";
            break;
        case 4:
            dateNow = dateNow + "Quarta-feira ";
            break;
        case 5:
            dateNow = dateNow + "Quinta-feira ";
            break;
        case 6:
            dateNow = dateNow + "Sexta-feira ";
            break;
        case 7:
            dateNow = dateNow + "Sábado ";
            break;
    } // end switch

    if (day() < 10) {
        dateNow = dateNow + "0" + String(day()) + "/";
    } else {
        dateNow = dateNow + String(day()) + "/";
    }
    if (month() < 10) {
        dateNow = dateNow + "0" + String(month()) + "/";
    } else {
        dateNow = dateNow + String(month()) + "/";
    }
    dateNow = dateNow + String(year()) + " ";
    if (hour() < 10) {
        dateNow = dateNow + "0" + String(hour()) + ":";
    } else {
        dateNow = dateNow + String(hour()) + ":";
    }
    if (minute() < 10) {
        dateNow = dateNow + "0" + String(minute()) + ":";
    } else {
        dateNow = dateNow + String(minute()) + ":";
    }
    if (second() < 10) {
        dateNow = dateNow + "0" + String(second()) + " ";
    } else {
        dateNow = dateNow + String(second()) + " ";
    }
    return dateNow;
} // end theDateIs()


// callback function associated with timer1
// Used to check ntp time on a regular basis.
void checkNtp(){
    if (ntpReceived) {
        timer1->Stop(); //stop the thread.
#if DEBUG > 0
        Serial.println("checkNtp:: checkNtp timer stopped");
#endif
        // starts temperature sensor readings
        timer2->setOnTimer(&checkTemperature); //callback function
        timer2->Start(); //start the thread.
#if DEBUG >= 1
        Serial.println("checkNtp:: timer2 to regularly read temperature sensor was initiated.");
#endif
  } else {
        Status = 90; // force a new request for ntp time
#if DEBUG > 0
        Serial.print(theDateIs());
        Serial.println("checkNtp:: Will request new ntp time");
#endif
  }
}

// callback function associated with timer1
void checkTemperature(){
    Status = 80; // force a new request for temperature read
#if DEBUG > 0
    Serial.print(theDateIs());
    Serial.println("checkTemperature:: Will request new temperature read"); 
#endif
}

    // to read temperature from DS18x20 device using onewire protocol
    int16_t readTemperature(void) {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius, fahrenheit;

    ds.reset_search();
    delay(250);
  
    if ( !ds.search(addr)) {
#if DEBUG > 1
        Serial.print(theDateIs());
        Serial.println("checkTemperature:: No more addresses.");
#endif
        ds.reset_search();
        delay(250);
        return (-1);
    }
 
#if DEBUG > 1
    Serial.print(theDateIs());
    Serial.print("checkTemperature:: ROM =");
    for( i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }
#endif

    if (OneWire::crc8(addr, 7) != addr[7]) {
#if DEBUG > 1
        Serial.print(theDateIs());
        Serial.println("checkTemperature:: CRC is not valid!");
#endif
        return(-1);
    }
    //Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {

    case 0x10:
      // Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
#if DEBUG > 1
      Serial.println("  Chip = DS18B20");    //  este é o nosso chip!!
#endif
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
#if DEBUG > 1
      Serial.println("Device is not a DS18x20 family device.");
#endif
      return (-1);
  }
  //type_s = 0;
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

#if DEBUG > 1
  Serial.print("readTemperature:: Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");

#endif
  
    //type_s = 0;
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

#if DEBUG >= 2
    Serial.print("checkTemperature:: Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
#endif
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
#if DEBUG >= 2
      Serial.print(data[i], HEX);
      Serial.print(" ");
#endif
  } // end for()
#if DEBUG >= 2
    Serial.println();
    Serial.print(theDateIs());
    Serial.print("checkTemperature::CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
#else
    OneWire::crc8(data, 8);
#endif

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];

 /*
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
*/

    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
//  }

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
#if DEBUG > 1
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
#endif
  
  return (raw);
  
} // end of checkTemperature



// callback function setup to process  data received by the LoRa radio
void onReceive(int packetSize) {
  if (packetSize == 0) return;                          // if there's no packet, return
  do {
    rx_byte = LoRa.read();                              // receives one byte
    rx_buffer[rx_buffer_head++] = rx_byte;                // salves received byte in the rx_buffer
  } while (LoRa.available());                           // until all bytes are received
#if DEBUG > 1
  Serial.print("onReceive::  rx_buffer_tail="); Serial.print(rx_buffer_tail); Serial.print("    rx_buffer_head="); Serial.println(rx_buffer_head);
#endif
}



// send just 1 byte - used to send ASCII commands and responses (ENQ, ACK)
void sendByte (byte tx_byte) {   
  LoRa.beginPacket();                                   // start packet
  LoRa.write(tx_byte);                                  // add byte to be transmitted
  LoRa.endPacket();                                     // finish packet and send it
  LoRa.receive();                                       // you must make Lora listen to reception again !!
}



byte difference (byte b, byte a) {
  if ((b-a)<0) {
#if DEBUG > 1
    Serial.println("difference::  tamanho do buffer = "+ String(b-a+256));
#endif
    return (b-a+256);
  }else{
#if DEBUG > 1
    Serial.println("difference::  tamanho do buffer = "+ String(b-a));
#endif
    return (b-a);
  }
}



// send a buffer of bytes
void sendBuffer() {

    byte temp_buffer [256];
    byte i = 0 , j = 0, crcCalculated =0;
    unsigned int   buffer_size = 0;
    
    // add crc at the end of the txbuffer
    j = tx_buffer_tail;
    buffer_size = difference (tx_buffer_head, tx_buffer_tail);
    for (i = 0; i< buffer_size; i++) {
        temp_buffer[i] = tx_buffer [j];
        j++;
    }
#if DEBUG >= 2
    Serial.print("temp_buffer: ");
    for (i = 0; i< buffer_size; i++) {
        Serial.print(temp_buffer[i],HEX);
        Serial.print("+");
    }
    Serial.println();
#endif
    crcCalculated = CRC8.smbus(temp_buffer, buffer_size); 

#if DEBUG > 1
    Serial.println("sendBuffer::  tamanho do buffer = "+ String(difference (tx_buffer_head, tx_buffer_tail),HEX));
#endif

    tx_buffer[tx_buffer_head++] = crcCalculated;             // add the crc at the end of the msg to be transmitted
#if DEBUG > 1
    Serial.println("sendBuffer::  CRC = "+ String(crcCalculated,HEX));
#endif
 
#if DEBUG > 0
    Serial.println("sendBuffer:: Sending LoRa message");
#endif

    LoRa.beginPacket();                                  // start packet
    do {
        tx_byte = tx_buffer[tx_buffer_tail++];              // get the byte from the buffer
#if DEBUG > 1
        Serial.println("sendBuffer:: TxByte= " + String(tx_byte,HEX)+" txhead= " + String(tx_buffer_head)+" txtail= " + String(tx_buffer_tail));
#endif
        LoRa.write(tx_byte);                              // add byte to be transmitted
    } while (tx_buffer_tail != tx_buffer_head);         // while these 2 pointers are different, there are bytes to be trasnmitted
    LoRa.endPacket();                                   // finish packet and send it
    LoRa.receive();                                     // you must make Lora listen to reception again !!
    TxmsgCount++;                                       // increment Tx message counter
#if DEBUG > 1
    Serial.println("sendBuffer:: LoRa message sent");
#endif

#if OLED > 0
    // displays in the OLED TX and RX msgs counters
    display.clear();
    display.drawString(0, 0, "Slave Tx to Master");
    display.drawString(40, 13, String(TxmsgCount));
    display.drawString(0, 26, "Slave Rx from Master");
    display.drawString(40, 39, String(RxmsgCount));
    display.display(); 
    delay (1000);
#endif
}  // end of sendBuffer()


// check if there are bytes in the Rx buffer
// returns:
//    FALSE if nothing has been received by the LoRa radio yet
//    TRUE  if something has been received by the LoRa radio

boolean received() {
  byte crcReceived, crcCalculated=0;
  byte temp_buffer [256];
  byte i = 0 , j = 0;
  byte buffer_size = 0;
  
  if (rx_buffer_head != rx_buffer_tail) {

      j = rx_buffer_tail;
      buffer_size = difference (rx_buffer_head, rx_buffer_tail);
      for (i = 0; i< buffer_size; i++) {
        temp_buffer[i] = rx_buffer [j];
        j++;
      }
      buffer_size--;
      crcCalculated = CRC8.smbus(temp_buffer, buffer_size); 

    
      // for debug purposes only
#if DEBUG > 1
      Serial.print("received::   rx_buffer_head = "); Serial.println(rx_buffer_head);
      Serial.print("received::   rx_buffer_tail = "); Serial.println(rx_buffer_tail);
#endif
//      crcCalculated = CRC8.smbus(&rx_buffer[rx_buffer_tail], difference(rx_buffer_head, rx_buffer_tail)-1 ); // calculates the payload crc8. it does not included the received crc byte 
      crcReceived = rx_buffer[rx_buffer_head-1];   // back one position as the head is always pointing to the next received byte. The CRC8 is one position less 
#if DEBUG > 1
      Serial.print("received::   crcCalculated = "); Serial.println(crcCalculated,HEX); // shows boths crcs
      Serial.print("received::   crcReceived   = "); Serial.println(crcReceived,HEX);
      Serial.print("received::  ");
      for (i=rx_buffer_tail; i< rx_buffer_head;i++) {   // prints the rxed buffer byes
        Serial.print(rx_buffer[i],HEX);
        Serial.print("-");
      }
      Serial.println();
#endif
      if (crcCalculated != crcReceived) {   // if we had a crc error
         rx_buffer_tail = rx_buffer_head;    // discard the received msg
#if DEBUG > 1
         Serial.println("received::  CRC ERROR - msg deleted !!");
#endif
         return (false);
      }
  }
  return (rx_buffer_head != rx_buffer_tail);
}  // end of received()


void decoder (byte Rx_msg[]) {
    Serial.println("decoder::  Msg received from MASTER");
    RxmsgCount++;
#if OLED > 0
    display.clear(); //apaga todo o conteúdo da tela do display
    display.drawString(0, 0, " Msg received OK!");
    display.drawString(40, 13, String(Rx_msg[1]));
    display.drawString(40, 26, String(RxmsgCount));
    display.display(); 
#endif
	/*
	implementar aqui o tratamento do payload recebido do master ???   de acordo com:
	Msgtype = byte - 02H = COMMAND
	Command_ID: byte =
	01 - motor_on                             // no futuro _X (X=1..9)
	02 - motor_ off                           // no futuro off_X (X=1..9)
	* 03 - water_level
	* 04 - water_temperature
	* 05 - current-ac_motor                     // no futuro _X (X=1..9)
	06 - master-reset
	07 - load_motor_table
	08 - show_status
	09 - update your clock with this time
	*/
	
}

// callback function who will treat interrupts from the door reed switch sensor
void onDooropen () {  
   	dooropen = true;
}




void sendWaterTemperature() {
  time_t localTime;
  int16_t Temperature;               // holds the temperature read from a DS18x20 sensor


  byte CP1, CP2, CP3, CP4;
  
#if DEBUG > 0
    Serial.println("loop::  Slave reads temperature sensor");
#endif
    // 1. Slave reads the temperature
    //    Every message must have: a HEADER, a payload lenght, and the payload.
    //  readTemperature session
    Temperature = readTemperature();                  // reads the temperature from sensor
    
    if (Temperature != -1) {
#if DEBUG > 0
       Serial.print(theDateIs());
       Serial.print ("loop::case 80::Temperature =  ");
       Serial.println (Temperature);
#endif
                 
       tx_buffer[tx_buffer_head++] = HEADER;                   // builds the msg to be transmitted with the water temperature     
       tx_buffer[tx_buffer_head++] = 0x08;                     // our message is 4 bytes long
       tx_buffer[tx_buffer_head++] = RESP;
       tx_buffer[tx_buffer_head++] = water_temperature;
       tx_buffer[tx_buffer_head++] = Temperature >> 8;         // prepare 2 bytes to be send as the temperature  MSB
       tx_buffer[tx_buffer_head++] = Temperature & 0x00FF;     //LSB
              
       // send timestamp
       localTime = now();
       
       CP1 = (localTime & 0xff000000UL) >> 24;   // most significant byte
       CP2 = (localTime & 0x00ff0000UL) >> 16;
       CP3 = (localTime & 0x0000ff00UL) >>  8;
       CP4 = (localTime & 0x000000ffUL)      ;   // least significant byte
       tx_buffer[tx_buffer_head++] = CP1;
       tx_buffer[tx_buffer_head++] = CP2;
       tx_buffer[tx_buffer_head++] = CP3;
       tx_buffer[tx_buffer_head++] = CP4;
       
/*
      How to convert EpochTime to FormattedTime:
      From https://github.com/arduino-libraries/NTPClient/blob/master/NTPClient.cpp
*/
              
       sendBuffer(); // sends the message stored at the tx_buffer

#if DEBUG > 1
       Serial.print ("loop::case 80:: ");
       Serial.println(theDateIs());
#endif

#if OLED > 0
       display.clear();
       display.drawString(0, 0, stringDate);
       display.drawString(0, 26, String ((float)Temperature/16.0)+" oC");
       display.display();
#endif
    } //  if (Temperature != -1) 

} // end of sendWaterTemperature


void slaveSetTime() {
    // 5. Slave receives 4 bytes
    CP1 = rx_buffer[rx_buffer_tail++];  // most significant byte
    CP2 = rx_buffer[rx_buffer_tail++]; 
    CP3 = rx_buffer[rx_buffer_tail++]; 
    CP4 = rx_buffer[rx_buffer_tail++];  // least significant byte
    rx_buffer_tail++;   // discart CRC byte. Already verified in received()
    ntpReceived = true; // true is the normal, false to force a loop, always asking time to master 
#if DEBUG > 1
    Serial.println("loop::  After getting ntp data from rx_buffer, we have: ");
    Serial.print("loop::  rx_buffer_tail="); Serial.print(rx_buffer_tail); Serial.print("   rx_buffer_head="); Serial.println(rx_buffer_head); 
#endif
    // 6. Slave converts 4 bytes to ntp time
    ntpTime = (CP1 << 24) | (CP2 << 16) | (CP3 << 8) | CP4;
    // 7. Slave uses ntp time to setup its clock
    setTime(ntpTime);   // sets local clock
    // 8. prints time for debug purposes only

#if DEBUG > 0
    Serial.print("loop::  Time set to: ");
    Serial.println(theDateIs());
#endif                
}

void slaveAskTime() {
#if DEBUG > 1
    Serial.println("loop::  Entering State 90");
#endif
    Serial.println("loop::  Slave asks ntp time to master.");
    // 1. Slave asks time to Master.
    //    Every message must have: a HEADER, a payload lenght, and the payload.
    tx_buffer[tx_buffer_head++] = HEADER;
    tx_buffer[tx_buffer_head++] = 0x02; // our message is 2 bytes long
    tx_buffer[tx_buffer_head++] = ALARM;
    tx_buffer[tx_buffer_head++] = NOCLOCK;
    sendBuffer(); // sends the message stored at the tx_buffer
    // Paulo: coloquei as 2 linhas abaixo no setup() pq estava com um bug onde
    // o relogio estava sendo ajustado novamente, mesmo após o timer do ntp
    // ter sido parado. A minha teoria é a de que é melhor não redefinir
    // timer1 e reinicia-lo para cada vez que o slave pede o ntp time,
    // afinal, o timer1 já foi criado e já está rodando (22-jul-2018).
    //timer1->setOnTimer(&checkNtp); //callback function
    //timer1->Start(); //start the thread.
    // 2. Master obtains ntp time
    // 3. Master converts ntp time to 4 bytes
    // 4. Master sends 4 bytes to slave
    // 5. Slave receives 4 bytes: this is treated by setting Status = 0  
}




void slaveCheckCommand() {
    byte gpio_pin, gpio_value;
  
    // Slave receives 3 bytes: gpiosetuo , gpio_pin and gpio_value
    switch (rx_buffer[rx_buffer_tail++]) {
      case GPIOSETUP :
           gpio_pin = rx_buffer[rx_buffer_tail++];
           gpio_value = rx_buffer[rx_buffer_tail++];
           rx_buffer_tail++;   // discart CRC byte. Already verified in received()
           pinMode(gpio_pin, OUTPUT); 
           digitalWrite(gpio_pin, gpio_value);
           break;
      default:
           // In case all previous cases were not identified, restart SSM.
#if DEBUG > 1
           Serial.println("slaveCheckCommand::  undefined state! - reseting to Status = 0");
#endif
           Status = 0;
           break; // default
    } // end switch (rx_buffer[rx_buffer_tail++])
#if DEBUG > 1
    Serial.println("slaveCheckCommand::  After getting the command from rx_buffer, we have: ");
    Serial.print("slaveCheckCommand::  rx_buffer_tail="); Serial.print(rx_buffer_tail); Serial.print("   rx_buffer_head="); Serial.println(rx_buffer_head); 
#endif
  
} // end of slaveCheckCommand()

// setup everything ----------------------------------------------------------------------------
void setup() {
  
  Serial.begin(115200);                   
  while (!Serial);
  Serial.println("setup::  PPP LoRa Halfduplex Slave");

  // Turns on onboard LED for 1 second.
  // This is for testing purposes, as a command can be sent
  // to turn this LED on; so it is good to know that the LED
  // is working after a reset.
  pinMode (25, OUTPUT);
  digitalWrite (25, HIGH);
  delay (1000);
  digitalWrite (25, LOW);
  
  // Door open alarm definitions
  pinMode(DOOROPEN, INPUT);             //  this pin will receive a low signal if the door is open
  attachInterrupt(digitalPinToInterrupt(DOOROPEN), onDooropen, FALLING);  //  then it will callback onDooropen when it happens

#if OLED > 0
  // OLED display reset
  pinMode(16,OUTPUT); 
  digitalWrite(16, LOW);
  delay(50); 
  digitalWrite(16, HIGH);

  // initializes the OLED
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10);

   // Master is OK -->> OLED
  display.drawString(0, 0, "Slave is OK!");
  display.display(); 
  delay(3000);
  display.clear(); 
#endif

  // overide the default CS, reset, and IRQ pins (optional)
  SPI.begin(SCK,MISO,MOSI,SS);                    // start SPI with LoRa
  LoRa.setPins(SS, RST, DI00);                    // set CS, reset, IRQ pin

  // initialize ratio at 433 MHz
  if (!LoRa.begin(BAND)) {                   
    Serial.println("setup::  LoRa init failed. Check your connections.");
#if OLED > 0
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();                  
#endif
    while (true);                                 // if failed, do nothing
  }

  // enable Lora radio crc check
  // - INVESTIGAR - ESTA ALTERANDO O PAYLOD !!!
//  LoRa.enableCrc();  

  // setup callback function for reception
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("setup::  LoRa start OK!");
#if OLED > 0
  display.drawString(0, 0, " LoRa started OK!");
  display.display();
#endif
  delay (2000);

    // As 2 linhas abaixo estavam dentro do State 90 (que virou função slaveAskTime().
    // Foram colocadas aqui para ver se resolvem o bug do slave setar novamente o relogio
    // mesmo depois de já ter feito isso. (Paulo - 22-jul-2018)
    timer1->setOnTimer(&checkNtp); //callback function
    timer1->Start(); //start the thread.
#if DEBUG >= 1
    Serial.println("timer1 to ask ntp time to master was criated.");
#endif

  // The first state of SSM is to obtain ntp time from master
  Status = 90;
  
} // end of setup()
  
// -------------------------------------- MAIN loop -------------------------------------------
void loop() {
  
  // SSM (software State Machine)
  switch (Status) {
    
      case 0:
      // this is the initial state of the SSM during steady-state operation
      // wait for Master's ENQ or HEADER of a new msg
          //Serial.println("loop::  Entering case 0");
          //yield();
          if (received()) {
              switch (rx_buffer[rx_buffer_tail++]) {
                  case ENQ:
                  // ENQ means the master is asking the slave if it has any information
                  // to provide. For now we simply return with an ACK.
                      sendByte(ACK);
#if DEBUG > 1
                      Serial.println("loop::  ENQ received from Master");
                      Serial.println("loop::  ACK sent to MASTER");
#endif
                      Status = 0; // put the SSM in its initial state
                      break; // case ENQ
    
                  case HEADER:
                  // If a HEADER was received, this means we have received a command from the master.
                  // We need to process this message and find what we need to do.
#if DEBUG > 1
                      Serial.println("loop::  HEADER received from Master");
#endif
                      Status = 1;
                      break; // case HEADER
              } // end of switch   
          } // end of 'if( received() )'
          break; // case 0
  
      case 1:  
      // Previous state should have been 0 and a HEADER was found.
      // So now we need to get the message length (msglength).
#if DEBUG > 1
          Serial.println("loop::  Entering State 1");
#endif    
          msglength = rx_buffer[rx_buffer_tail++];         // get the msg lenght
          Status = 2;                                      // go process the rest of the message
          break; // case 1
  
      case 2:
      // This state process the content of the payload.
      // The payload length (msglength) was obtained in state 1.
#if DEBUG > 1
          Serial.println("loop::  Entering State 2");
#endif
          switch(rx_buffer[rx_buffer_tail++]) { // capture and treat the type of command
              case NTPTIME:
                  slaveSetTime();
                  Status = 0; // go back to steady state
                  break; // case NTPTIME
              case COMMAND:
                  slaveCheckCommand();
                  Status = 0; // go back to steady state
                  break; // case COMMAND
          } // end of switch
          break; // case 2

      case 80:
      // Read the temperature sensor and transmit its information to the master
#if DEBUG >= 2
          Serial.print ("loop::case 80:: rx_buffer_head antes de sendWaterTemperature() = ");
          Serial.println (rx_buffer_head);
          Serial.print ("loop::case 80:: rx_buffer_tail antes de sendWaterTemperature()= ");
          Serial.println (rx_buffer_tail);  
#endif

          sendWaterTemperature();

#if DEBUG >= 2
          Serial.print ("loop::case 80:: rx_buffer_head DEPOIS de sendWaterTemperature()= ");
          Serial.println (rx_buffer_head);
          Serial.print ("loop::case 80:: rx_buffer_tail DEPOIS de sendWaterTemperature()= ");
          Serial.println (rx_buffer_tail);  
#endif
          Status = 0;
          break; // case 80

      case 90:
      // Get ntp time from master: sends an ALARM message to master saying slave
      // does not have its clock properly set up (NOCLOCK).
         slaveAskTime();
         Status = 0; // waits for ntp time from master
         break; // case 90


      default:
      // In case all previous cases were not identified, restart SSM.
#if DEBUG > 1
          Serial.println("loop::  undefined state! - reseting to Status = 0");
#endif
          Status = 0;
          break; // default

  } // end of switch(Status)


    timer1->Update(); // this is the ntp timer (waiting for master to send ntp time).
    timer2->Update(); // this is the temp sensor timer (waiting for master to send ntp time).
    
 } // end of loop()
 
// ----------------------------------   END OF PROGRAM ----------------------------------------------------
