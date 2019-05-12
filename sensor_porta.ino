/*===============================================================================================
  LoRa HalfDuplex communication wth callback

  Este módulo funciona como um slave (ou seja, supõe que há um master correspondente).
  
  SLAVE - teste de acesso Paulo via Mac

  Listen to the Master, and uses callback   for new incoming messages. 
  Not implements callback for transmission (yet!!)
  
  Formats:

  enq = x05 - 1 byte
  ack = x06 - 1 byte

  msg { 
   header  = 1byte (xFF)
   payloadlen = 1byte (0..xFF)
   payload  = array byte [255]
   crc16  = 2bytes  // not ready yet!
  }

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

  created by Evaldo Nascimento and Paulo Abreu in 05/29, 2018
  based on the original version by Tom Igoe
 =============================================================================================== */

//-------------------------------------------
// Inicializações
//-------------------------------------------

// include libraries
#include <SPI.h>            // comunication with radio LoRa 
#include <LoRa.h>           // radio LoRa sx1276 lib
#include <Wire.h>           // comunication i2c
#include "SSD1306.h"        // comunication with display OLED SSD1306

// Protocol command definitions 
#define HEADER     0xFF     // header da mensagem
#define ENQ        0x05     // ASCII ENQ - enquire command
#define ACK        0x06     // ASCII ACK - acnoledge response
#define GETTIME     0x01    // slave asks ntp time to master

// Pinout definition
#define SCK     5          // GPIO5  -- SX127x's SCK
#define MISO    19         // GPIO19 -- SX127x's MISO
#define MOSI    27         // GPIO27 -- SX127x's MOSI
#define SS      18         // GPIO18 -- SX127x's CS
#define RST     14         // GPIO14 -- SX127x's RESET
#define DI00    26         // GPIO26 -- SX127x's IRQ(Interrupt Request)

// LoRa frequency definition
#define BAND    433E6       // Lora Radio frequency -  433E6 for China, 868E6 for Europe, 915E6 for USA, Brazil

// Variables definitions
byte    TxmsgCount =                    0;              // counter of outgoing messages
byte    RxmsgCount =                    0;              // counter of incomming messages
long    lastSendTime, lastSendTime2 =   0;              // last send time
int     interval, interval2 =        2000;              // interval between sends
int     i =                             0;              // general counter

byte    rx_buffer[255];                                 // holds received bytes in callback function
byte    rx_byte;                                        // holds 1 received byte
byte    rx_buffer_head =                0;              // array index of rx_buffer  in callback  onReceive function
byte    rx_buffer_tail =                0;              // array index of rx_buffer  in foreground  
byte    p_rx_msg =                      0;              // array index of RX_msg 
byte    RX_msg [255];                                   // holds received bytes in foreground functions

// test - FF - 10 bytes - ABCDEFGHIJ
byte    tx_test[] =                    {0xFF, 10, 65, 66, 67, 68, 69, 70, 71, 72, 73,74}; 

byte    tx_buffer[255];
byte    tx_byte;                                        // holds 1 transmitted  byte in foreground sendBuffer function
byte    tx_buffer_head =                0;              // array index of tx_buffer  set before calling sendBuffer function
byte    tx_buffer_tail =                0;              // array index of tx_buffer  in "callback" actually: foreground sendBuffer function. Callback for transmission not implemented yet 
byte    msglenght=                      0;              // size of received message

byte    Status =                        0;              // SSM - Software State Machine controler

//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15);     //construtor do objeto que controlaremos o display

String rssi = "RSSI --";
String packSize = "--";
String packet ;

//-------------------------------------------
// Funções auxiliares
//-------------------------------------------
//
// Funções usadas no loop principal do programa.
//


//--------------------------------------------------------------------------------
// Função onReceive() - é uma função de callback (acionada por interrupção)
//   que trata o recebimento de pacotes no rádio LoRa.
void onReceive(int packetSize) {
  if (packetSize == 0) return;                          // se o tamanho do pacote recebido foi zero, faça nada.
  do {
    rx_byte = LoRa.read();                              // lê o byte recebido
    rx_buffer[rx_buffer_head] = rx_byte;                // guarda o byte no buffer rx_buffer
    rx_buffer_head++;                                   // incrementa o apontador para a próxima posição no buffer rx_buffer
  } while (LoRa.available());                           // repete até que todos os bytes tenham sido lidos
}


//--------------------------------------------------------------------------------
// função sendByte() - envia 1 byte pelo rádio LoRa.
// Useda para enviar mensagens ASCII (tanto comandos como respostas tipo ACK)
void sendByte (byte tx_byte) {
                                // o formato padrão para se enviar um byte é:
  LoRa.beginPacket();           // 1. inicia um pacote
  LoRa.write(tx_byte);          // 2. prepara o byte a ser transmitido
  LoRa.endPacket();             // 3. encerra o pacote e transmite
  LoRa.receive();               // 4. chama função de escutar a recepção (necessário, caso contrário você pode perder uma transmissão que foi feita pelo master)
}


//--------------------------------------------------------------------------------
// Função sendBuffer() - envia vários bytes pelo rádio LoRa.
//   sendBuffer() transmite todos os bytes que foram armazenados (por alguém) 
//   no buffer de transmissão tx_buffer.
// send a buffer of bytes
void sendBuffer() {
  LoRa.beginPacket();                                 // start packet
  do {
    tx_byte = tx_buffer[tx_buffer_head];              // get the byte from the buffer
    Serial.println("TxByte= " + String(tx_byte)+" txhead= " + String(tx_buffer_head)+" txtail= " + String(tx_buffer_head));
    LoRa.write(tx_byte);                              // add byte to be transmitted
    tx_buffer_head++;                                 // increments the tx_buffer pointer to get the next  byte
  } while (tx_buffer_head != tx_buffer_tail);         // while these 2 pointers are different, there are byte to be trasnmitted
  LoRa.endPacket();                                   // finish packet and send it
  LoRa.receive();                                     // you must make Lora listen to reception again !!
  TxmsgCount++;                                       // increment Tx message counter

  // displays in the OLED TX and RX msgs counters
  // (Útil como um tipo de debug, para acompanhar o que está acontecendo).
  display.clear();
  display.drawString(0, 0, "Slave enviou pacote: ");
  display.drawString(40, 13, String(TxmsgCount));
  display.drawString(0, 26, "Recebido do Master: ");
  display.drawString(40, 39, String(RxmsgCount));
  display.display(); 
}


//--------------------------------------------------------------------------------
// Função received() - verifica se há bytes no buffer de recepção rx_buffer.
// returns:
//    FALSE if nothing has been received by the LoRa radio yet
//    TRUE  if something has been received by the LoRa radio
boolean received() {
  return (rx_buffer_head != rx_buffer_tail);
}


//--------------------------------------------------------------------------------
// Função decoder() - faz o que mesmo?
//
void decoder (byte Rx_msg[]) {
    Serial.println("Msg received from MASTER");
    RxmsgCount++;
    display.clear(); //apaga todo o conteúdo da tela do display
    display.drawString(0, 0, " Msg received OK!");
    display.drawString(40, 13, String(Rx_msg[1]));
    display.drawString(40, 26, String(RxmsgCount));
    display.display(); 
}


//-------------------------------------------
// Setup
//-------------------------------------------

void setup() {
  Serial.begin(115200);                   
  while (!Serial);
  Serial.println("PPP LoRa Halfduplex Slave");

  // OLED display reset
  pinMode(16,OUTPUT); 
  digitalWrite(16, LOW);
  delay(50); 
  digitalWrite(16, HIGH);

  // initializes the OLED
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10);

   // slave is OK -->> OLED
  display.drawString(0, 0, "Slave OK");
  display.display(); 
  delay(3000);
  display.clear(); 
  
  // overide the default CS, reset, and IRQ pins (optional)
  SPI.begin(SCK,MISO,MOSI,SS);                    // start SPI with LoRa
  LoRa.setPins(SS, RST, DI00);                    // set CS, reset, IRQ pin

  // initialize ratio at 433 MHz
  if (!LoRa.begin(BAND)) {                   
    Serial.println("LoRa init failed. Check your connections.");
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();                  
    while (true);                                 // if failed, do nothing
  }

  // setup callback function for reception
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("Inicializado LoRa");
  display.drawString(0, 0, " Inicializado LoRa");
  display.display();
  delay (2000);


  // Clock adjustment at Slave
  // 1. Slave asks time to Master
  sendByte(GETTIME);
  // 2. Master obtains ntp time
  // 3. Master converts ntp time to 4 bytes
  // 4. Master sends 4 bytes to slave
  // 5. Slave receives 4 bytes
  Serial.print("Waiting for LoRa packet");
  while( !received() ) {
    Serial.print(".");
    delay(100);
    yield(); // necessário?
  }
  // 6. Slave converts 4 bytes to ntp time


  // 7. Slave uses ntp time to setup its clock


  // 8. prints time for debug purposes only

} // end of setup()


//-------------------------------------------
// lopp principal
//-------------------------------------------

void loop() {
  
  // Reception session
  // SSM (software State Machine
  switch (Status) {
    case 0:                                             // wait for Master´s  ENQ or HEADER od a new msg
       if (received()) {
          switch (rx_buffer[rx_buffer_tail]) {
             case ENQ:
                Serial.println("ENQ from Master");
                rx_buffer_tail++;
                sendByte (ACK);
                Serial.println("ACK to MASTER");
                Status = 0;
             break;
             case HEADER:                              //  master  sent a msg
                rx_buffer_tail++;                      // skip header
                Status = 1;
             break;
          }
      }
    break;
    
    case 1:  
      msglenght = rx_buffer[rx_buffer_tail];          // get the msg lenght
      rx_buffer_tail++;
      p_rx_msg = 0;
      Status = 2;
    break;  
    
    case 2:                                           // and get the rest of de msg
      rx_byte = rx_buffer[rx_buffer_tail];
      RX_msg [p_rx_msg] = rx_byte;
      rx_buffer_tail++;
      p_rx_msg++;
      if (msglenght == p_rx_msg) {                    // at the end of the received msg change to SSM initial state
        Status = 0;
        decoder (RX_msg);                             // decode theh rx msg - show the second payload byte in the OLED
      }
    break;
     default:                                         // if something goes wrong restart SMM
      Status = 0;
    break;
    }
    
//  transmition session
//  timer depend only for test
    if (millis() - lastSendTime > interval) {
      lastSendTime = millis();                        // timestamp the message
      interval = random(2000) + 3000;                 // 3-5 seconds

      for (i=0; i< sizeof(tx_test); i++) {            // copy test msg to tx_buffer
        tx_buffer[tx_buffer_tail] = tx_test[i];
        tx_buffer_tail++;
      }
      // tx_buffer_tail += sizeof(tx_test);           // set tx_buffer_tail to the last byte of the message to be trasnmitted
      Serial.println("SLAVE Tx Buffer");
      Serial.println("Tx Buffer Tail = " + String(tx_buffer_tail, DEC));
      Serial.println();
      sendBuffer ();                                  // sends the msg
    } else {                                          // Slave does not poll the master
    }  
 }


// fim do programa
