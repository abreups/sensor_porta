/*===============================================================================================
   
  Sensor de porta aberta/fechada usando ESP32 LoRa Wi-Fi da HelTec
  Autor: Paulo S Abreu
  v1: Maio-2019

  teste5: envia um pacote LoRa onde o quarto byte (ALARMVALUE) não é conhecido pelo master (gateway).
  
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

//#include <TimeLib.h>		    // time/clock functions. To synch up with ntp time
#include <time.h>
// ambos os arquivos vem da fonte: https://github.com/PaulStoffregen/Time


#include <FastCRC.h> // fonte: https://github.com/FrankBoesing/FastCRC


FastCRC8 CRC8;


// Protocol command definitions 
//#define HEADER     0xFF     // header da mensagem
//#define ENQ        0x05     // ASCII ENQ - enquire command
//#define ACK        0x06     // ASCII ACK - acnoledge response


//#define ALARM      0x01      // the message to be transitted is an alarm


// As mensagens enviadas pelo Slave (e portanto recebidas pelo Master) têm o seguinte formato:
// 5 bytes: RESP, SLAVEID, ALARMID, ALARMVALUE, CRC
//
//          RESP        =  indica que a mensagem é uma resposta enviada pelo Slave.
//          SLAVEID     =  ID que identifica o Slave que enviou a mensagem.
//          ALARMID     =  qual o tipo de alarme que o Slave está informando.
//          ALARMVALUE  =  qual o valor do alarme informado.
//          CRC         =  CRC calculado para os bytes anteriores.

#define     COMMAND     0x02
#define     RESP        0x03

#define     SLAVEID     0x01    // este é o ID do nosso slave

// Valores para ALARMID
#define     ALARMEDEPORTA   0x01

// Valores válidos para ALARMVALUE:
#define     PORTAABERTA   0x01
#define     PORTAFECHADA  0x02

// Pinout definition da placa HelTec Wi-Fi LoRa 32
#define SCK     5          // GPIO5  -- SX127x's SCK
#define MISO    19         // GPIO19 -- SX127x's MISO
#define MOSI    27         // GPIO27 -- SX127x's MOSI
#define SS      18         // GPIO18 -- SX127x's CS
#define RST     14         // GPIO14 -- SX127x's RESET
#define DI00    26         // GPIO26 -- SX127x's IRQ(Interrupt Request)


#define GPIO_SENSOR_PORTA 35        // pin to receive information from a reed switch sensor


// Frequencia do rádio LoRa (depende de que placa você está usando):
#define BAND    915E6       // Brazil e USA
//#define BAND    868E6       // Europa
//#define BAND    433E6       // China


// Variables definitions
byte    TxmsgCount =                    0;              // counter of outgoing messages
byte    RxmsgCount =                    0;              // counter of incomming messages
long    lastSendTime, lastSendTime2 =   0;              // last send time
int     interval, interval2 =        2000;              // interval between sends
int     i =                             0;              // general counter


byte    tx_buffer[256];
byte    tx_byte;                                        // holds 1 transmitted  byte in foreground sendBuffer function
byte    tx_buffer_head =                0;              // array index of tx_buffer  set before calling sendBuffer function
byte    tx_buffer_tail =                0;              // array index of tx_buffer  in "callback" actually: foreground sendBuffer function. Callback for transmission not implemented yet 
byte    msglength=                      0;              // size of received message
byte    crc =                           0;
byte    Status =                        0;              // SSM - Software State Machine controler


// Se for utilizar alguma variável para passar dados entre uma ISR e o programa principal, 
// a variável deve ser definida como 'volatile', garantindo assim que elas sejam atualizadas corretamente.
// Veja: https://www.fernandok.com/2018/06/os-profissionais-sabem-disso-interrupt.html
volatile boolean portaAberta = false;    // false == porta está fechada; true == porta está aberta
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;    // semáforo para ser usado dentro da interrupção


#if OLED >= 1
//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15);          //construtor do objeto do display
#endif

String rssi = "RSSI --";
String packSize = "--";
String packet;


//---------------------------------------------------------------------------
// Definições das funções auxiliares:
//


//-----------------------------------------------------------------------------
// difference(): função que calcula o tamanho de um buffer circular
byte difference (byte b, byte a) {
  if ((b-a)<0) {
#if DEBUG > 1
    Serial.println("difference::tamanho do buffer = "+ String(b-a+256));
#endif
    return (b-a+256);
  }else{
#if DEBUG > 1
    Serial.println("difference::tamanho do buffer = "+ String(b-a));
#endif
    return (b-a);
  }
}


//--------------------------------
// sendBuffer(): função que transmite vários bytes pelo rádio LoRa
//               CRC é inserido no final.
//
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
    Serial.print("sendBuffer::temp_buffer: ");
    for (i = 0; i< buffer_size; i++) {
        Serial.print(temp_buffer[i],HEX);
        Serial.print("+");
    }
    Serial.println();
#endif
    crcCalculated = CRC8.smbus(temp_buffer, buffer_size); 
#if DEBUG >= 2
    Serial.println("sendBuffer::tamanho do buffer = "+ String(difference (tx_buffer_head, tx_buffer_tail),HEX));
#endif
    tx_buffer[tx_buffer_head++] = crcCalculated;             // add the crc at the end of the msg to be transmitted
#if DEBUG >= 2
    Serial.println("sendBuffer::CRC = "+ String(crcCalculated,HEX));
#endif
#if DEBUG >= 1
    Serial.println("sendBuffer::Sending LoRa message");
#endif
    LoRa.beginPacket();                                  // start packet
    do {
        tx_byte = tx_buffer[tx_buffer_tail++];              // get the byte from the buffer
#if DEBUG >= 2
        Serial.println("sendBuffer::TxByte= " + String(tx_byte,HEX)+" txhead= " + String(tx_buffer_head)+" txtail= " + String(tx_buffer_tail));
#endif
        LoRa.write(tx_byte);                              // add byte to be transmitted
    } while (tx_buffer_tail != tx_buffer_head);         // while these 2 pointers are different, there are bytes to be trasnmitted
    LoRa.endPacket();                                   // finish packet and send it
    LoRa.receive();                                     // you must make Lora listen to reception again !!
    TxmsgCount++;                                       // increment Tx message counter
#if DEBUG >= 2
    Serial.println("sendBuffer::LoRa message sent");
#endif

#if OLED > 0
    // displays in the OLED TX and RX msgs counters
    display.clear();
    display.drawString(0, 0, "Slave Tx para Master");
    display.drawString(40, 13, String(TxmsgCount));
    display.drawString(0, 26, "Slave Rx de Master");
    display.drawString(40, 39, String(RxmsgCount));
    display.display(); 
    delay (1000);
#endif
} // end of sendBuffer()


// --------------------------------  setup ---------------------------------
void setup() {
  
  Serial.begin(115200);
  delay(1000);  // espera a serial "estabilizar"
  while (!Serial);
  Serial.println("setup::Serial OK");

  // Turns on onboard LED for 1 second.
  // This is for testing purposes, as a command can be sent
  // to turn this LED on; so it is good to know that the LED
  // is working after a reset.
  pinMode (25, OUTPUT);
  digitalWrite (25, HIGH);
  delay (1000);
  digitalWrite (25, LOW);

  // Inicializa display OLED
#if OLED >= 1
  // OLED display reset
  pinMode(16,OUTPUT); 
  digitalWrite(16, LOW);
  delay(50); 
  digitalWrite(16, HIGH);

  // initializes the OLED
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10);

   // Slave OK -->> OLED
  display.drawString(0, 0, "Slave OK!");
  display.display(); 
  delay(3000);
  display.clear(); 
#endif

  // Definições para o sensor de porta
  pinMode(GPIO_SENSOR_PORTA, INPUT);  //  this pin will receive a low signal if the door is open (?)

  // testa estado da GPIO para pegar o estado da porta
  byte leitura_porta = digitalRead(GPIO_SENSOR_PORTA);
  if(leitura_porta == LOW) {
#if DEBUG >= 1
      Serial.print("Porta fechada = "); Serial.println(leitura_porta, DEC);
#endif
      portENTER_CRITICAL_ISR(&mux);
      portaAberta = false;
      portEXIT_CRITICAL_ISR(&mux);
  } else {
#if DEBUG >= 1    
      Serial.print("Porta aberta = "); Serial.println(leitura_porta, DEC);
#endif      
      portENTER_CRITICAL_ISR(&mux);
      portaAberta = true;
      portEXIT_CRITICAL_ISR(&mux);
  }
  

  // Inicializa rádio LoRa:
  // override the default CS, reset, and IRQ pins (optional)
  SPI.begin(SCK,MISO,MOSI,SS);                    // start SPI with LoRa
  LoRa.setPins(SS, RST, DI00);                    // set CS, reset, IRQ pin
  // inicializa rádio LoRa na frequencia especificada
  if (!LoRa.begin(BAND)) {                   
    Serial.println("setup::inicialização LoRa falhou.");
#if OLED > 0
    display.drawString(0, 0, "Inicialização LoRa falhou");
    display.display();                  
#endif
    //while (true);     // se LoRa falhou, entra em loop infinito.
    // ou... entra em deep sleep de novo:
    //esp_deep_sleep_start();   // entra no modo deep sleep!    
  } else {
#if DEBUG >= 1
  Serial.println("setup::LoRa OK!");
#endif  
#if OLED >= 1
  display.drawString(0, 0, "LoRa OK!");
  display.display();
#endif    
  }


#if DEBUG >=1 
  Serial.println("Transmitindo mensagem com ALARMVALUE incorreto.");
#endif
  // transmite um pacote com alarme de porta
  if(portaAberta) {
      // prepara mensagem de alarme que porta foi aberta
      tx_buffer[tx_buffer_head++] = RESP;
      tx_buffer[tx_buffer_head++] = SLAVEID;
      tx_buffer[tx_buffer_head++] = ALARMEDEPORTA;
      tx_buffer[tx_buffer_head++] = 0x99;         // 0x99 é um ALARMEVALUE desconhecido para o gateway.
      // ajusta a próxima interrupção para quando a porta for fechada
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
  } else {
      // prepara menagem de alarme que porta foi fechada
      tx_buffer[tx_buffer_head++] = RESP;           // 0x05 é um byte desconhecido para o mstaer (gateway)
      tx_buffer[tx_buffer_head++] = SLAVEID;
      tx_buffer[tx_buffer_head++] = ALARMEDEPORTA;
      tx_buffer[tx_buffer_head++] = 0x99;         // 0x99 é um ALARMEID desconhecido para o gateway.
      // ajusta a próxima interrupção para quando a porta for aberta
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
  }
  sendBuffer(); // transmite
#if DEBUG >=1 
  Serial.println("Pacote transmitido.");
  Serial.println("Entrando em deep sleep");
#endif
  delay(3000); // aguarda 3s antes do deep sleep para dar tempo de ler o display.
  esp_deep_sleep_start();   // entra no modo deep sleep!
} // end of setup()


// ------------------------------------ loop -------------------------------------------
void loop() {
    // Nada a fazer.
    // A interrupção na GPIO do sensor é a única coisa que faz algo.   
}
 
// ----------------------------------  fim do programa  --------------------------------------------
