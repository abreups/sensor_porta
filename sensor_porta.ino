/*===============================================================================================
   
  Sensor de porta aberta/fechada usando ESP32 LoRa Wi-Fi da HelTec
  Autor: Paulo S Abreu
  v1: Maio-2019
  
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

// As mensagens recebidas pelo Slave (e portanto enviadas pelo Master) têm o seguinte formato:
// 3 bytes: COMMAND, SLAVEID, GPIOPIN, GPIOVALUE
//
//          COMMAND   =  indica que a mensagem recebida é um comando enviado pelo Master.
//          SLAVEID   =  ID que identifica o Slave para o qual o comando se destina.
//          GPIOPIN   =  indica qual a porta GPIO a ser comandada.
//          GPIOVALUE =  indica qual o valor a ser colocado na porta GPIO (UP or DOWN).

// As mensagens enviadas pelo Slave (e portanto recebidas pelo Master) têm o seguinte formato:
// 3 bytes: RESP, SLAVEID, ALARMID, ALARMVALUE
//
//          RESP        =  indica que a mensagem é uma resposta enviada pelo Slave.
//          SLAVEID     =  ID que identifica o Slave que enviou a mensagem.
//          ALARMID     =  qual o tipo de alarme que o Slave está informando.
//          ALARMVALUE  =  qual o valor do alarme informado.

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

byte    rx_buffer[256];                                 // holds received bytes in callback function
byte    rx_byte;                                        // holds 1 received byte
byte    rx_buffer_head =                0;              // array index of rx_buffer  in callback  onReceive function
byte    rx_buffer_tail =                0;              // array index of rx_buffer  in foreground  
byte    p_rx_msg =                      0;              // array index of RX_msg 
byte    RX_msg [256];                                   // holds received bytes in foreground functions

// test - FF - 10 bytes - ABCDEFGHIJ
//byte    tx_test[] =                    {0xFF, 10, 65, 66, 67, 68, 69, 70, 71, 72, 73,74}; 

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


byte    tx_dooralarm[] =              {0xFF, 0x02, 0x01, 0x10}; 
//                                       |     |     |     +--> Alarm_ID = Door Open
//                                       |     |     +--------> Msgtype  = ALARM
//                                       |     +--------------> payload lenght - 2 bytes
//                                       +--------------------> msg header

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

/*
 * não estamos interessados no que chega pelo rádio LoRa.
 * 
//----------------------------------------------------------------------------
// onReceive: Função de callback para processar os dados recebidos pelo rádio LoRa.
//
void onReceive(int packetSize) {
  if (packetSize == 0) return;                // se nada foi recebido, volte.
  do {                                        // repita o loop...
    rx_byte = LoRa.read();                    // recebe 1 byte
    rx_buffer[rx_buffer_head++] = rx_byte;    // guarda o byte no rx_buffer e incrementa o ponteiro
  } while (LoRa.available());                 // ... enquanto tiver bytes sendo recebidos
#if DEBUG > 1
  Serial.print("onReceive::rx_buffer_tail = "); Serial.print(rx_buffer_tail);
  Serial.print("; rx_buffer_head = "); Serial.println(rx_buffer_head);
#endif
}
*/

/*
 * não estamos usando sendByte()
//------------------------------------------------------------------------------
// sendByte(): função que envia 1 byte pelo rádio LoRa.
//
void sendByte (byte tx_byte) {   
  LoRa.beginPacket();           // inicia pacote
  LoRa.write(tx_byte);          // adiciona byte ao pacote a ser transmitido
  LoRa.endPacket();             // encerra pacote e transmite
  LoRa.receive();               // chamada necessária para não perder alguma recepção de pacote
}
*/

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


/*
 * não vamos usar esse tipo de interrupção. vamos usar a interrupção do deep sleep.
//--------------------------------
// onDooropen(): callback function who will treat interrupts from the door reed switch sensor
//
void onDooropen () {  
    // veja: https://www.fernandok.com/2018/06/os-profissionais-sabem-disso-interrupt.html

    portENTER_CRITICAL_ISR(&mux);
   	portaAberta = !portaAberta;
    portEXIT_CRITICAL_ISR(&mux);
#if DEBUG >= 1
    if (portaAberta) {
        Serial.println("Porta abriu");
    } else {
        Serial.println("Porta fechou");
    }
#endif
}
*/

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
  

/*  
  não vamos usar essa interrupção. vamos usar a interrupção do deep sleep

  // chama função de callback onDooropen se houver interrupção na porta GPIO_SENSOR_PORTA
  // Opções:
  //     LOW : aciona a interrupção sempre que o pino estiver baixo.
  //     CHANGE: aciona a interrupção sempre que o pino muda de estado.
  //     RISING: aciona a interrupção quando o pino vai de baixo para alto (LOW > HIGH).
  //     FALLING: para acionar a interrupção quando o pino vai de alto para baixo (HIGH > LOW)
  //     HIGH : aciona a interrupção sempre que o pino estiver alto.
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR_PORTA), onDooropen, CHANGE);  
*/



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

/*
 não vamos receber nada pelo rádio LoRa

  // setup callback function for reception
  LoRa.onReceive(onReceive);
  LoRa.receive();

  delay (2000);
*/

  // transmite um pacote com alarme de porta
  if(portaAberta) {
      // prepara mensagem de alarme que porta foi aberta
      tx_buffer[tx_buffer_head++] = RESP;
      tx_buffer[tx_buffer_head++] = SLAVEID;
      tx_buffer[tx_buffer_head++] = ALARMEDEPORTA;
      tx_buffer[tx_buffer_head++] = PORTAABERTA;
      // ajusta a próxima interrupção para quando a porta for fechada
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
  } else {
      // prepara menagem de alarme que porta foi fechada
      tx_buffer[tx_buffer_head++] = RESP;
      tx_buffer[tx_buffer_head++] = SLAVEID;
      tx_buffer[tx_buffer_head++] = ALARMEDEPORTA;
      tx_buffer[tx_buffer_head++] = PORTAFECHADA;
      // ajusta a próxima interrupção para quando a porta for aberta
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
  }
  sendBuffer(); // transmite
  delay(3000); // aguarda 3s antes do deep sleep para dar tempo de ler o display.
  esp_deep_sleep_start();   // entra no modo deep sleep!
} // end of setup()


// ------------------------------------ loop -------------------------------------------
void loop() {
    // Nada a fazer.
    // A interrupção na GPIO do sensor é a única coisa que faz algo.   
}
 
// ----------------------------------  fim do programa  --------------------------------------------
