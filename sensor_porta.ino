/*===============================================================================================
   
  Sensor de porta aberta/fechada usando ESP32 LoRa Wi-Fi da HelTec
  Autor: Paulo S Abreu
  v1: Maio-2019
  v2: Junho 2019
      Usando classe Cqueue em C++ para buffer circular.
  
 =============================================================================================== */


#define DEBUG           2   // 0 = no debug;  1 = partial debug;   2 = full debug

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

// Buffer circular implementado pela classe Cqueue.
// Crie uma instância com 'Cqueue <seuBuffer>;'
// Métodos:
//    Empty(): 
//        retorna true se buffer está vazio.
//        retorna false se buffer não está vazio.
//
//    Full():
//        retorna true se buffer está cheio.
//        retorna false se buffer não está cheio.
//
//    Add(int Element): coloca um byte ('Element') no fim do buffer.
//        retorna true se operação teve sucesso.
//        retorna false se houve problema (e byte não foi para a fila).
//
//    Delete(): retira o elemento que está na frente da fila.
//        retorna true se operação foi bem sucedida.
//        retorna false se houve problema (e elemento não foi retirado da fila).
//
//    getFront(): 
//        retorna o byte que está na frente da fila.
//        retorna -1 se o buffer estava vazio.
//    Size():
//        retorna a quantidade de bytes na fila (o tamanho do buffer ocupado por dados). 
//
//    Clean():
//        zera o conteúdo do buffer.
//
class Cqueue
{
	private:
		int Rear_, Front_;
		int Queue_[256];
		int Max_;
		int Size_;
	public:
		Cqueue() {Size_ = 0; Max_ = 256; Rear_ = Front_ = -1;}
		bool Empty() const;
		bool Full() const;
		bool Add(int Element);
		bool Delete();
		int getFront();
    int Size() const;
    void Clean();
};

bool Cqueue::Empty() const
{
	if(Size_ == 0)
		return true; // sim, está vazia.
	else
		return false; // não, não está vazia.
}

bool Cqueue::Full() const
{
	if(Size_ == Max_)
		return true; // buffer está cheio.
	else
		return false; // buffer não está cheio.
}

bool Cqueue::Add(int Element)
{
	if(!Full()) {
		Rear_ = (Rear_ + 1) % Max_;
		Queue_[Rear_] = Element;
		Size_++;
		return true;
	} else
		return false;
}

bool Cqueue::Delete()
{
	if(!Empty()) {
		Front_ = (Front_ + 1) % Max_;
		Size_--;
		return true;
	} else
		return false;
}

int Cqueue::getFront()
{
	int Temp;
	if(!Empty()) {
		Temp = (Front_ + 1) % Max_;
		return(Queue_[Temp]);
	} else
		return(-1);
}

int Cqueue::Size() const
{
    return Size_;
}

void Cqueue::Clean()
{
    Rear_ = Front_;
    Size_ = 0;
}

// Cria uma instância de buffer circular chamada 'Q'.
// S é o nosso "Send buffer" (ou Transmit buffer) do rádio LoRa.
Cqueue S;

Cqueue T; // fila temporária.

// imprimeFila(): função que coloca na serial o conteúdo de uma fila (um buffer Cqueue).
//
void imprimeFila(Cqueue Fila)
{
        Cqueue T;
        T = Fila;
        while (!T.Empty()) {
                Serial.print(T.getFront());Serial.print("-");
                T.Delete();
        }
        Serial.println();
}


// As mensagens enviadas pelo Slave (e portanto recebidas pelo Master) têm o seguinte formato:
// 5 bytes: RESP, SLAVEID, ALARMID, ALARMVALUE, CRC
//
//          RESP        =  indica que a mensagem é uma resposta enviada pelo Slave.
//          SLAVEID     =  ID que identifica o Slave que enviou a mensagem.
//          ALARMID     =  qual o tipo de alarme que o Slave está informando.
//          ALARMVALUE  =  qual o valor do alarme informado.
//	    CRC	        = CRC calculado para os bytes anteriores.

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

#define GPIO_SENSOR_PORTA 36        // pin to receive information from a reed switch sensor
//#define GPIO_SENSOR_PORTA 35        // pin to receive information from a reed switch sensor

// Frequencia do rádio LoRa (depende de que placa você está usando):
#define BAND    915E6       // Lora Radio frequency -  433E6 for China, 868E6 for Europe, 915E6 for USA, Brazil

boolean portaAberta = false; // false == porta está fechada; true == porta está aberta

#if OLED >= 1
//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15);          //construtor do objeto do display
#endif


//---------------------------------------------------------------------------
// Definições das funções auxiliares:
//

//--------------------------------
// sendBuffer(): função que transmite vários bytes pelo rádio LoRa
//               Essa função acrescenta automaticamente um byte de CRC ao final da mensagem.
//
void sendBuffer() {

    byte crc_buffer[4];    // buffer temporário para calcular CRC.
    Cqueue Qtemp;          // buffer temporário.
    byte crcCalculado = 0;
    
    Qtemp = S; // copia o conteúdo do Send buffer para o buffer temporário Qtemp.
#if DEBUG >= 2
    Serial.print("sendBuffer::Qtemp (sem CRC) = ");
    imprimeFila(Qtemp);
#endif

    // pega os bytes da mensagem a ser transmitida.
    crc_buffer[0] = Qtemp.getFront(); Qtemp.Delete(); // RESP
    crc_buffer[1] = Qtemp.getFront(); Qtemp.Delete(); // SLAVEID
    crc_buffer[2] = Qtemp.getFront(); Qtemp.Delete(); // ALARMID
    crc_buffer[3] = Qtemp.getFront(); Qtemp.Delete(); // ALARMVALUE
    crcCalculado = CRC8.smbus(crc_buffer, 4); 
    
    S.Add(crcCalculado);  // coloca o byte do CRC na fila da mensagem.
#if DEBUG >= 2
    Serial.print("sendBuffer::S (com CRC) = "); imprimeFila(S);
#endif
#if DEBUG >= 1
    Serial.println("sendBuffer::Sending LoRa message");
#endif
    LoRa.beginPacket();               // Inicia pacote LoRa.
    do {
        LoRa.write( S.getFront() );   // Pega próximo byte da fila para transmitir.
        S.Delete();                   // Remove da fila esse byte.
    } while ( !S.Empty() );         // Se a fila não estiver vazia, pega o próximo byte.
    LoRa.endPacket();                 // Termina pacote LoRa e transmite.
    LoRa.receive();                   // Necessário para não perder pacotes a serem recebidos (não é nosso caso, mas...)
#if DEBUG >= 1
    Serial.println("sendBuffer::LoRa message sent");
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

  // inicializa display OLED
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10);

  display.drawString(0, 0, "Display OK");
  display.display(); 
  delay(3000); //depois retirar
#endif

  // Definições para o sensor de porta
  pinMode(GPIO_SENSOR_PORTA, INPUT);  //  this pin will receive a low signal if the door is open (?)

  // testa estado da GPIO para pegar o estado da porta
  byte leitura_porta = digitalRead(GPIO_SENSOR_PORTA);
  if(leitura_porta == LOW) {
#if DEBUG >= 1
      Serial.print("Porta fechada ("); Serial.print(leitura_porta, DEC); Serial.println(")");
#endif
      portaAberta = false;
  } else {
#if DEBUG >= 1    
      Serial.print("Porta aberta ("); Serial.print(leitura_porta, DEC); Serial.println(")");
#endif      
      portaAberta = true;
  }
  

  // Inicializa rádio LoRa:
  // override the default CS, reset, and IRQ pins (optional)
  SPI.begin(SCK,MISO,MOSI,SS);                    // start SPI with LoRa
  LoRa.setPins(SS, RST, DI00);                    // set CS, reset, IRQ pin
  // inicializa rádio LoRa na frequencia especificada
  if (!LoRa.begin(BAND)) {
    Serial.println("setup::inicialização LoRa falhou.");
#if OLED > 0
    display.drawString(0, 10, "Inicializacao LoRa falhou");
    display.display();                  
#endif
    //while (true);     // se LoRa falhou, entra em loop infinito.
    // ou... entra em deep sleep de novo:
    //esp_deep_sleep_start();   // entra no modo deep sleep!    
    // ou... dá um reset geral?
  } else {
#if DEBUG >= 1
  Serial.println("setup::LoRa OK!");
#endif  
#if OLED >= 1
  display.drawString(0, 10, "Radio LoRa OK");
  display.display();
  delay(3000); //depois retirar
#endif    
  }


// Ajusta potência de transmissão.
// Se a potência do rádio LoRa para a transmissão estiver ajustada para um valor
// muito alto (como o default, que é o modo PA_BOOST, a placa vai puxar muita
// corrente e se estiver alimentada por bateria, estas não aguentam a demanda
// e a placa reseta.
// Ajustando para uma potência mais baixa elimina o problema. Obviamente o alcance
// do rádio loRa diminui.
// void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
//   PA_OUTPUT_PA_BOOST_PIN = 1: é o default e é a potência de transmissão mais alta.
//   0 <= level <= 14
LoRa.setTxPower( 1 , 0);


#if DEBUG >=1 
  Serial.println("Transmitindo mensagem.");
#endif
  // transmite um pacote com alarme de porta
  if(portaAberta) {
      // prepara mensagem de alarme que porta foi aberta
      S.Add( RESP );
      S.Add( SLAVEID );
      S.Add( ALARMEDEPORTA );
      S.Add( PORTAABERTA );
#if OLED >= 1
      display.drawString(0, 20, "Alarme PORTA ABERTA");
      display.display();
#endif 
#if DEBUG >=1 
      Serial.println("Alarme PORTA ABERTA.");
#endif
      // ajusta a próxima interrupção para quando a porta for fechada
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
  } else {
      // prepara menagem de alarme que porta foi fechada
      S.Add( RESP );
      S.Add( SLAVEID );
      S.Add( ALARMEDEPORTA );
      S.Add( PORTAFECHADA );
#if OLED >= 1
      display.drawString(0, 20, "Alarme PORTA FECHADA");
      display.display();
#endif 
#if DEBUG >=1 
      Serial.println("Alarme PORTA FECHADA.");
#endif
      // ajusta a próxima interrupção para quando a porta for aberta
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
  }
  sendBuffer(); // transmite

#if OLED >= 1
  // pra dar tempo de ler o display
  delay(3000); // depois retirar
#endif 

#if DEBUG >=1 
  Serial.println("Alarme transmitido.");
  Serial.println("Entrando em deep sleep");
#endif

#if OLED >= 1
  display.drawString(0, 30, "Entrando em deep sleep");
  display.display();
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
