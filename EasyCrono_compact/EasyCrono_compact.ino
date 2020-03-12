#define BuzzerPin 3
#define STRING
//#define NUMBER
#define SOUND
//#define  SERIALDEBUG
#define BLENGHT 5
#define LED_1  8
#define LED_2 7

//*********************************************************

char cmd;
const char Start = 's';                //Carattere corrispondente a comando di Start
const char Reset = 'r';                //Carattere corrispondente a comando di Reset
const char Stop = 'p';                 //Carattere corrispondente a comando di Stop
const char Remote = 'z';               //Carattere corrispondente a comando di Stop
const char Connect = 'c';              //Carattere corrispondente ad una richiesta di ECHO per la connessione
const char InterTime = 'y';            //Carattere corrispondente ad una eventuale richiesta di intertempo (Non utilizzato)
const char ShortBeep = 'b';            //Carattere corrispondente ad una richiesta esclusivamente di un BEEP corto
const char LongBeep = 'l';             //Carattere corrispondente ad una richiesta esclusivamente di un BEEP lungo
const char Confirm = 'a';              //Carattere corrispondente alla conferma che la persona è ferma e stabile prima dell'inizio del Jump Test
const char ReactTime = 't';            //Carattere corrispondente all'inizio del Test del tempo di reazione
const char ReactTimeSpeed = 'u';       //Carattere corrispondente all'inizio del Test del tempo di reazione e Test di velocità insieme
const char SampleSend = 'd';           //Carattere corrispondente alla richiesta di invio di un campione del tempo di reazione
const char EndDataSend = 'e';          //Carattere che conferma l'avvenuta trasmissione di tutti i campioni relativi al tempo di reazione


const int InPinStart = 4;             //Pin relativo alla fotocellula di Start
const int InPinStop =  5;             //Pin relativo alla fotocellula di Stop             
const int InPinJump = 9;              //Pin relativo alla pedana a contatto per il Jump Test


unsigned long lastConnection = 0;
unsigned long connectionIntervall = 500;
bool connect_status = false;

int offsetX;                           //Parametro usato per settare lo zero dell'accelerometro al momento dell'accensione
int offsetY;                           //Parametro usato per settare lo zero dell'accelerometro al momento dell'accensione
int offsetZ;                           //Parametro usato per settare lo zero dell'accelerometro al momento dell'accensione

const int AntirimbalzoTime = 10;       //Tempo in millisecondi utilizzato per fissare un limite massimo ai ribbalzi dell'interruttore
const int AttesaRilascioTime = 10;     //Tempo in millisecondi che intercorre tra una verifica di rilascio di un tasto/sensore e l'altra (determina il tempo minimo per poter recepire due segnali distinti)
const int BeepTime = BLENGHT;               //Tempo in millisecondi che determina la lunghezza del beep
const int ArrayLeght = 299;            //Lunghezza dell'array di campioni per il tempo di reazione

uint8_t reactSamples[ArrayLeght];            //Buffer dedicato a immagazzinare i campioni provenienti dall'accelerometro
uint16_t reactInstant[ArrayLeght];            //Buffer dedicato a immagiazzinare gli istanti i tempo associati ai campioni provenienti dell'accelerometro
bool FirstJump = true;
 
int CountTransfer = 0;                                   //Contatore dei campioni durante l'invio al PC                   
unsigned long TStart, TStop, TReleased,TStartReact = 0;  //Variabili dedicate a cronometro e al tempo di reazione
unsigned long TStartPressed, TStopPressed,TPressed = 0;  //Variabili dedicate al TestJump
String EchoTime;


bool onReact = false;
#include <SPI.h>
#include "RF24.h"

#define CSNPIN       10
#define CEPIN        9
#define DYNPD        0x1C
#define DEBUG
int intervalSample = 1000;
bool radioNumber = 0;
int msg  = 0;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(CEPIN,CSNPIN);
/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;  //1 = TRASMETTITORE 0 = RICEVITORE

String prevId = "";

void setup()  
{
 
  Serial.begin(250000);                                  // Iniziallizzazione della seriale
  //******* SETTING OUTPUT ********                           

  //******* SETTING INPUT *********
  pinMode(InPinStart, INPUT);                            //Comandi per settare i pin come input e per attivare tutti i PULL-UP interni
  digitalWrite(InPinStart,HIGH);
  pinMode(InPinStop,INPUT);
  digitalWrite(InPinStop,HIGH);
  pinMode(InPinJump, INPUT);
  digitalWrite(InPinJump,HIGH);
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1,HIGH);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2,HIGH);
  
  delay(1000);
    
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1,LOW);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2,LOW);


  bool FirstJump = true;                                 //Setta la variabile di controllo a true per far si che avvenga il controllo di persona stabile sulla pedana prima del test

}




void loop()                                              //Loop
{  

    connection();
   
    SerialEvent();                                   //Verifica se sono presenti dati nel buffer della seriale
    
    TastoPremuto();                                  //Verifica se fotocellule o pedana di bosco connesse ai pin digitali sono premuti
    
}

//****************************************** GESTIONE PULSANTI E BEEPER *******************************************************************



void connection(){
  if(millis()-lastConnection >connectionIntervall){
    lastConnection = millis();
    doConnect();
  }
}


void connection_signal(){
  connect_status = !connect_status;
  digitalWrite(LED_2, connect_status);
}


void SerialEvent(){                                    //Verifica del contenuto del buffer di ricezione seriale e interpretazione dell'eventuale comando

   if(Serial.available()){                             //Tutti tutte le funzioni che vengono chiamate successivamente al alla ricezione del comando seriale restituiscono un ECHO al per notificarne l'avvenuta esecuzione
		                                       //Se il programma non riceve correttamente l'ECHO del comando viene generato un errore di connessione
		cmd = Serial.read();
		
		if(cmd == Connect){
			//doConnect();                   //Feedback di connessione che viene fornito al momento della selezione della porta da parte del programma su PC
      connection_signal();
		} 
		if(cmd == Start){
  
			doStart();                     //Esecuzione del comando di Start che si occupa di restituire il valore dei millisecondi a quell'istante
		}
		else if(cmd == Reset)
		{ 
			doReset();                     //Resetta il JumpTest reimpostando il test di persona stabile sulla pedana prima dell'inizio del Jump Test
		}
		else if(cmd == Stop)
		{
  
			doStop();                      //Esecuzione del comando di Stop che si occupa di restituire il valore dei millisecondi a quell'istante
		}
		else if(cmd == ShortBeep)
		{

      Beep();                        //Funzione BEEP
		}
     else if(cmd == ReactTime)
    {
      onReact = true;
      doReactTime();                 //Chiamata della funzione che per Test su Tempo di Reazione
		}
    else if(cmd == ReactTimeSpeed)
    {
      onReact = true;
      doReactTimeSpeed();           //Chiamata della funzione che per Test su Tempo di Reazione insieme al Test di Velocità
		}
    else if(cmd == SampleSend)
    {
      
		  Transfer();                   //Chiamata scatenate dalla richiesta di trasferimento del campione successivo fino ad esaurimento dei campioni
      
     }
   }

}




void Beep(){                                         //funzione che aziona il buzzer
  #ifdef SOUND
  
  digitalWrite(BuzzerPin,HIGH);
	delay(BeepTime);
	digitalWrite(BuzzerPin,LOW);
	
  #endif
}



void TastoPremuto(){                                  //Verifica della variazione di stato di tutti i sensori

	if(AntiRimbalzo(InPinStart)){
    digitalWrite(LED_1, HIGH);
		doStart();
		AttesaRilascio(InPinStart);
	}else
	{
	  digitalWrite(LED_1, LOW);
	}
 
	if(AntiRimbalzo(InPinStop)){
    digitalWrite(LED_1, HIGH);
		doStop();
		AttesaRilascio(InPinStop);
	}else
	{
	  digitalWrite(LED_1, LOW);
	}
 
	if(AntiRimbalzo(InPinJump)){               //Per quanto riguarda il Jump Test viene trasmesso l'istante della pressione della pedana e quello del rilascio della pedana in modo da poter calcolare il tempo di contatto
		doStopJump();
		AttesaRilascioJump(InPinJump);
		doStartJump();
	}
	
}

//************************************** CONTROLLI PER PULSANTI ***********************************************************

void AttesaRilascio(int Pin){

	while(!digitalRead(Pin))
        {
          connection();
          SerialEvent();
       		delay(AttesaRilascioTime);
        }
}


void AttesaRilascioJump(int Pin){
           if(FirstJump){
	        delay(1000);
                if(!digitalRead(Pin)){
                doConfirm();
                FirstJump = false;
                AttesaRilascio(Pin);
                }
           }else{
                while(!digitalRead(Pin)){
                connection();
		            delay(AttesaRilascioTime);}
           }
}



bool AntiRimbalzo(int Pin){

	if(!digitalRead(Pin)){
		delay(AntirimbalzoTime);
			if(!digitalRead(Pin)){
				return true;
			}
			
	}
	return false;
}
void Received(char cmd){
	
	Echo((String)cmd+";");

	SerialClear();
}

//*************************************** ATTUAZIONE COMANDI ***************************************************************



void doConnect(){                                                  //ECHO del segnale connessione per notifica al programma su PC
	
     Echo("c;");
}


void doReactTime(){
       Echo("t;0000;");
       randomSeed(sqrt(square(analogRead(A0)-offsetX)+square(analogRead(A1)-offsetY)+square(analogRead(A2)-offsetZ)));
       delay(random(2500,5000));                                   //Randomizza un tempo che va da 2,5 secondi a 5 secondi
       
       Calibrate();      
       digitalWrite(BuzzerPin,HIGH);
       TStartReact = millis();	
       delay(25);
       digitalWrite(BuzzerPin,LOW);
       for(int i = 0; i<ArrayLeght ; i++){
       delayMicroseconds(intervalSample);
       reactSamples[i] = sqrt(square(analogRead(A0)-offsetX)+square(analogRead(A1)-offsetY)+square(analogRead(A2)-offsetZ));
       reactInstant[i] = millis() - TStartReact;
       }
       Beep();
       CountTransfer = 0;
       Echo("e;0000;");
       Beep();
      
      
}

void doReactTimeSpeed(){
       
       //AccendiSensore();
       Echo("u;0000;");
       randomSeed(sqrt(square(analogRead(A0)-offsetX)+square(analogRead(A1)-offsetY)+square(analogRead(A2)-offsetZ)));          //Imposta il seme della funzione random in base dei valori casuali prelevati dagli ingressi analogici
      
       delay(random(2500,3500));     
       Calibrate();
       
       digitalWrite(BuzzerPin,HIGH);
       TStartReact = TStart= millis();
       EchoTime = "s;"+(String)(millis())+";";
       Echo(EchoTime);
       delay(50);
       digitalWrite(BuzzerPin,LOW);
       for(int i = 0; i<ArrayLeght ; i++){
       delayMicroseconds(intervalSample);                                                                                             //Ritardo intrdotto per far si che tutti i campioni siano distanziati di 2 millisecondi
       reactSamples[i] = sqrt(square(analogRead(A0)-offsetX)+square(analogRead(A1)-offsetY)+square(analogRead(A2)-offsetZ))/4;   //Il ritardo introdotto da questo calcolo e di 350 microSecondi che sommato ai precedenti 1650 da 2 milliSecondi di intervallo tra un campione e l'altro      
       reactInstant[i] = millis() - TStartReact;
       }
       Beep();
       CountTransfer = 0;
       Echo("e;0000;");

       Beep();
       //SpengiSensore();
       
}


void Calibrate()                          //Associa ai valori di offset i valori iniziali delle tre uscite dell'accelerometro per poter fare l'azzeramento
{

offsetX = analogRead(A0);
offsetY = analogRead(A1);
offsetZ = analogRead(A2);

}


void doStart(){
	Beep();
	//TStart = millis();
	EchoTime = "s;"+(String)(millis())+";";
	Echo(EchoTime);
}

void doStart(unsigned long tOffset){
  Beep();
  //TStart = millis();
  EchoTime = "z;"+(String)(millis()-tOffset)+";"+(String)tOffset + ";" ;
  Echo(EchoTime);
}


void doStartJump(){
	Beep();
	//TStart = millis();
	EchoTime = "j;"+(String)(millis())+";";
	Echo(EchoTime);
}
void doConfirm(){
  
  Beep();
	//TStart = millis();
	EchoTime = "a;0000;";
	Echo(EchoTime);

}

void doStopJump(){
	Beep();
	//TStart = millis();
	EchoTime = "k;"+(String)(millis())+";";
	Echo(EchoTime);
}


void doStop(){
	Beep();
	EchoTime ="p;"+(String)(millis())+";";
	Echo(EchoTime);
}

void doReset(){                                   
    Beep();
    //TStart, TStop = 0;
    FirstJump = true;
    Echo("r;0000;");
}

void sendBlevel(int level)
{
Echo("b;"+ level);
}
//***************************************** COMUNICAZIONI SERIALI ********************************************************************

void Echo(String Echo){                                    //Echo generico al quale viene passato il relativo comando ricevuto
	      Serial.println(Echo);
        SerialClear();
        
}


void SerialClear(){                                        //Svuotamento del Buffer della seriale

        while(Serial.available())
        {
           Serial.read();
           
        }
        Serial.flush();
        
        
}
//***************************************** PROTOCOLLI INVIO ********************************************************************


void Transfer(){       
//Invio dei campioni al programma
if(CountTransfer < ArrayLeght){
  
  EchoTime= "d;"+(String)CountTransfer+"-"+ (String)reactInstant[CountTransfer]+"-"+ (String)reactSamples[CountTransfer]+";";
  Echo(EchoTime);
  CountTransfer++;
}
else{
Echo("i;0000;");
onReact = false;
}
}


//#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
//#define ACTIVATE      0x50
//#define R_RX_PL_WID   0x60
//#define R_RX_PAYLOAD  0x61
//#define W_TX_PAYLOAD  0xA0
//#define W_ACK_PAYLOAD 0xA8
//#define FLUSH_TX      0xE1
//#define FLUSH_RX      0xE2
//#define REUSE_TX_PL   0xE3
//#define NOP 0xFF

uint8_t write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

 
  csn(LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  csn(HIGH);

  return status;
}


void csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz

  digitalWrite(CSNPIN,mode);
}
