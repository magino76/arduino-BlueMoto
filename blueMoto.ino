#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>

#define EE_Timeout 10 // indirizzo in EEProm dove si trova il byte EE_Timeout
#define EE_speed 11 // indirizzo in EEProm dove si trova il low byte della speed
#define EE_PswFlag 12 // indirizzo in EEProm dove si trova il flag password (sara settato a 1 solo quando interviene allarme)
#define EE_Psw0 0
#define EE_Psw1 1
#define EE_Psw2 2
#define EE_Psw3 3
#define EE_Psw4 4
#define EE_Psw5 5


#define rxPin 12
#define txPin 11
#define PswChange 5 // ingresso  utilizzato con pull-up per cambiare password. Ingresso a 0 consente il cambio password!!
#define LedPin 8 // seleziono il piedino 8 come uscita Led lampeggiante in allarme cruscotto

#define Connessione 7  // seleziono il piedino 7 come uscita led connessione
#define allarmPin 2  // seleziono il piedino 2 come uscita allarme relè
#define SpeedPin A0 // seloziono il piedino A0 come ingresso analogico
#define DefaultTimeout 120
#define On 1
#define Off 0

//Imposto INPUT/OUTPUT
//SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
SoftwareSerial BTserial(rxPin, txPin); // RX, TX

// costanti salvate in EEPROM

const byte EEPROM_ID = 0x99; // serve a identificare se nella EEPROM ci sono dati validi


// inizializzo variabili
int ContatoreTimeout = 255;
int Speed = 0;
int ActualSpeed = 0;
int Timeout = 60;
char Password[7] = {'1', '2', '3', '4', '5', '6'};
char EE_Password[7];

boolean PswFlag = false;

boolean AllarmeInCorso = false;
boolean TimeoutAvvenuto = false; // se settato a 1 significa che è stato raggiunto il EE_Timeout e quindi deve essere scatenato l'allarme
boolean ConnessionePresente = false;
boolean passwordOK = false;
int data = 0;
static int CONTATORE_TIMEOUT = 2; // il contatore timeouto impostato dal cell (esempio 25) viene decrementato del valore di "CONTATORE_TIMEOUT" per volta -->inizialmente a 3.
unsigned int polling_intervall = 2000; // 2 secondi. --> inizialmente a 3000.
String inputString = "";         // stringa conenente dati in arrivo
boolean stringComplete = false;  // fino a quando la scringa è incompleta  
unsigned int SpeedValue=0; 

void setup() {

    pinMode(PswChange, INPUT);
    digitalWrite(PswChange, HIGH); // abilito resistenza di pull-up sull'ingresso per password change
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT); 
    pinMode(LedPin, OUTPUT);
    pinMode(Connessione, OUTPUT); // uscita connessione BT presente.
    pinMode(allarmPin, OUTPUT); // imposto pin 13 come uscita allarme normalmente a 0.
    digitalWrite(Connessione, LOW);
    digitalWrite(allarmPin, LOW);
    digitalWrite(LedPin, LOW);
  Timer1.initialize(200000); // set a timer of length 200000 microseconds (or 0.2 sec - or 5Hz => the led will blink 5 times, 5 cycles of on-and-0, per second)
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
    
    //switch module to AT mode
    //digitalWrite(modeBtPin, HIGH); // piedino 9 come mode pin del modulo BT: 1= AT command e 0=serial data
    //Serial.begin(38400);
    //Serial.println("Enter AT commands:");
    //BTserial.begin(38400);  // HC-05 default speed in AT command more
    BTserial.begin(9600); 
    
    Serial.begin(9600);
    Serial.println("Inizializzazione");
    
/*
 * Picture of Steps To Switch The HC-05 Into Command Mode
For the HC-05 module to switch to AT command mode, the HC-05 pin 34 (often referred to as the Key pin) needs to pulled HIGH but in a certain order of events explained below. When the HC-05 enters the AT command mode, it will communicate at 38400 baud rate. Follow these steps in the stated order to switch to the HC-05 to AT command mode. 

1- Wire the HC-05 and Arduino Uno per instructions.
2- BEFORE YOU CONNECT THE ARDUINO TO THE USB remove the VCC (power) red wire from the HC-05 so it's not getting any power from the Arduino. All other wires are still connected.
3- Now connect the Arduino Uno to the USB cable extended from your PC.
4- Make sure the HC-05 module is NOT PAIRED with any other Bluetooth device.
5- Re-connect the Arduino Uno 5V wire to the HC-05's VCC (5V power) pin.
6- The HC-05 LED will blink on and 0 at about 2 second intervals. Now the HC-05 is in AT command mode ready to accept commands to change configuration and settings.
7- To test if everything is wired correctly,  open the Serial Monitor from the Arduino IDE and type "AT" and click SEND. You should see an "OK"
8- If you don't see an "OK" check your wiring.
 */
    BTserial.listen(); // abilito la recezione tramite porta seriale BTserial

    //Leggo i valori in EEPROM e li immagazzino nelle variabili locali...
    data =  EEPROM.read(EE_Timeout);
    delay(100);


    if( data != 255)
    {

      ContatoreTimeout = Timeout = data;
    }
    else // imposto valore di default a 120 secondi
    {
      ScriviEEPROM (EE_Timeout, DefaultTimeout);

    }

    BTserial.print('S');
    BTserial.print('T'); 
    BTserial.println(Timeout, DEC);
    data = EEPROM.read(EE_speed);

    if( data <= 50)
    {
      Speed = data;
    }
    else
    {
       Speed = 10; // non leggendo valore in eeprom imposto per precauzione un valore di 10 km/h
      }

     delay(200);
     BTserial.print('S');
     BTserial.print('S'); 
     BTserial.println(Speed, DEC); //acciunge carriage return con println e invia dato a client BT della velocità memorizzata in eeprom
     delay(200);
    data =  EEPROM.read(EE_PswFlag);
    Serial.print("Flag allarme = ");
        Serial.println(data, DEC);
    if (data == 255)
    {
      EEPROM.write(EE_PswFlag, 0); // per la prima accensione setto eeprom flag allarme a 0
      PswFlag = false;
      Serial.println("PswFlag = false");
      }
      else
      {
              PswFlag = (boolean)data; // se è avvenuto allarme, appena alimento il circuito pongo il relè in condizione di allarme ovvero diseccitato.
              if(PswFlag)
              {
             digitalWrite(allarmPin, LOW);
             Timer1.attachInterrupt(timerIsr);
             digitalWrite(LedPin, HIGH);
             // setto flag EE_PswFlag
             ScriviEEPROM(EE_PswFlag,1);
             Serial.println("ALLARME ON");
                
              }
              else
              {
            Timer1.detachInterrupt();
            digitalWrite(allarmPin, HIGH);
            digitalWrite(LedPin, LOW);      
            Serial.println("ALLARME OFF");
            // resetto flag EE_PswFlag
            ScriviEEPROM(EE_PswFlag,0);

                
                }
        
        }
        Serial.print("Flag allarme = ");
        Serial.println(PswFlag, DEC);
    delay(100);
}

void loop() {
    Serial.println();

   if(PswFlag)// letto da eeprom
              {
                allarme(On);
                
              }
              else
              {
                allarme(Off);
                
                }
    ConnessionePresente = false;
    Serial.println("Testo la connessione...");
    TestConnection();
    if(ConnessionePresente)
      {
        ContatoreTimeout = Timeout;
        allarme(Off);
        digitalWrite(Connessione, HIGH);
        delay(200);
        digitalWrite(Connessione, LOW);
        Serial.print("Reset timeout al valore di: ");
        Serial.println(ContatoreTimeout, DEC);
        
      }
      else
      {
          
          if(ContatoreTimeout>CONTATORE_TIMEOUT)// decremento di CONTATORE_TIMEOUT secondi per volta
          {
          ContatoreTimeout = ContatoreTimeout-CONTATORE_TIMEOUT;  //decremento di 1 il contatore 
          Serial.print("Contatore timeout: ");
          Serial.println(ContatoreTimeout,DEC);
          }
          else  // se sono qui il contatore è quasi a 0
          {
               //data = EEPROM.read(EE_speed);
             if( Speed > 50)
               {
                Speed = 10;// velocita massima di default
                }
           ActualSpeed = TestSpeed ();  // Questa velocità viene controllata quando, ormai non più presente la connessione BT, si vuole spegnere il veicolo in sicurezza!!
           // imposto flag allarme = 1 perchè alla successiva inizializzazione va direttamente in allarme..

            ScriviEEPROM(EE_PswFlag,1);
           
            if(ActualSpeed <= Speed)
            {
       //allarme...

               allarme(On);
              
              }
          }
      }
      delay(polling_intervall);

  }
  
  /// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{
    // Toggle LED
    digitalWrite( LedPin, digitalRead( LedPin ) ^ 1 ); //XOR operato per bit
    
    /*  xor table
     *  A B   Y
     *  0 0   0
     *  0 1   1
     *  1 0   1
     *  1 1   0
     */
}

void allarme(boolean all)
{
        //if(digitalRead(CmdIn) == LOW)
       // Serial.println("routine allarme");
        if(all == 0)
      {
          
        if(PswFlag == 1) // se era già in allarme...
          {
            digitalWrite(allarmPin, HIGH);
            Timer1.detachInterrupt();
            digitalWrite(LedPin, LOW);      
            Serial.println("ALLARME OFF");
            // resetto flag EE_PswFlag
            ConnessionePresente = true;
            ScriviEEPROM(EE_PswFlag,0);
            PswFlag = 0;
          }


        
        }
        if(all == 1)
        {
          if(PswFlag == 0) // se non era in allarme...
          {
             digitalWrite(allarmPin, LOW); //relè diseccitato con allarme attivo
             Timer1.attachInterrupt(timerIsr);
             digitalWrite(LedPin, HIGH);
             // setto flag EE_PswFlag
             ScriviEEPROM(EE_PswFlag,1);
             PswFlag = 1;
             Serial.println("ALLARME ON");
          }

        }
  }

void TestConnection()
{
      ConnessionePresente = false;
      RxData(); // qui ricevo il dato...
   // Incomincio a verificare he il link seriale sia stabilito...

      if (PswFlag) // se è setteto il flag di richiesta password...
      {
         BTserial.print('P');
         BTserial.print('S');
         BTserial.println('W');
         Serial.println("Test inviato a client...PSW");
         ContatoreTimeout = 0;
       }
       else
       {
          BTserial.println('?');
          Serial.println("Test inviato a client...?");      
          delay(100);// prepare for next data ...
        }
  }

boolean CheckPassword(char *psw)
{
  boolean check = false; // false = password discordanti !!!!
  if(digitalRead( PswChange ) == HIGH)
  {
  for (int i =0; i<=5; i++)
  {
    EE_Password[i] = EEPROM.read(i);
    }

  for (int i=0; i<=5;i++)
  {
    Serial.print("Test carattere ");
    Serial.println(i);
    Serial.print(psw[i]);
    Serial.print(" ? ");
    Serial.println(EE_Password[i]);
    
    if(psw[i] == EE_Password[i])
      {
        
      check = true;
      Serial.print("Carattere n "+ i);
      Serial.println(" "+  EE_Password[i]);
      }
      else
      {
        check = false;
        return check;
       }
     }
  
      return check;
  }
  else // cambio la password...
  {
    for (int i =0; i<=5; i++)
      {
        EEPROM.write(i, Password[i]);
        delay(100);
      }
      return true;
    }

  
  }
  
void SendMeasuredRealSpeed () // Velocità spedita verso cell quando la connessione è presente
{
  //unsigned int MeasuredSpeed = 255; // max valore di default
  //MeasuredSpeed = ((map( analogRead(SpeedPin), 0, 1023, 0, 255))*5);

  double MeasuredSpeed = 0; 
  unsigned int nCampioni = 10;
  for(int i = 0; i<nCampioni; i++)
  {
    MeasuredSpeed += analogRead(SpeedPin);
    delay(25);
    }
  MeasuredSpeed = MeasuredSpeed/nCampioni;
            
            Serial.print("Velocita' spedita ");
            Serial.println(MeasuredSpeed,DEC);
            
            BTserial.print('S');
            BTserial.print('P');
            BTserial.println(MeasuredSpeed,DEC);
            delay(50);
}
unsigned int TestSpeed () // Questa velocità viene controllata quando, ormai non più presente la connessione BT, si vuole spegnere il veicolo in sicurezza!!
{
  //unsigned int MeasuredSpeed = 255; // max valore di default
  //MeasuredSpeed = ((map( analogRead(SpeedPin), 0, 1023, 0, 255))*5);

   double MeasuredSpeed = 0; 
  unsigned int nCampioni = 10;
  for(int i = 0; i<nCampioni; i++)
  {
    MeasuredSpeed += analogRead(SpeedPin);
    delay(25);
    }
  MeasuredSpeed = MeasuredSpeed/nCampioni;

  
           Serial.print("Velocita' limite: ");
           Serial.println(Speed,DEC);
           Serial.print("Velocita' rilevata: ");
           Serial.println(MeasuredSpeed,DEC);
           delay(50);
//**********
            BTserial.print('S');
            BTserial.print('P');
            BTserial.println(MeasuredSpeed,DEC);
           
//*********           
  if(MeasuredSpeed <= Speed) // velocità letta in EEPROM
  {

    return MeasuredSpeed;
    
    }
    else
    {
       return 255; // sostanzialmente non faccio intervenire allarme in quanto non conosco l'attuale velocità del veicolo
      
    }
  
  
  }

void RxData() {
        int index = 0;
        char inChar;
        String TestString="";
        inputString ="";


    while (BTserial.available() > 0) // Don't read unless
    {
      TestString ="";
      do
        {
          inChar = BTserial.read();
          TestString += inChar;
        
        } 
        while(inChar != '\n');

          // Qui ho ricevuto la prima stringa contenuta nel buffer dove potrebbero essercene n. Inizio con l'analisi della prima e poi proseguiro ad analizzare
          // le altre fino al completo svuotamento del buffer

//--------------------------------------------------------------------------------------------------------------

          // Testo se la stringa ricevuta inizia con "SLO"
          if( TestString.startsWith("SLO"))   // Set Speed da valore ricevuto dal cell in KM/h
              {
                inputString = "SLO";//(TestString.substring(3,5));//.toInt());
                Serial.println("Ricevuto SLO");
                SetSpeed();                
                BTserial.print('S');
                BTserial.print('L');
                BTserial.print('O');
  
                 if( Speed < 10)
                  {
                     BTserial.print(0,DEC);
                     BTserial.print(0,DEC);
                     BTserial.println(0,DEC);
                  }
                  else
                  {
                  
                BTserial.println(Speed, DEC); // spedisco verso client nuovo valore velocità appreso come echo
                  }
                rxSLO(inputString);
              }
//--------------------------------------------------------------------------------------------------------------
            // Testo se la stringa ricevuta inizia con "PSW"
            
            if( TestString.startsWith("PSW"))   // Test password
            {
              Serial.print("Ricevuto PSW: ");
              inputString = (TestString.substring(3,9));//.toInt());
              Serial.println(inputString);
              inputString.toCharArray(Password, 7);
              Serial.println(Password);
              // devo testare la password....
              passwordOK = CheckPassword(Password);
              if(passwordOK)
                 {
                  //Serial.println("Password corretta !!");
                    BTserial.print('P');
                    BTserial.print('S');
                    BTserial.print('W');
                    BTserial.println('=');
                    rxPSW(inputString); // allora potrò disinserire allarme...
                 }
               else
                  {
                  Serial.println("Password sbagliata !!");
                  rxPSWko(inputString);
                  }
            }
//--------------------------------------------------------------------------------------------------------------
             // Testo se la stringa ricevuta inizia con "SS"
             
            if( TestString.startsWith("SS"))   // Set Speed
            {
              inputString = (TestString.substring(2,4));//.toInt());
              Serial.print("Ricevuto SS");
              Serial.println(inputString);
              BTserial.print('S');
              BTserial.print('S');
               if( inputString.toInt() < 5)
                  {
                     BTserial.print(0);
                     BTserial.println(0);
                  }
               BTserial.println(inputString.toInt());
               rxSS(inputString);
            }
//--------------------------------------------------------------------------------------------------------------

             // Testo se la stringa ricevuta inizia con "ST"

            if( TestString.startsWith("ST"))   // Set Timeout
            {
                inputString = (TestString.substring(2,5));//.toInt());
               
                Serial.println(inputString);
                // ripristino il valore in EEPROM se ricevuto valore errato
                BTserial.print('S');
                BTserial.print('T');
                if( inputString.toInt() < 100 && inputString.toInt() > 5)
                 {
                     BTserial.println(inputString.toInt());
                      Serial.print("Spedito ST "+ inputString.toInt());
                  }
                 else
                 {
                    BTserial.println(0);
                  } 
                rxST(inputString);
                
            }
//--------------------------------------------------------------------------------------------------------------

             // Testo se la stringa ricevuta inizia con "!"            

            if( TestString.startsWith("!"))   // Set Speed
              {
                //Serial.println("! testString: "+ TestString);
                  rxPollingOK();
              }            
     } // close while cicle

      delay(100);// prepare for next data ...
  }


void rxSLO(String dato)
  {
    // significa che la ho ricevuto la stringa "SETTA SPEED con valore di VELOCITA' ATTUALE DEL VEICOLO" e il risultato è stato messo nella variabile "inputString"
    Serial.println("SLO ricevuto..");
    allarme(Off);
    // resetto EE_Timeout
    TimeoutAvvenuto = false;
    ConnessionePresente = true;
    SetSpeed();                

   }

   
void rxST(String dato)
  {
      // significa che la ho ricevuto la stringa "SETTA TIMEOUT"
      Serial.println("ST ricevuto..");
      allarme(Off);
      // resetto EE_Timeout
      TimeoutAvvenuto = false;
      ConnessionePresente = true;
      // a questo punto inputString contiene i dato timeout ricevuto dal cell
      if (inputString.toInt()<= 250 && inputString.toInt()>= 10)
      {
          SetTimeOut(inputString.toInt());                            
          Serial.println("Nuovo contatore timeout " + inputString);
      }
      delay(100);
  }

void rxSS(String dato)
  {
      Serial.println("SS ricevuto..");
      allarme(Off);
      // resetto EE_Timeout
      TimeoutAvvenuto = false;
      ContatoreTimeout = Timeout;
      // Scrivo nuovo valore di velocità in EEPROM
      SetSpeedFromCell((unsigned int)inputString.toInt());
      ConnessionePresente = true;
  }
  
void rxPSW(String dato)
{

  
      TimeoutAvvenuto = false;
      Serial.println("PSW corretta ricevuta...");
      TimeoutAvvenuto = false;
      ConnessionePresente = true;
      allarme(Off);
      ContatoreTimeout = Timeout;
  
  }
void rxPSWko(String dato)
  {
       Serial.println("PSW errata...");
  }

  
void rxPollingOK()
  {
    if (PswFlag == 0)
      {
      allarme(Off);
       TimeoutAvvenuto = false;
      ConnessionePresente = true;
      SendMeasuredRealSpeed();
      delay(100);
      Serial.println("Echo ricevuto da client...");
      inputString ="";
      }
     
  }


void SetSpeedFromCell(unsigned int sp)
{
  Speed = sp;
  ScriviEEPROM(EE_speed,sp);
          Serial.print("Set speed to: ");
          Serial.println(sp);
  }

void SetSpeed()
{
  // read the value from the speed:
//Utilizzo il comando MAP per convertire il valore letto dall'ingresso analogico in 10 bit in un byte (ovvero 8 bit)
// esempio:  val = map(input, 0, 1023, 0, 255); converte il valore di input 0-1023 in un valore di output 0-255 e lo inserisce nella variabile val.
  
  
  //SpeedValue = ((map( analogRead(SpeedPin), 0, 1023, 0, 255))*5);

  double SpeedValue = 0; 
  unsigned int nCampioni = 10;
  for(int i = 0; i<nCampioni; i++)
  {
    SpeedValue += analogRead(SpeedPin);
    delay(25);
    }
  SpeedValue = SpeedValue/nCampioni;

  

// aggiungere routine che limiti la velocità impostabile ad un valore reale inferiore ai 50 Km/h anche se si tenta di settrla ad una superiore mediante
// il tasto "SetSpeed" del cellulare...
  delay(50);
  if (Speed != SpeedValue)  // prima effettuo una lettura dell'indirizzo e se il valore al suo interno corrisponde già al valore che intendo scrivere, evito di farlo...
  {
  ScriviEEPROM(EE_speed,SpeedValue);
  }
          Serial.print("Setto speed al valore del veicolo: ");
          Serial.println(SpeedValue);
           Serial.println("Veccho valore: " + Speed);
           Speed = SpeedValue;

  }
 void SetTimeOut(int TO)
{

       if (TO != Timeout)  // prima effettuo una lettura dell'indirizzo e se il valore al suo interno corrisponde già al valore che intendo scrivere, evito di farlo...
          {
            ScriviEEPROM(EE_Timeout, TO);
            ContatoreTimeout = Timeout = TO;
            delay(200);
            Serial.print("Setto timeout a ");
            Serial.println(TO, DEC);
          }

  
} 
void ScriviEEPROM (int addr, byte valueToWrite)
{
  // prima effettuo una lettura dell'indirizzo e se il valore al suo interno corrisponde già al valore che intendo scrivere, evito di farlo...
  
    EEPROM.write(addr, valueToWrite);
    delay(100); //ritardo necessario per la scritura in eeprom del dato
  }
//Called when data is available. Use Serial.read() to capture this data.

void serialEvent()
{
  while (Serial.available()) 
  {
      // get the new byte:
       char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n')
      {
        stringComplete = true;
      }
  }
}

