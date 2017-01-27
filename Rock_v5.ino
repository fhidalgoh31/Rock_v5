// v4.0  with potbot v4 board
//
//includes events for wet switch
// ROCKBLOCK    10  - RX
//              11  - TX
//               2  - Powermode 
//GPS            12  - RX
//               13  - TX
//Switch         A3  - Input:   Nothing connected --> 5V (off),  connected to pin 3 --> GND (trigger)
//               3  - GND   - 
//Open Log
//#include </Time/src/Time.h>     
#include "Time.h"     
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <IridiumSBD.h>
#include <EEPROM.h>
#define EEPROM_addrs  1

#define SW_GPS  4
#define RX_GPS  12
#define TX_GPS  13
#define SW_TOL  3
#define RX_OL  11
#define TX_OL  10
#define I_HUM  A5    // air --> 1, water --> 0
#define SW_RCK A4
#define RX_RCK  8
#define TX_RCK  9

#define BIG_SLEEP 10                     // in minutes
//////// ROCKBLOCK CONFIG //////////////////////////////////////////////////////////
#define MIN_SIG_CSQ 0                       // Minimum signal 0 -->5 ,    Time in seconds
#define AT_TIMEOUT 15                       //  30 default Wait for response timeout
#define SBDIX_TIMEOUT 15                  // 300 default send SBDIX loop timout
#define SBDIX_LOOP 1                      // number of loops SBDIX_TIMEOUT, it will read the GPS in between
////////////////////////////////////////////////////////////////////////////////////

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define GPS_TIMEOUT 1    // GPS timeout in minutes
#define GPS_THRESHOLD 1



SoftwareSerial myRock(RX_RCK, TX_RCK); // RX, TX
IridiumSBD isbd(myRock, SW_RCK);
SoftwareSerial myGPS(RX_GPS, TX_GPS); // RX, TX       
SoftwareSerial ol_serial(RX_OL, TX_OL); // RX, TX   // OL  

TinyGPS gps;

int signalQuality = 0 ;   //signal quality  for rockblock
byte ol_status=0;     //ol status
bool first;   //first time runing 
bool screen=0;   // connects OpenLog directly
bool logger=0;   //logs acceleromter
static bool gps_write=0;    
char gps_thresh=0;     // threshold of valid readings
time_t GPS_timed;       // global gps

byte state=0;    // 1: in air    3: deployed    5: in water    7: recovered
//dummy    ,Video_time [min], temp_freq [min], acc_threshold [%], GPS_time out [min]
byte parameters[5]={0,15,60,20,5};

int get_gps(float *latitude, float *longitude);
int command(char data);
void digitalClockDisplay();
void printDigits(int digits);
int ol_cmd();
int ol_append();
int ol_potbot_parameters();
int sequence();
void ol_timestamp();
void ol_printDigits(char separator, int digits);
float latitude,longitude;
void ol_logevent(int event);

void setup() {

  pinMode(SW_GPS,OUTPUT);
  pinMode(SW_TOL,OUTPUT);
  pinMode(SW_RCK, OUTPUT);
  pinMode(I_HUM,INPUT);
 
  // Open serial communications and wait for port to open:
  Serial.begin(4800);
  delay(100);

//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  // set the data rate for the SoftwareSerial port
  myRock.begin(19200);   // 19200 rockblock
  myGPS.begin(4800);   // 4800 GPS
  ol_serial.begin(4800);
  delay (1000);

   if(Serial)
    Serial.println("Rockblock v 4.0");
 
  delay(1000);
  ol_potbot_parameters();
  state=0;
    
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  myRock.listen();
  isbd.setPowerProfile(1);
  isbd.begin();
  isbd.setMinimumSignalQuality(MIN_SIG_CSQ);   // set to 0 for continous send
  isbd.adjustATTimeout(AT_TIMEOUT);                       // Wait for response timeout
  isbd.adjustSendReceiveTimeout(SBDIX_TIMEOUT);               // send SBDIX loop timout

  int get_sig;
  int err = isbd.getSignalQuality(get_sig);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is:   ");
  Serial.println(get_sig);

  Serial.print("Waiting for Wetswitch");
}

void loop() {

  char data;
  static bool cur_hum, prev_hum;
    
    
  if (screen)                        // OPENLONG SCREEN INTERFACE
       {  
        ol_serial.listen();
        if (ol_serial.available()) {
        Serial.write(ol_serial.read());
        }
        if (Serial.available()) {
        ol_serial.write(Serial.read());
         }
      }
  else {
          // DEBUG interface
      if (Serial.available() && (data   = Serial.read()) != 13){
      Serial.print("new command");
      Serial.write(data);
      Serial.print("   :");
      command(data);
      }
      // high priority log humdity sensor
      cur_hum=digitalRead(I_HUM);
      if (cur_hum!=prev_hum) 
        {
          if (cur_hum==1) ol_logevent(9);    // E9 in air
          else ol_logevent(8);               //E8 in water
          prev_hum=cur_hum;
        }
        
      sequence();
    }
}


/// get_gps:
//Read GPS for a second and 
//return 1: valid sentence received 
//return 0: no valid
// latitude and longitude as floats
//SYNC TIME
int get_gps(float *latitude, float *longitude)
{
   myGPS.listen();
   extern bool gps_write;
   bool newData = false;
   int year;
   byte month, day, hour, minutes, second, hundredths;
   unsigned long fix_age;
   
   // For one second we parse GPS data and report some key values
  Serial.println("Waiting for GPS");
  
   
   for (unsigned long start = millis(); millis() - start < 2000;)
   {
     while (myGPS.available())
     {
     char c = myGPS.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
     if (gps.encode(c)) // Did a new valid sentence come in?
     {
      newData = true;
      gps_thresh++;
     }
     }
   }

   if (newData && gps_thresh>GPS_THRESHOLD) // threshold # of good sentences
   {
    unsigned long age;
    gps.f_get_position(latitude, longitude, &age);
    gps.crack_datetime(&year, &month, &day,
    &hour, &minutes, &second, &hundredths, &fix_age);
    setTime(hour,minutes,second,day,month,year);
    setTime(now()+28800);
    Serial.print("GPS received, messages:");
    Serial.println(gps_thresh,DEC);
        
    gps_thresh=0;
    
    return 1;
   }
return 0;
  
 }
/////////////////////////////////////////////////////////
/////////////// POTBOT CODE /////////////////////////////
/////////////////////////////////////////////////////////

int command(char data){
  switch (data){

    case 'o': 
      
      ol_cmd();
      break;

    case 'g': 
      digitalWrite(SW_GPS, HIGH);
      Serial.println("GPS on...");
      myGPS.listen();
  break;
  
  case 'G': 
      digitalWrite(SW_GPS, LOW);
      Serial.println("GPS off...");
  break;
  
    case 'h':
      Serial.print(digitalRead(I_HUM),DEC);
      Serial.print("  hum\n");
      break;

    case 'w':
      digitalWrite(SW_TOL, HIGH);
      Serial.print("Open log writing test\n");
      delay(1000);
      for(int i = 1 ; i < 10 ; i++) {
      ol_serial.print(i, DEC);     
      ol_serial.println(":abcdefsdff-!#");
      delay(100);
      }
      
      delay(1000);
      Serial.print("finish writing\n");
      digitalWrite(SW_TOL, LOW);

      break;

     case 's':                                   //screen
     screen=1;
     ol_serial.listen();
     digitalWrite(SW_TOL, HIGH);
     ol_cmd();
     break;


     case 'l':                                   //log
     digitalWrite(SW_TOL, HIGH);
     delay(500);
      Serial.print("  Logger: ");
      Serial.print(!logger,DEC);
      Serial.println("");
      ol_serial.listen();
    
    if (!logger){
       for (int i=0; i<20 ; i++){
         ol_serial.println("");
         ol_serial.write(26);
         ol_serial.write(26);
         ol_serial.write(26);
         ol_serial.println("");
         if (ol_serial.available())
        //Serial.println(ol_serial.read());
          if(ol_serial.read()=='>')
            {Serial.print(i, DEC); Serial.println(">"); break;}
         }
       delay(20);
       int value = EEPROM.read(EEPROM_addrs);
       EEPROM.write(EEPROM_addrs, value+1);
       ol_serial.println("");
       delay(10);
       ol_serial.println("");
       delay(10);
       ol_serial.print("append acc_");
       if (value<10) ol_serial.print("0");
       ol_serial.print(value,DEC);
       ol_serial.println(".txt");
       delay(10);
    }else
    {
         ol_serial.write(26);
         ol_serial.println("");
         delay(10);
    }
    logger=!logger;
     break;

     case 'r':
      EEPROM.write(EEPROM_addrs, 0);
      Serial.print("  reset logger file to new01.txt\n");
      break;

    case 13:
    break;

    case 'x':
      Serial.println("Start sequence");
      state=2;
      break;
      
    default:
      Serial.print("  error command\n");
      break;
    }
  return 1;
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}


void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void ol_timestamp(){
  // digital clock display of the time
  ol_printDigits(0,day());
  ol_printDigits('/',month());
  ol_printDigits('/',year());
  ol_printDigits(';',hour());
  ol_printDigits(':',minute());
  ol_printDigits(':',second());
  ol_serial.print(';'); 
}

// separator = 0 --> doesn't print
void ol_printDigits(char separator, int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if (separator) ol_serial.write(separator);
  if(digits < 10)
    ol_serial.print('0');
  ol_serial.print(digits);
}


int ol_cmd(){
if (!ol_status)
{ 
  char temp;
  digitalWrite(SW_TOL, HIGH);
  Serial.print(digitalRead(SW_TOL),DEC);
  Serial.print("Open log ON\n");
  ol_serial.listen();
  delay(100);
  for (int i=0; i<100 ; i++){
      delay(5);
      ol_serial.write(26);
      delay(5);
      for (int z=0; z<10 ; z++)
        if (ol_serial.available())
          //Serial.println(ol_serial.read());
          if(ol_serial.read()=='>')
          { Serial.print("try: "); Serial.print(i, DEC); Serial.println(">");   delay(20); 
            ol_serial.println("");
            for (int j=0; j<200; j++)          //flush
              if (ol_serial.available())
                {temp=ol_serial.read(); 
                //Serial.print(temp);
                }
            delay(5);
            Serial.println("");
            Serial.println("OL CMD_MODE");
            return 1;
          }
        delay(5);
        if (i>=99) { Serial.println("error openlong cmd"); return 0;}
      }
     ol_status=1; 
    }
} 


int ol_append(){
  if (ol_status!=2)
  {
      ol_cmd();
      ol_serial.println("");
      delay(10);
      ol_serial.println("append log.txt");
      delay(10);
      Serial.println("Openlong ready");
      ol_status=2;
      return 1;
  }
}
int ol_potbot_parameters(){
  int  par_index=0;  
  char character;
  int temp=0;
  ol_cmd();
  delay(200);
  ol_serial.println("read potbot");
  for (int i=0; i< 200; i++) {
    if (ol_serial.available()) 
    {
      character= ol_serial.read(); //Serial.write(character);
      if (character=='.') {Serial.println("parameters loaded");digitalWrite(SW_TOL, LOW);return 1; break;}
        if ((character>= '0') &&  (character<= '9'))
          temp= temp*10 + character - '0';
        else if (character==',')
        {
          parameters[par_index]=temp;
          par_index ++;
          temp=0;
          Serial.print("    paramter[");
          Serial.print(par_index-1);
          Serial.print("] :");
          Serial.println(parameters[par_index - 1]);
        }
     }
     delay(10);
    }
   Serial.println("Error OL_parameters");
   digitalWrite(SW_TOL, LOW);
   return 0;
}

int sequence(){
  static time_t temp1, temp2;
  
  switch(state){

  case 0:
      myRock.listen();
      digitalWrite(SW_GPS, LOW);  //stops gps
      isbd.sleep();               //Rockblock off
      Serial.println("waiting wetswitch ...");
	  ol_logevent(1);
      state++;
      break;
  
  case 1:
      
      if (digitalRead(I_HUM))       // wait hum
      {  Serial.print("Turn GPS and Rockblock ON...");
      state++;
      }
    break;

  case 2:
//    temp1=now() + 60*parameters[4];    // gps time
    ol_logevent(2);
    delay(1000);
    myRock.listen();
    isbd.begin();                //start Rockblock
    digitalWrite(SW_GPS, HIGH);  //start gps
    state++;
  break;
  
  case 3:               // send message rock block
  {
      myRock.listen();
      String str_rock= " ";     //Create String and Capture current time
      str_rock= str_rock + "Time: " + String(day()) + "/" + String(month()) + "/" + String(year()) + "   " + String(hour())+ ":" + String(minute()) + ":" + String(second());
      str_rock= str_rock + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
      Serial.println(str_rock);
      char charBuf[100];
    ////////////////////////////////////
      // Send RockBlock
    //////////////////////////////////////
      str_rock.toCharArray(charBuf, 100) ;
      Serial.println("Message to send SBD:");
      Serial.println(charBuf);
      for (int i=0; i < SBDIX_LOOP; i++)
      {
        if (get_gps(&latitude, &longitude))
			{ol_logevent(4);
			 digitalWrite(SW_GPS, LOW);  //stops gps
			}
		String str_gps= " ";     //Create String and Capture current time
		str_gps= str_gps + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
        Serial.println(str_gps);
        myRock.listen();
        Serial.print("attempt: ");
        Serial.println(i);
        delay(100);
        int err = isbd.sendSBDText(charBuf);
        if (err != 0)
        {
          Serial.print("sendSBDText failed: error ");
          Serial.println(err);
        }
        if (err==0)                    //TX Successfully    go to beginning
        {
          ol_logevent(3);
          Serial.println("Msg sent confirmed!");
          Serial.print("Messages left: ");
          Serial.println(isbd.getWaitingMessageCount());
		  isbd.sleep();               //Rockblock off
		  Serial.println("Go to begining");
		  state=0;
          break;
        }
        
      }
	  // Message not received
	  temp1=now() + 60* BIG_SLEEP;    // gps time   save BIG SLEEP
	  ol_logevent(5);
      state++;
    break;
  }
   case 4:           //Message not received BIG sleep
  {
    if (now()>temp1)
	{		
		isbd.begin();                //start Rockblock
		digitalWrite(SW_GPS, HIGH);  //start gps
		state++;
	}

	break;
  }
	
	case 5:            //Second chance
	{
      myRock.listen();
      String str_rock= " ";     //Create String and Capture current time
      str_rock= str_rock + "Time: " + String(day()) + "/" + String(month()) + "/" + String(year()) + "   " + String(hour())+ ":" + String(minute()) + ":" + String(second());
      str_rock= str_rock + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
      Serial.println(str_rock);
      char charBuf[100];
    ////////////////////////////////////
      // Send RockBlock
    //////////////////////////////////////
      str_rock.toCharArray(charBuf, 100) ;
      Serial.println("Message to send SBD:");
      Serial.println(charBuf);
      for (int i=0; i < SBDIX_LOOP; i++)
      {
        if (get_gps(&latitude, &longitude))
			{ol_logevent(7);
			 digitalWrite(SW_GPS, LOW);  //stops gps
			}
		String str_gps= " ";     //Create String and Capture current time
		str_gps= str_gps + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
        Serial.println(str_gps);
        myRock.listen();
        Serial.print("attempt: ");
        Serial.println(i);
        delay(100);
        int err = isbd.sendSBDText(charBuf);
        if (err != 0)
        {
          Serial.print("sendSBDText failed: error ");
          Serial.println(err);
        }
        if (err==0)                    //TX Successfully    go to beginning
        {
          ol_logevent(6);
          Serial.println("Msg sent confirmed!");
          Serial.print("Messages left: ");
          Serial.println(isbd.getWaitingMessageCount());
		  isbd.sleep();               //Rockblock off
		  Serial.println("Go to begining");
		  state=0;
          break;
        }
        
      }
	  // Message not received
	  temp1=now() + 60* BIG_SLEEP;    // gps time   save BIG SLEEP
      state++;
	}
   
      case 6:           //Message not received BIG sleep
  {
    if (now()>temp1)
	{		
		isbd.begin();                //start Rockblock
		digitalWrite(SW_GPS, HIGH);  //start gps
		state=0;
	}

	break;
  }
  default:
    state=0;
    break; 
  }
}

// logs event timestamp and position
void ol_logevent(int event)
{
      ol_serial.listen();
      ol_append();
      ol_timestamp();
      ol_serial.print("E");
      ol_serial.print(event);
      ol_serial.print(";");
      ol_serial.print(latitude,6);
      ol_serial.print(';');
      ol_serial.print(longitude,6);
      ol_serial.println("");
}
