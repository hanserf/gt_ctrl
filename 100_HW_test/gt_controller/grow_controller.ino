#include "RTClib.h"
//#include "growfun.h"

//-----------------------------------------------------------
//                Pinout:
//-----------------------------------------------------------
// D2 - Relay LED Driver 1
// D3 - PWM LED Driver 1
// D4 - Relay LED Driver 2
// D5 - PWM LED Driver 2
// D7 - One Wire DS18S20 - 2 stk ADDR??
// Vin - 12V 1A - To LVL Shifter VDDB and BUFFER
// 5V - To Relay and LVL Shifter VDDA
//-----------------------------------------------------------
//                Global Variables
//-----------------------------------------------------------
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday","Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const int LED_RB = 2;
const int LED1_PWM = 3;
const int LED_COB = 4;
const int LED2_PWM = 5;
//const int ONEWIRE = 7;
int globalCntr = 0;
DateTime globalFutureEvent;


String getFullTimeString(DateTime aTime){
  String currTime;
  currTime = String(aTime.year(), DEC) + '/';
  currTime += String( aTime.month(), DEC) + '/';
  currTime += String( aTime.day(), DEC) ; 
  currTime += " (";
  currTime += daysOfTheWeek[aTime.dayOfTheWeek()];
  currTime +=  ") ";
  currTime += String( aTime.hour(), DEC) + ':';
  currTime += String( aTime.minute(), DEC ) + ':';
  currTime += String( aTime.second() , DEC );
  return currTime;
}
String getClockString( DateTime aTime){
  String aClk;
  aClk = String( aTime.hour(), DEC) + ':';
  aClk += String( aTime.minute(), DEC ) + ':';
  aClk += String( aTime.second(), DEC ); 
  return aClk;
}
void setFutureEvent(DateTime aTime){
  globalFutureEvent = aTime;
}
DateTime getFutureEvent(){
  return globalFutureEvent;
}

int isFutureevent(DateTime now){
  int rc = 0;
  if(now >= getFutureEvent()){
    rc = 1;
  }
  return rc;
}


void setup () {
  //GPIO
  pinMode(LED_RB, OUTPUT);
  digitalWrite(LED_RB,HIGH); //Active LOW
  pinMode(LED_COB, OUTPUT);
  digitalWrite(LED_COB,HIGH); //Active LOW
  pinMode(LED1_PWM, OUTPUT);
  pinMode(LED2_PWM, OUTPUT);
  //pinMode(ONEWIRE, INPUT);
  //Read Time on wake;
  while (!Serial) {
    delay(1);  // for Leonardo/Micro/Zero
  }

  Serial.begin(115200);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2019, 7, 29, 22, 01, 0));
  }
}

void loop () {
    int toggle_ledstate = 0;
    DateTime now = rtc.now(); 
    String myTD = getFullTimeString(now);
    Serial.println("My Full time");
    Serial.print(myTD);
    Serial.println();
    String myTime = getClockString(now);
    Serial.println("My Clock");
    Serial.print(myTime);
    Serial.println();

    // calculate a date which is 7 days, 12 hours and 30 seconds into the future
    //DateTime future (now + TimeSpan(7,12,30,6));
    
    
    if( globalCntr % 2 == 0 ){
      Serial.println("Writing LED HIGH");
      //digitalWrite(LED_COB,HIGH);
      //digitalWrite(LED_RB,HIGH);
      digitalWrite(LED1_PWM , HIGH);
      digitalWrite(LED2_PWM , HIGH);
    }
    else{
      Serial.println("Writing LED LOW");
      //digitalWrite(LED_COB,LOW);
      //digitalWrite(LED_RB,LOW);  
      digitalWrite(LED1_PWM , LOW);
      digitalWrite(LED2_PWM , LOW);
    }
    globalCntr++;
    Serial.println();
    
    delay(3000);
}
