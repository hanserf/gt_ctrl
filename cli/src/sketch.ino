#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//#include "growfun.h"
#define ONE_WIRE_BUS 7

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
//DS18S20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors (&oneWire);
int numberOfDevices;
DeviceAddress tempDeviceAddress;
//RTC
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday","Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//Global vars
const int LED_RB = 2;
const int LEDRB_PWM = 3;
const int LED_COB = 4;
const int LEDCOB_PWM = 5;

int pwm_adjust = 0;

void setup () {
  //GPIO
  pinMode(LED_RB, OUTPUT);
  digitalWrite(LED_RB,HIGH); //Active LOW
  pinMode(LED_COB, OUTPUT);
  digitalWrite(LED_COB,HIGH); //Active LOW
  pinMode(LEDRB_PWM, OUTPUT);
  pinMode(LEDCOB_PWM, OUTPUT);
  pwm_set(LEDRB_PWM,0);
  pwm_set(LEDCOB_PWM,0);
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
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  printDallasDevices();
}

void loop () {
    DateTime now = rtc.now(); 
    String myTD = getFullTimeString(now);
    Serial.println("My Full time");
    Serial.print(myTD);
    Serial.println();
    String myTime = getClockString(now);
    Serial.println("My Clock");
    Serial.print(myTime);
    Serial.println();
    //Get temperatures
    sensors.requestTemperatures(); // Send the command to get temperatures
    printTemperatures();
    if( globalCntr % 2 == 0 ){
      Serial.println("Writing LED RGB LOW, COB HIGH");
      digitalWrite(LED_RB,LOW);  
      digitalWrite(LED_COB,HIGH);
    }
    else{
      Serial.println("Writing LED RGB HIGH, COB LOW");
      digitalWrite(LED_RB,HIGH);
      digitalWrite(LED_COB,LOW);
    }
    globalCntr++;

    pwm_set(LEDRB_PWM,pwm_adjust);
    pwm_set(LEDCOB_PWM,255-pwm_adjust);
    if(pwm_adjust >= 255){
      pwm_adjust = 0;
    }
    else{
      pwm_adjust +=10;
    }


    Serial.println();
    delay(10000);
}

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

void printTemperatures(){
// Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
		
		// Output the device ID
		Serial.print("Temperature for device: ");
		Serial.println(i,DEC);

    // Print the data
    float tempC = sensors.getTempC(tempDeviceAddress);
    Serial.print("Temp C: ");
    Serial.print(tempC);
    Serial.print(" Temp F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
    } 	
  }

}

void printDallasDevices(){
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
		} else {
		  Serial.print("Found ghost device at ");
		  Serial.print(i, DEC);
		  Serial.print(" but could not detect address. Check power and cabling");
		}
  }
}
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void pwm_set(const int myPin, int value){
  if(value >255){
    value = 255;
  }
  if(value < 24 || value < 0){ //PWM DC is not allow to be less than 10% and a neg value makes no sense. 
    value = 0;
  }
  analogWrite(myPin,value);
}
