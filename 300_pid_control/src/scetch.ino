#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <stdlib.h>
#define ONE_WIRE_BUS 7

//-----------------------------------------------------------
//                Global Variables
//-----------------------------------------------------------
//DS18S20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int numberOfDevices;
DeviceAddress tempDeviceAddress;
int indoor_temp_sensor = 0, outdoor_temp_sensor = 1;
float sensor_indoor_temp, sensor_outdoor_temp;
//RTC
RTC_PCF8523 rtc;
DateTime now;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//GPIO
const int LED_RB = 4;
const int LEDRB_PWM = 5;
const int LED_COB = 2;
const int LEDCOB_PWM = 3;
int loopCntr = 0;
int pwm_control = 0;
//PID
double pid_input, pid_output, pid_setpoint;
//Aggressive regulation when far away from setpoint.
double pid_aggKi = 8, pid_aggKp = 0.8, pid_aggKd = 2;
//Presice regulation near setpoint.
double pid_consKi = 2, pid_consKp = 0.2, pid_consKd = 0.5;
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_consKp, pid_consKi, pid_consKd, DIRECT);

extern String getFullTimeString(DateTime aTime);
extern String getClockString(DateTime aTime);
extern void printTemperatures();
extern void printDallasDevices();
extern void printAddress(DeviceAddress deviceAddress);
extern void pwm_set(const int myPin, int value);
extern String float_to_String(float val);
extern String integer_to_String(int val);

void setup()
{
  //GPIO
  pwm_control = 0;
  pinMode(LED_RB, OUTPUT);
  digitalWrite(LED_RB, HIGH); //Active LOW
  pinMode(LED_COB, OUTPUT);
  digitalWrite(LED_COB, HIGH); //Active LOW
  pinMode(LEDRB_PWM, OUTPUT);
  pinMode(LEDCOB_PWM, OUTPUT);
  pwm_set(LEDRB_PWM, pwm_control);
  pwm_set(LEDCOB_PWM, pwm_control);
  while (!Serial)
  {
    delay(1); // for Leonardo/Micro/Zero
  }
  Serial.begin(115200);
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  if (!rtc.initialized())
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  now = rtc.now();
  // Start OneWire sensor
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Locating devices... Found: ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  printDallasDevices();
  sensors.getAddress(tempDeviceAddress, indoor_temp_sensor);
  sensor_indoor_temp = sensors.getTempC(tempDeviceAddress);
  pid_setpoint = 22.0; //Deg centigradess
  pid_input = double(sensor_indoor_temp);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0, 255.0);
}

void loop()
{
  loopCntr++;
  now = rtc.now();
  String myTD = getFullTimeString(now);
  String Serial_Message = "";
  Serial_Message += ("Run: " + integer_to_String(loopCntr) + "\r\n");
  Serial_Message += (myTD + "\r\n");
  /*
      Get sensor temperatures
    */
  sensors.getAddress(tempDeviceAddress, indoor_temp_sensor);
  sensor_indoor_temp = sensors.getTempC(tempDeviceAddress);
  sensors.getAddress(tempDeviceAddress, outdoor_temp_sensor);
  sensor_outdoor_temp = sensors.getTempC(tempDeviceAddress);
  Serial_Message += ("T_indoor = " + float_to_String(sensor_indoor_temp) + "\r\n");
  Serial_Message += ("T_outdoor = " + float_to_String(sensor_outdoor_temp) + "\r\n");
  /*
      PID Algoritm
    */
  pid_input = double(sensor_indoor_temp);
  double gap = abs(pid_setpoint - pid_input); //distance away from setpoint
  bool pid_mode_aggressive;
  if (gap < 1.5)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(pid_consKp, pid_consKi, pid_consKd);
    pid_mode_aggressive = false;
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(pid_aggKp, pid_aggKi, pid_aggKd);
    pid_mode_aggressive = true;
  }
  myPID.Compute();
  /*
      PID MESSAGE
    */
  Serial_Message += ("T_err = " + float_to_String(gap) + "\r\n");
  Serial_Message += ("PID_output = " + float_to_String(pid_output) + "\r\n");
  if (pid_mode_aggressive)
  {
    Serial_Message += ('PID_mode = AGGRESSIVE' + '\n');
    Serial_Message += ("Kd = " + float_to_String(pid_aggKd) + " , ");
    Serial_Message += ("Ki = " + float_to_String(pid_aggKi) + " , ");
    Serial_Message += ("Kp = " + float_to_String(pid_aggKp) + "\r\n");
  }
  else
  {
    Serial_Message += ("PID_mode = CONSERVATICE" + '\n');
    Serial_Message += ("Kd = " + float_to_String(pid_consKd) + " , ");
    Serial_Message += ("Ki = " + float_to_String(pid_consKi) + " , ");
    Serial_Message += ("Kp = " + float_to_String(pid_consKp) + "\r\n");
  }
  /*
    PWM Control
    */
  pwm_control = int(pid_output);
  if(pwm_control> 25){ //NEEDS TO BE EXTENDED TO CHECK FOR TIME
    digitalWrite(LED_RB, LOW);  //Active low?
    digitalWrite(LED_COB, LOW); //YES
  }
  else
  {
    digitalWrite(LED_RB, HIGH);  //TURN OFF?
    digitalWrite(LED_COB, HIGH); 
  }
  pwm_set(LEDRB_PWM, pwm_control);
  pwm_set(LEDCOB_PWM, pwm_control);
  float pwm_val = 100.0*(pwm_control/255);
  /*
    Print out text accumulated through run
    */
  Serial_Message +=("PWM_control = " + float_to_String(pwm_val),"\r\n");
  if(loopCntr%2==0){
    Serial.print(Serial_Message);
  }
  if (loopCntr >= 10000)
  {
    loopCntr = 0;
  }
  delay(5000);
}

String getFullTimeString(DateTime aTime)
{
  String currTime;
  currTime = String(aTime.year(), DEC) + '/';
  currTime += String(aTime.month(), DEC) + '/';
  currTime += String(aTime.day(), DEC);
  currTime += " (";
  currTime += daysOfTheWeek[aTime.dayOfTheWeek()];
  currTime += ") ";
  currTime += String(aTime.hour(), DEC) + ':';
  currTime += String(aTime.minute(), DEC) + ':';
  currTime += String(aTime.second(), DEC);
  return currTime;
}

String getClockString(DateTime aTime)
{
  String aClk;
  aClk = String(aTime.hour(), DEC) + ':';
  aClk += String(aTime.minute(), DEC) + ':';
  aClk += String(aTime.second(), DEC);
  return aClk;
}

void printTemperatures()
{
  // Loop through each device, print out temperature data
  for (int i = 0; i < numberOfDevices; i++)
  {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i))
    {

      // Output the device ID
      Serial.print("Temperature for device: ");
      Serial.println(i, DEC);

      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print("Temp C: ");
      Serial.print(tempC);
      Serial.print(" Temp F: ");
      Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
    }
  }
}

void printDallasDevices()
{
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++)
  {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    }
    else
    {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
}
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void pwm_set(const int myPin, int value)
{
  if (value > 255)
  {
    value = 255;
  }
  if (value < 24 || value < 0)
  { //PWM DC is not allow to be less than 10% and a neg value makes no sense.
    value = 0;
  }
  analogWrite(myPin, value);
}
String float_to_String(float val)
{
  int buffersize = 10;
  char float2Stringbuffer[buffersize];
  String valueString = "";
  dtostrf(val, 4, 6, float2Stringbuffer); //4 is mininum width, 6 is precision
  valueString += float2Stringbuffer;
  return valueString;
}
String integer_to_String(int val)
{
  int buffersize = 10;
  char integer2Stringbuffer[buffersize];
  String valueString = "";
  itoa(val, integer2Stringbuffer, 10); //Radix is 10
  valueString += integer2Stringbuffer;
  return valueString;
}
