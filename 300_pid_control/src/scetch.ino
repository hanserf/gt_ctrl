#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
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
int pwm_adjust = 0;
//PID
double pid_input, pid_output, pid_setpoint;
//Aggressive regulation when far away from setpoint.
double pid_aggKi = 4, pid_aggKp = 0.2, pid_aggKd = 1;
//Presice regulation near setpoint.
double pid_consKi = 1, pid_consKp = 0.05, pid_consKd = 0.25;
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_consKp, pid_consKi, pid_consKd, DIRECT);

extern String getFullTimeString(DateTime aTime);
extern String getClockString(DateTime aTime);
extern int isFutureevent(DateTime now);
extern void printTemperatures();
extern void printDallasDevices();
extern void printAddress(DeviceAddress deviceAddress);
extern void pwm_set(const int myPin, int value);

void setup()
{
  //GPIO
  pinMode(LED_RB, OUTPUT);
  digitalWrite(LED_RB, HIGH); //Active LOW
  pinMode(LED_COB, OUTPUT);
  digitalWrite(LED_COB, HIGH); //Active LOW
  pinMode(LEDRB_PWM, OUTPUT);
  pinMode(LEDCOB_PWM, OUTPUT);
  pwm_set(LEDRB_PWM, 0);
  pwm_set(LEDCOB_PWM, 0);
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
}

void loop()
{

  loopCntr++;
  now = rtc.now();
  String myTD = getFullTimeString(now);
  String Serial_Message;
  Serial_Message = "Run: " + String(loopCntr) + '\n';
  Serial_Message += (myTD + '\n');
  /*
      Get sensor temperatures
    */
  sensors.getAddress(tempDeviceAddress, indoor_temp_sensor);
  sensor_indoor_temp = sensors.getTempC(tempDeviceAddress);
  sensors.getAddress(tempDeviceAddress, outdoor_temp_sensor);
  sensor_outdoor_temp = sensors.getTempC(tempDeviceAddress);
  Serial_Message += ('T_indoor = ' + String(sensor_indoor_temp) + '\n');
  Serial_Message += ('T_outdoor = ' + String(sensor_outdoor_temp) + '\n');
  /*
      PID Algoritm
    */
  pid_input = double(sensor_indoor_temp);
  double gap = abs(pid_setpoint - pid_input); //distance away from setpoint
  bool pid_mode_aggressive;
  if (gap < 2.0)
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
  Serial_Message += ('T_err = ' + String(gap) + '\n');
  Serial_Message += ('PID_output = ' + String(pid_output) + '\n');
  if (pid_mode_aggressive)
  {
    Serial_Message += ('PID_mode = AGGRESSIVE' + '\n');
    Serial_Message += ('Kd = ' + String(pid_aggKd) + ' , ');
    Serial_Message += ('Ki = ' + String(pid_aggKi) + ' , ');
    Serial_Message += ('Kp = ' + String(pid_aggKp) + '\n');
  }
  else
  {
    Serial_Message += ('PID_mode = CONSERVATICE' + '\n');
    Serial_Message += ('Kd = ' + String(pid_consKd) + ' , ');
    Serial_Message += ('Ki = ' + String(pid_consKi) + ' , ');
    Serial_Message += ('Kp = ' + String(pid_consKp) + '\n');
  }

  digitalWrite(LED_RB, LOW);  //Active low?
  digitalWrite(LED_COB, LOW); //YES
  pwm_set(LEDRB_PWM, 140);
  pwm_set(LEDCOB_PWM, 140);
  Serial_Message += ("Purple lights ON, 60%, White lights ON, 60%" + '\n');

  if (loopCntr % 10)
  {
    Serial.print(Serial_Message);
  }
  loopCntr++;
  if (loopCntr >= 10000)
  {
    loopCntr = 0;
  }
  delay(1000);
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

int isFutureevent(DateTime now)
{
  int rc = 0;
  if (now >= getFutureEvent())
  {
    rc = 1;
  }
  return rc;
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
