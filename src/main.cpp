#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>       // MCP9808 - Temperature Senor
#include <MPU9250_asukiaaa.h>       // MPU9250 - Inertial Measurement Unit
#include "RTClib.h"                 // Real Time Clock
#include <Adafruit_DRV2605.h>       // DRV2605 - Motor Driver
#include <CircularBuffer.h>         // Circular Buffer
#include <MAX30105.h>               // Heart rate monitor
#include "heartRate.h"              // Used to obtain beatsPerMinute
#include <BLEPeripheral.h>
#include "BLESerial.h"
#include <stdio.h>
#include <stdint.h>

//#define USE_LFRC
//#define PRINT_CIRCULAR_BUFFER
//#define PRINT_STRING
//#define BLE_TRANSMISSION

#define MIN_CONN_INTERVAL (uint16_t)(MSEC_TO_UNITS(30, UNIT_1_25_MS)) #define MAX_CONN_INTERVAL (uint16_t)(MSEC_TO_UNITS(60, UNIT_1_25_MS))

#define BLE_REQ   0
#define BLE_RDY   0
#define BLE_RST   0

#define _NRF_DELAY_H
//#define CONNECTION_TESTING
//#define STATUS
//#define SERIAL_LOGGING
//#define CONNECTION_INTERVAL
#define BLE_LOGGING


// Functions Prototypes
void printTerminalData();
void createDatalog();
void createHeader();
void getTemperature();
void getAccelerometer();
void getDate();
void getTime();
void randomHaptic();
String batteryVoltage();
void waveformConfig();
void clearBuffer();


// Data Storage variables
const int chipSelect = 8;
const int bufferSize = 50;
int headerCount;
int motorCount;
int connectionEndTime;
int nextConnection;
int execution[10];
int dateArray[6];
int commandCount = 0;
int commandPointer = 0;
long irReading = 0;
long lastBeat = 0;
long delta = 0;
float bpm = 0.0;
float c;
float f;
String date;
String time;
String motorState;
//uint8_t sensorId;
float aX, aY, aZ;
String fileName = "";
String commandFile = "COMMAND.txt";
String timeFile = "TIMESHEET.txt";
String dataString;
int sampleNumber = 0;
int hourPlaceholder;
int eventSignal;
String unixTime;
boolean lowBattery = false;
boolean eventTimer = false;
boolean connectionState = true;
boolean connectionCheck = true;
boolean timingCondition;
boolean bleInit;
boolean motorOn = false;
unsigned long millisecondsHolder;
unsigned int numberOfFiles;

// Create program objects
File dataFile;
File exportFile;
File timeSheet;
RTC_Millis rtc;
Adafruit_DRV2605 drv;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
MPU9250_asukiaaa imuSensor;
MAX30105 hrm;

#ifdef BLE_LOGGING
  BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);
#endif

namespace data {
 typedef struct {
   int sampleNumber;
   String time;
   float tempC;
   float accX;
   float accY;
   float accZ;
   //float heartRate;
   String motorStatus;
   String voltage;
 } record;

  void print(record r) {
   #ifdef SERIAL_LOGGING 
    Serial.print(r.sampleNumber);
    Serial.print(",");
    Serial.print(r.time);
    Serial.print(',');
    Serial.print(r.tempC);
    Serial.print(",");
    Serial.print(r.accX);
    Serial.print(",");
    Serial.print(r.accY);
    Serial.print(",");
    Serial.print(r.accZ);
    Serial.print(",");
    // Serial.print(r.heartRate);
    // Serial.print(",");
    Serial.println(r.motorStatus);
    Serial.print(",");
    Serial.println(r.voltage);
   #endif 
 }

  void write(record r) {
   File dataFile = SD.open(fileName, FILE_WRITE);
   File timeSheet = SD.open(timeFile, FILE_WRITE);
   dataFile.print(r.sampleNumber);
   dataFile.print(",");
   dataFile.print(r.time);
   dataFile.print(",");
   dataFile.print(r.tempC);
   dataFile.print(",");
   dataFile.print(r.accX);
   dataFile.print(",");
   dataFile.print(r.accY);
   dataFile.print(",");
   dataFile.print(r.accZ);
   dataFile.print(",");
  //  dataFile.print(r.heartRate);
  //  dataFile.print(",");
   dataFile.print(r.motorStatus);
   dataFile.print(",");
   dataFile.print(r.voltage);
   dataFile.println();
   timeSheet.println(unixTime);
   dataFile.close();
   timeSheet.close();
 }
}
CircularBuffer<data::record,bufferSize> stack;



//Functions-----------------------------------------------------------------

/**
 * This method is simply used to print current data to the terminal viewer.
*/
void printTerminalData(){
  #ifdef SERIAL_LOGGING
    Serial.print("Temp: ");
    Serial.print(c);
    Serial.print("*C\t"); 
    Serial.print(f);
    Serial.println("*F");
    Serial.print("X: ");
    Serial.println(String(aX));
    Serial.print("Y: " );
    Serial.println(String(aY));
    Serial.print("Z: ");
    Serial.println(String(aZ));
    Serial.print("Milli: ");
    Serial.println(String(millis()));
    Serial.print("Date: ");
    Serial.println(date);
    Serial.print("Time: ");
    Serial.println(time);
  #endif
}

/**
 * This method creates a single header for the Datalog file that is present on the microSD card.
*/
void createHeader()
{
  File dataFile = SD.open(fileName, FILE_WRITE);
  //Print the header for each column of the file
  while(headerCount < 1){
    dataFile.print("Sample Number");
    dataFile.print(",");
    dataFile.print("Unixtime");
    dataFile.print(",");
    dataFile.print("Temperature (C)");
    dataFile.print(",");
    dataFile.print("Accel X");
    dataFile.print(",");
    dataFile.print("Accel Y");
    dataFile.print(",");
    dataFile.print("Accel Z");
    dataFile.print(",");
    // dataFile.print("Heart Rate (BPM)");
    // dataFile.print(",");
    dataFile.print("Motor");
    dataFile.print(",");
    dataFile.print("Voltage (V)");
    dataFile.println();
    dataFile.close();
    headerCount++;
  }

}



/**
 * This method takes current temperature readings in both Celcius and Farhenheit.
*/
void getTemperature()
{
  //Temperature Readings
  c = tempsensor.readTempC();
}

/**
 * This method takes Accelerometer readings and saves them as float values.
*/
void getAccelerometer(){
  //IMU Readings
  imuSensor.accelUpdate();
  aX = imuSensor.accelX();
  aY = imuSensor.accelY();
  aZ = imuSensor.accelZ();
}

/**
 * This method is and example of how to implement the haptic motor as a notification device.
*/
void randomHaptic()
{
  if (!motorOn)
  {
    eventSignal = (execution[commandPointer] * 60000) + millis();
    motorOn = true;
  }

  //motorCount = random(100);
  if (motorOn && (millis() >= eventSignal))
  {
    motorState = "ON";
    drv.go();
    BLESerial.println("MADE ITTTTTTTTTTTTTTTTTTTTTTTTTT");
    motorCount = 0;
    if (commandPointer >= 10)
    {
      commandPointer = 0;
      motorOn = false;
    }
    else 
    {
      commandPointer++;
      motorOn = false;
    }
  }
  else{
    motorState = "";
  }
}

/**
 * This method is designed to warn the user if the battery charge is too low.
*/
String batteryVoltage()
{
  //Battery Charge Monitoring
  String voltageHolder;
  float reading = analogRead(PIN_A0);
  float voltage = reading * (3.7 / 1023) - 0.4;
  if (voltage < 1.65)
  {
    voltageHolder = String(voltage) + " - LOW_BATT: ENTERING SLEEP MODE";
    lowBattery = true;
  }
  else if(lowBattery && voltage >= 1.65)
  {
    voltageHolder = String(voltage) + " - EXITING SLEEP MODE";
    lowBattery = false;
  }
  else
  {
    voltageHolder = String(voltage);
    lowBattery = false;
  }
  return voltageHolder;
}

/**
 * This method sets the vibration mode for the onboard haptic depending on the task.
 * Param waveformValue - Sets supported DRV2605 hardcoded waveforms
 */
void waveformConfig (int waveformValue)
{
  //Motor Driver Commands -- I2C trigger by sending 'go' command 
  // drv.begin();
  // drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
  // drv.selectLibrary(1);
  drv.setWaveform(0, waveformValue);  // ramp up medium 1, see datasheet part 11.2
  drv.setWaveform(1, waveformValue);
  drv.setWaveform(2, 0);  // end of waveforms
}
/**
 * This method generates a new file on the SD card dependant upon the reset state of the microcontroller.
 */
void SD_INIT()
{
  #ifdef SERIAL_LOGGING
    Serial.println("SD_INIT BEGIN");
  #endif
  while (!SD.begin(chipSelect)) {
    #ifdef SERIAL_LOGGING
      Serial.println("SD_INIT FAILED!");
    #endif
  }
  
  String message = String();
  unsigned int filenumber = 1;
  while(!filenumber==0) {
    fileName = "file_";
    fileName += filenumber;
    fileName += ".csv";
    message = fileName;
    char charFileName[fileName.length() + 1];
    fileName.toCharArray(charFileName, sizeof(charFileName));
    if (SD.exists(charFileName)) { 
      message += " exists.";
      filenumber++;
    }
    else {
      File dataFile = SD.open(charFileName, FILE_WRITE);
      message += " created.";
      dataFile.close();
      filenumber = 0;
    }
  }
    char exeFileName[commandFile.length() + 1];
    commandFile.toCharArray(exeFileName, sizeof(exeFileName));
    if (SD.exists(exeFileName)) { 
      message += " exists.";
    }
    else {
      File comFile = SD.open(exeFileName, FILE_WRITE);
      message += " created.";
      comFile.print("10,20,30,40,60,120,15,75,90,65");
      comFile.close();
    }
  numberOfFiles = filenumber;       // This will be used for iterating through the files present on the SD card. Ex. If the numberOfFiles was equal to 9 then the program should itterate through FILE_1 through FILE_9
  delay(1000);
}

/**
 * This method uses the RTC Library to get the current date and save it as a String.
 */
void getDate()
{
  DateTime now = rtc.now();

  String year = String(now.year(), DEC);    //go to definition of year() to revert back to normal
  String month = String(now.month(), DEC);
  String day = String(now.day(), DEC);
  date = year + '/' + month + '/' + day;
}

void checkTime()
{
  DateTime now = rtc.now();
  if (now.hour() != hourPlaceholder)
  {
    headerCount = 0;
    SD_INIT();
    createHeader();
    hourPlaceholder = now.hour();
  }
}


/**
 * Retrieves basic unixtime for timing reference.
 */
void getUnixTime()
{
  DateTime now = rtc.now();
  // unixTime = String(now.unixtime());

  int milliseconds = (millis() % 1000);
  String milliString = "";
  // Formats the milliseconds to add three aditional decimal places to the time.
  if (milliseconds >= 100)
  {
    milliString += String(milliseconds);
  }
  else if (milliseconds < 100 && milliseconds >= 10)
  {
    milliString += "0" + String(milliseconds);
  }
  else if (milliseconds < 10)
  {
    milliString += "00" + String(milliseconds);
  }
  unixTime = String(now.unixtime()) + '.' + milliString;        // Used for more accurate datalogging
  time = String(now.unixtime());                                // Used for settime comparator
}

/**
 *  Used to adjust the time over BLE. 
 */
void adjustTime(int year, int month, int day, int hour, int minute, int second)
{
  // Example:
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  rtc.begin(DateTime(year, month, day, hour, minute, second));
}

/**
 * This method clears the memory buffer of the device to prevent potential overflows.
 */
void clearBuffer()
{
  dataFile.println();
  dataFile.flush();     //Clears the buffer so that it does not overflow
  dataFile.close();     //Closes the file to make sure there are no unintended writes
}

void microDelay(int us)
{
  int c = 64 * us;
  for (int i = 0; i < c; i++)
  {
    timingCondition = false;
  }
}

int split(String input, int numDelimeters, char delimeter)
{
  char timeArray[input.length() + 1];
  input.toCharArray(timeArray, sizeof(timeArray));

  int delimeterCount = 0;
  int result = 0;
  String holder = "";
  int i = 0;
  while (delimeterCount != numDelimeters)
    {
      if (timeArray[i] != delimeter)
      {
        i++;
      }
      else
      {
        delimeterCount++;
      }
    }

  for (i; i < input.length(); i++)
  {
    if (delimeterCount == numDelimeters && timeArray[i] != delimeter)
    {
      holder += timeArray[i];
    }
    else if (delimeterCount == numDelimeters + 1)
    {
      break;
    }
    delimeterCount++; 
  }

  result = atoi(holder.c_str());

  return result;
}

/**
 * Allows dor reading of the alogrithm file of the device.
 */
void assignAlgorithm()
{
  File comFile = SD.open(commandFile, FILE_READ);
  String holder = comFile.readStringUntil('\n');
  BLESerial.print("FILESTRING: ");
  BLESerial.println(holder);

  char commandArray[holder.length() + 1];
  holder.toCharArray(commandArray, sizeof(commandArray));

  char * strtokIndex;
  strtokIndex = strtok(commandArray, ",");
  execution[0] = atoi(strtokIndex);
  BLESerial.println(execution[0]);

  for (int i = 1; i < 10; i++)
  {
    strtokIndex = strtok(NULL, ",");
    execution[i] = atoi(strtokIndex);
    BLESerial.println(execution[i]);
  }

  // int number;
  // char delimeter;
  // for(int i = 0; i < 10; i++)
  // {
  //   number = split(holder, i, delimeter);
  //   execution[i] = number;
  //   BLESerial.println(execution[i]);
  // }

  #ifdef SERIAL_LOGGING
    for (int i = 0; i < 10; i++)
    {
      Serial.println("COMMAND " + i + ": ");
      Serial.println(execution[i]);
    }
  #endif
}
/**
 * Gives us the ability to read from the SD card and transmit files via BLE. 
 */
void readFile(String fileIn)
{
  String holderString = "";
  int packetCounter = 0;
  int byteCounter = 0;
  File exportFile = SD.open(fileIn);
  if (exportFile && BLESerial.connected())
  {
    #ifdef SERIAL_LOGGING
      Serial.print("File contains: ");
      Serial.print(exportFile.available());
      Serial.println(" bytes");
    #endif
    BLESerial.print("File contains: ");
    BLESerial.print(exportFile.available());
    BLESerial.println(" bytes");
    while (exportFile.available() && connectionState && BLESerial.connected())
    {
      // Splits files from the SD card into 20 byte chunks to be sent over BLE.
      while (byteCounter < 20)
      {
        char holder = exportFile.read();
        holderString += String(holder);
        byteCounter++;
        #ifdef SERIAL_LOGGING
          //Serial.println("CHAR: " + String(holder));
        #endif
        if (exportFile.peek() == EOF)
        {
          #ifdef SERIAL_LOGGING
            Serial.println("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
          #endif
          // holderString += " ";
          // byteCounter++;
          break;
        }
      }

      packetCounter++;
      #ifdef SERIAL_LOGGING
        Serial.print("C: ");
        Serial.println(packetCounter);
        Serial.print(" ");
        Serial.println(holderString);
        Serial.println("HOLDER STRING: " + holderString + " XXX");
      #endif
      
      #ifdef BLE_LOGGING
        BLESerial.print(holderString);    //Only writing one character at a time.
      #endif
      microDelay(1500);                   //The most efficient delay for minimal packet loss
      //delay(10);                        //Slightly less effiecient delay method
      holderString = "";                  //Clear holder string
      byteCounter = 0;                    //Clear byte counter
    }
    #ifdef SERIAL_LOGGING
      Serial.println();
    #endif
    exportFile.close();
    packetCounter = 0;
  }
}

/**
 * Iterates through all the files on the SD card to transmit all data.
 */
void sdCardRead(int fileNum)
{
  boolean filesExist = true;
  int numHolder = 1;
  if (fileNum > 0 && fileNum != 1)
  {
    numHolder = fileNum;
    #ifdef SERIAL_LOGGING
      Serial.println("FILE NUMBER: " + String(fileNum));
    #endif  
  }
  while(filesExist)
  {
    String fileHolder = "file_";
    fileHolder += numHolder;
    fileHolder += ".csv";
    char charFileName[fileHolder.length() + 1];
    fileName.toCharArray(charFileName, sizeof(charFileName));
    if (SD.exists(fileHolder))
    {
      #ifdef SERIAL_LOGGING
        Serial.println(fileHolder);
      #endif
      delay(50);
      if (connectionState && BLESerial.connected())
      {
        BLESerial.println(fileHolder);
      }
      readFile(fileHolder);
      numHolder++;
    }
    else
    {
      filesExist = false;
      #ifdef SERIAL_LOGGING
        Serial.println("File Transfer Complete");
      #endif
      
      #ifdef BLE_LOGGING
        if (connectionState && BLESerial.connected())
        {
          BLESerial.println( "," + String(numHolder) + ",!!!");
          #ifdef SERIAL_LOGGING 
            Serial.println("," + String(numHolder) + ",!!!");
          #endif
          SD_INIT();
          createHeader();
        }
      #endif
    }
  }
}

// int split(String input, int numDelimeters)
// {
//   char timeArray[input.length() + 1];
//   input.toCharArray(timeArray, sizeof(timeArray));

//   int delimeterCount = 0;
//   int result = 0;
//   String holder = "";
//   for (int i = 0; i < input.length(); i++)
//   {
//     if (delimeterCount == numDelimeters - 1)
//     {
//       holder += timeArray[i];
//     }
//     else if (delimeterCount == numDelimeters)
//     {
//       break;
//     }
//     delimeterCount++;
//   }

//   result = atoi(holder.c_str());

//   return result;
// }

/**
 * Allows the Base station to issue commands to the activity monitor to perform different tasks. 
 */
void bleRead()
{
  if (connectionState && BLESerial.connected())
  {
    String holder = BLESerial.readStringUntil(',');
    
    if (holder.equals("comm"))
    {
      String fileNumString = BLESerial.readStringUntil(',');
      
      int fileNum = atoi(fileNumString.c_str());


      #ifdef SERIAL_LOGGING
        Serial.println("MESSAGE RECEIVED");
      #endif
      BLESerial.println("start");
      sdCardRead(fileNum);
    }

    else if (holder.equals("def"))
    {
      SD.remove(commandFile);
      File comFile = SD.open(commandFile, FILE_WRITE);
      comFile.println("10,20,30,40,60,120,15,75,90,65");
      comFile.close();
      assignAlgorithm();
      commandCount = 10;
      commandPointer = 0;
    }

    else if (holder.equals("test"))
    {
      String valueString = BLESerial.readString();
      int value = valueString.toInt();
      BLESerial.print("RECEIVED VALUE: ");
      BLESerial.println(value);
      BLESerial.print("EXECUTION VALUE: ");
      if (value < 0 || value > 10)
      {
        BLESerial.println("NOT VALID");
      }
      else
      {
        BLESerial.println(execution[value]);
      }
    }

    else if (holder.equals("test2"))
    {
      assignAlgorithm();
    }

    else if (holder.equals("cont"))
    {
      readFile(commandFile);
    }

    else if (holder.equals("settime"))
    {
      String newTime = BLESerial.readString();


      String yearString = BLESerial.readStringUntil(',');
      String monthString = BLESerial.readStringUntil(',');
      String dayString = BLESerial.readStringUntil(',');
      String hourString = BLESerial.readStringUntil(',');
      String minuteString = BLESerial.readStringUntil(',');
      String secondString = BLESerial.readStringUntil(',');

      // Methodology for reading in a single string from the base station and parsing it into a usable setstring
      // int yearNum = split(newTime, 1);
      // int monthNum = split(newTime, 2);
      // int dayNum = split(newTime, 3);
      // int hourNum = split(newTime, 4);
      // int minuteNum = split(newTime, 5);
      // int secondNum = split(newTime, 6);

      int yearNum = atoi(yearString.c_str());
      int monthNum = atoi(monthString.c_str());
      int dayNum = atoi(dayString.c_str());
      int hourNum = atoi(hourString.c_str());
      int minuteNum = atoi(minuteString.c_str());
      int secondNum = atoi(secondString.c_str());
      #ifdef SERIAL_LOGGING
        Serial.print("YEAR: ");
        Serial.println(yearNum);
        Serial.print("MON: ");
        Serial.println(monthNum);
        Serial.print("DAY: ");
        Serial.println(dayNum);
        Serial.print("HOUR: ");
        Serial.println(hourNum);
        Serial.print("MIN: ");
        Serial.println(minuteNum);
        Serial.print("SEC: ");
        Serial.println(secondNum);
      #endif

      adjustTime(yearNum,monthNum,dayNum,hourNum,minuteNum,secondNum);
      waveformConfig(24);
      drv.go();
      waveformConfig(47);
    }

    else if (holder.equals("gettime"))
    {
      getUnixTime();
      BLESerial.println(time);
    }

    else if (holder.equals("add"))
    {
      String comHolder = "";
      String stage1 = BLESerial.readStringUntil(',');

      comHolder += stage1;
      comHolder += ",";

      delay(20);
      if (commandCount < 10)
      {
        File comFile = SD.open(commandFile, FILE_WRITE);
        comFile.print(comHolder);
        BLESerial.println("REACHEDDDDDDDDDDDDDDDDD");
        commandCount++;
        if (commandCount = 10)
        {
          comFile.println();
          assignAlgorithm();
          commandPointer = 0;
        }
        comFile.close();
      }
      else
      {
        {
          BLESerial.println("COMMAND LIMIT REACHED");
        }
      }
      
    }

    else if (holder.equals("ec"))
    {
      SD.remove(commandFile);
      commandCount = 0;
    }

    else if (holder.equals("limu"))
    {
      do
      {
        getAccelerometer();
        getUnixTime();
        BLESerial.print("TIME: ");
        BLESerial.print(unixTime);
        BLESerial.print(" X: ");
        BLESerial.print(aX);
        BLESerial.print(" Y: ");
        BLESerial.print(aY);
        BLESerial.print(" Z: ");
        BLESerial.println(aZ);
      } while (BLESerial.readString() != "stop");
    }


    else if (holder.equals("motor"))
    {
      do
      {
        //BLESerial.println("ON");
        drv.go();
      } while (BLESerial.readString() != "stop");
      
    }

    else if (holder.equals("motor2"))
    {
      waveformConfig(24);
      do
      {
        drv.go();
      } while (BLESerial.readString() != "stop");
      waveformConfig(47);
    }

    else if (holder.equals("temp"))
    {
      getTemperature();
      BLESerial.print("Temp C: ");
      BLESerial.println(c);
    }
  }
}

/**
 * Sets the connection time variables so that the device knows when to properly disconnect from the base station.
 */
void setConnectionTimes()
{
  connectionCheck = false;
  connectionEndTime = millis() + 180000;
  nextConnection = millis() + 360000;
  //connectionEndTime = millis() + 300000;
  //nextConnection = millis() + 1500000;
}

void beatsPerMinute()
{
  irReading = hrm.getIR();
  if (checkForBeat(irReading) == true)
  {
    delta = millis() - lastBeat;
    lastBeat = millis();

    bpm = 60 / (delta / 1000.0);
  }
}

//---------------------------------------------------------------------------
void setup()
{
  #ifdef SERIAL_LOGGING
    Serial.begin(115200);
  #endif

  #ifdef CONNECTION_TESTING
    Serial.begin(115200);
  #endif
    
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = rtc.now();
  hourPlaceholder = now.hour();

  #ifdef BLE_LOGGING
    BLESerial.setLocalName("ACTIVITY_MONITOR_NEW");
    BLESerial.begin();
    bleInit = true;
  #endif
  
  pinMode(SS, OUTPUT);  //Chip select set to output
  SD_INIT();
  assignAlgorithm();
  createHeader();

  //Initializes the microcontroller with the time of compilation
  Wire.begin();
  imuSensor.setWire(&Wire);
  imuSensor.beginAccel();
  delay(10);
  //sensorId = imuSensor.readId();
  tempsensor.begin();
  hrm.begin();
  hrm.setup();
  hrm.setPulseAmplitudeRed(0x0A);
  hrm.setPulseAmplitudeGreen(0);
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
  waveformConfig(47);
}
//--------------------------------------------------------------------------------------------------
void loop()
{ 
  #ifdef BLE_LOGGING
      BLESerial.poll();
      // Checks to see if the connection interval times need to be set. Otherwise the times will continue reseting while connected.
      #ifdef CONNECTION_INTERVAL
      if (connectionState)
      { 
        if (!bleInit)
        {
          bleInit = true;
        }
        if (connectionCheck || BLESerial.connected())
        {
          setConnectionTimes();
          #ifdef CONNECTION_TESTING
            Serial.println("SET INTERVAL TIMES");
          #endif
        }
        // If the interval time has passed then put the device in a non-connection state.
        else if (millis() >= connectionEndTime)
        {
          connectionState = false;
          ((BLEPeripheral) BLESerial).~BLEPeripheral();
          #ifdef CONNECTION_TESTING
            Serial.println("MADE IT TO BLE DISCONNECT STATE");
          #endif
        }
      }
      else if (!connectionState && millis() >= nextConnection)
      {
        connectionCheck = true;
        connectionState = true;
        #ifdef CONNECTION_TESTING
          Serial.println("MADE IT TO NEXT CONNECTION CONDITION");
        #endif
        BLESerial.begin();
        microDelay(50);
        bleInit = false;
      }
      #endif
  #endif

  #ifdef CONNECTION_TESTING
  #ifdef STATUS
    Serial.println("RUNNING");
  #endif
  #endif
  getUnixTime();
  sampleNumber++;
  String voltageReading = batteryVoltage();
  if (!lowBattery && eventTimer || !eventTimer)
  {
    millisecondsHolder = millis();
    // This while loop statment ensures that data is collected over a 5 minute span of time
    while (millisecondsHolder + 300000 > millis())
    {
      getUnixTime();
      sampleNumber++;
      getTemperature();
      getAccelerometer();
      beatsPerMinute();
      randomHaptic();

      stack.push(data::record{sampleNumber, unixTime, c, aX, aY, aZ, motorState, voltageReading});
      while(!stack.isEmpty())
      {
        data::write(stack.shift());
        delay(10);
      }
      stack.clear(); 
        
      #ifdef PRINT_STRING
        printTerminalData();
      #endif
      #ifdef BLE_LOGGING
        if (connectionState && BLESerial.connected())
        {
          bleRead();
        }
      #endif
    }
    motorState = "";
    eventTimer = false;
  }

  // If the activity monitor currently has a low batery or is not currently in a data collection state the device will enter a sleep mode to prevent unecessary power consumption.
  else if (lowBattery  || !eventTimer)
  {
    #ifdef SERIAL_LOGGING
      Serial.println("SLEEP STATE " + unixTime);
    #endif
    File dataFile = SD.open(fileName, FILE_WRITE);
    dataFile.print(sampleNumber);
    dataFile.print(",");
    dataFile.println(unixTime);
    dataFile.close();
    delay(100);
    randomHaptic();
    if (motorState.equals("ON"))
    {
      eventTimer = true;
    }
  }
  #ifdef BLE_LOGGING
  if (BLESerial.connected())
  {
    bleRead();
  }
    // if (connectionState && BLESerial.connected())
    // {
    //   if (!bleInit)
    //   {
    //     bleInit = true;
    //   }
    //   bleRead();
    // }
  #endif
  checkTime();
}
