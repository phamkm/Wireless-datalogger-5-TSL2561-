# Wireless-datalogger-5-TSL2561-
/* 
 *  Code for 5 TSL2561 lux sensors and 1 x TCS34725 colour sensor (if adding additional sensors) 
*/

#define DEBUG 1 // if 1 print to serial port      if(DEBUG)


#include <Wire.h>
#include <Adafruit_Sensor.h>

#define REDLED 13         //sensor box switch on/off *to add later 


#include <Adafruit_TSL2561_U.h>
Adafruit_TSL2561_Unified luxSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#define NUMSENSORS 5
int sensorPins[] = {5, 6, 9, 10, 11};
bool firstRun = true;

#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
#define NUM_RGBSENSORS 2
int rgb_sensorPins[] = {A3, A4};

#include <TimeLib.h>
#include <DS1307RTC.h>

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

const char *monthName[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
bool parse = false;
bool config = false;

tmElements_t tm;



#include <SD.h>
#include <SPI.h>
#define SD_SELECT 4
#define SD_DETECT 7
#define GREENLED 8    //turn on green LED when SD is recording 
#define VBATPIN A7 //Lipo battery pin 
boolean SERIAL_TEXT = true;
boolean SD_REMOVED = false;
float vbat = 0; 

int sensorVals[5];
struct rgbSensorType {
  int r;
  int g;
  int b;
  int c;
  int colourTemp;
  int lux;
};

rgbSensorType rgbSensor[2];



//----------------------------------------setup-----------------------------------------//


void digitalClockDisplay() {
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

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void setup() {


  pinMode(GREENLED, OUTPUT); //GREENLED
  pinMode(REDLED, OUTPUT); //REDLED
  pinMode(SD_DETECT, INPUT_PULLUP);
  pinMode(A3, OUTPUT); // SENSOR PINS TCS 
  pinMode(A4, OUTPUT); // SENSOR PINS TCS 

  digitalWrite(REDLED, HIGH);  // WHEN SLIDE SWITCH SET TO ON FLASH REDLED 

  Serial.begin(9600);
/*  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Hello!"); */
  digitalWrite(GREENLED, HIGH);

  //lux sensor init --------
  for (int i = 0; i < NUMSENSORS; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], LOW);
  }

  Serial.println("1_Light Sensor Initialised");
  Wire.begin();
  Wire.beginTransmission(TSL2561_ADDR_FLOAT);



 if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    
  }

  //rtc init--------
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  }  else {
    Serial.println("RTC has set the system time");
  }

  //SD init-------------------------
  if (digitalRead (7)) {
    Serial.println ("SD Initialising");
    if (!SD.begin(SD_SELECT)) {
      Serial.println ("SD Not Present");
    }
  }

  File dataFile = SD.open ("datalog.csv", FILE_WRITE);
  if (dataFile) {
    digitalWrite (GREENLED, HIGH);
    /*
        // Record reads as:   <  Date,time,lux0,lux1,lux2,lux3,lux4,*,rgb0.r,rgb0.g,rgb0.b,rgb0.c,rgb0.colourtemp,rgb0.lux,rgb1.r,rgb1.g,rgb1.b,rgb1.c,rgb1.colourtemp,rgb1.lux,* >
    */
    dataFile.print("Date,time,bat,lux_z+,lux_y-,lux_y+,lux_x+,lux_x-,*,rgb0.r,rgb0.g,rgb0.b,rgb0.c,rgb0.colourtemp,rgb0.lux,*");
    Serial.println ("big string written");
    dataFile.println("");
    Serial.println ("close file");
    dataFile.close();
    digitalWrite (GREENLED, LOW);
  } else {
    Serial.println ("SD DID NOT LOG");
  }

  Serial.println("");
  Serial.println("begin rtc to sd test");

  digitalWrite(GREENLED, LOW);
  

}

//----------------------------------------loop-----------------------------------------//
void loop() {


  if (timeStatus() == timeSet) {
    digitalClockDisplay();
  } else {
    Serial.println("The time has not been set.  Please run the Time");
    Serial.println("TimeRTCSet example, or DS1307RTC SetTime example.");
    Serial.println();
    delay(4000);
  }

  digitalWrite(REDLED, HIGH);


  getBatV(); 
  readLuxSensors();
  readColourData();
  writeSensorsToSD();

  delay (60000);  // overall loop delay

}// void loop


void getBatV() {
  vbat = analogRead(VBATPIN);
  vbat *= 2; // we divided by 2, so multiply back
  vbat *= 3.3; // Multiply by 3.3V, our reference voltage
  vbat /= 1024; // convert to voltage
} // getBatV

void readColourData() {
  
    uint16_t r, g, b, c, colorTemp, lux;

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.println(" ");

    rgbSensor[0].r = r;
    rgbSensor[0].g = g;
    rgbSensor[0].b = b;
    rgbSensor[0].c = c;
    rgbSensor[0].colourTemp = colorTemp;
    rgbSensor[0].lux = lux;

    delay(2000); 
  
}


void readLuxSensors() {
  // Lets read the lux sensors

  for (int i = 0; i < NUMSENSORS; i++) {
    Serial.println(" ");
    Serial.println("Lux Read Loop");
    digitalWrite(sensorPins[i], HIGH);
    Serial.print("Sensor #:"); Serial.println(i);
    delay(2000); // let it turn on

    Wire.write(TSL2561_REGISTER_ID);
    Wire.endTransmission();
    Wire.requestFrom(TSL2561_ADDR_FLOAT, 1);

    Serial.println("Read from Wire..");
    int x = Wire.read();
    Serial.print("0x"); Serial.println(x, HEX);
    if (x & 0x0A) {
      Serial.println("Found TSL2561 using if loopy bit");
    }
    Serial.println("autorange...");
    luxSensor.enableAutoRange(true);
    Serial.println("ok, integration...");
    luxSensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
    Serial.println("ok, declare event...");
    sensors_event_t event;
    Serial.println("ok, get event...");
    luxSensor.getEvent(&event);

    if (event.light) {
      sensorVals[i] = event.light;
      Serial.print(event.light); Serial.print(" lux"); Serial.print(" "); Serial.print("from: "); Serial.print(" "); Serial.println(i);
    }

    digitalWrite(sensorPins[i], LOW);
  }   //  Lux sensor if loop
}




void writeSensorsToSD() {
  // Record reads as:   <  Date,time,lux0,lux1,lux2,lux3,lux4,rgb0.r,rgb0.g,rgb0.b,rgb0.c,rgb0.colourtemp,rgb0.lux,rgb1.r,rgb1.g,rgb1.b,rgb1.c,rgb1.colourtemp,rgb1.lux,* >
  if (digitalRead(SD_DETECT)) {
    File dataFile = SD.open ("datalog.csv", FILE_WRITE);
    if (dataFile) {
      digitalWrite(GREENLED, HIGH);

      dataFile.print(day()); dataFile.print('/'); dataFile.print(month()); dataFile.print('/'); dataFile.print(year());
      dataFile.print(',');
      dataFile.print(hour()); dataFile.print(':'); dataFile.print(minute()); dataFile.print(':'); dataFile.print(second());
      dataFile.print(',');
      dataFile.print(vbat); dataFile.print(','); 

      for (int i = 0; i < 5; i++) { // lux values
        //dataFile.print(i);
        //dataFile.print(',');

        dataFile.print(sensorVals[i]);
        dataFile.print(',');
      }

      dataFile.print('*');
      dataFile.print(',');

      for (int i = 0; i < 1; i++) { // rgb sesnor vals
        dataFile.print(rgbSensor[i].r);
        dataFile.print(',');
        dataFile.print(rgbSensor[i].g);
        dataFile.print(',');
        dataFile.print(rgbSensor[i].b);
        dataFile.print(',');
        dataFile.print(rgbSensor[i].c);
        dataFile.print(',');
        dataFile.print(rgbSensor[i].colourTemp);
        dataFile.print(',');
        dataFile.print(rgbSensor[i].lux);
        dataFile.print(',');
      }

      dataFile.println("*");
      dataFile.close();
      digitalWrite(GREENLED, LOW);
      digitalClockDisplay();
      if (SERIAL_TEXT) {
      } // SERIALTEXT
    }
  } else { // SD_detect if
    Serial.println("Oh my, no SD!!!!!");
  }

}
