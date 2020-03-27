#include <Arduino.h>
#include "avdweb_SAMDtimer.h"
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <RTCZero.h>


#include "SPI.h"

#define SPI_CMD_READ   0x4000  // read command
#define SPI_REG_AGC    0x3ffd  /* agc register
                                  Diagnostics flags */
#define SPI_REG_MAG    0x3ffe  /* magnitude Register
                                  Magnitude information after ATAN calculation */
#define SPI_REG_DATA   0x3fff  /* data register
                                  Angle information after ATAN calculation
                                  and zero position adder */
#define SPI_REG_CLRERR 0x0001  /* clear error register
                                  All errors are cleared by access */

#define NEO_LED_PIN    14
#define NUMPIXELS 16 // Popular NeoPixel ring size
#define DELAYVAL 120 // Time (in milliseconds) to pause between pixels


#define status_awaiting 1
#define status_logging 2
#define status_error 3
#define status_msgRecognized 4
#define status_booting 5

#define intensityMax 60
#define intensityMin  5



word result;
unsigned int HighByte = 0;
unsigned int LowByte = 0;
long previousMicros = 0;
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean isLogging = false;
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_LED_PIN, NEO_GRB + NEO_KHZ800);
int deviceStatus = status_booting;
byte intensity = 150;
boolean breathingIn = true;
int breathingRate = 2;
File dataFile;


Sd2Card card;
SdVolume volume;
SdFile root;

//<RTC
RTCZero rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;

/* Change these values to set the current initial date */
const byte day = 1;
const byte month = 1;
const byte year = 70;
//RTC>

int ss_own = 6;  // CSn is connected to PIN 6
int SD_CHIP_SELECT = 28;

// Example timer library for the SAMD21 and Arduino Zero



void ReadSensor
(struct tc_module *const module_inst)
{

  digitalWrite(ss_own, LOW);
  HighByte = SPI.transfer(0x00);
  LowByte = SPI.transfer(0x00);
  digitalWrite(ss_own, HIGH);    // release SPI device.

  // masking
  HighByte &= 0b00111111;

  long angle = HighByte * 256 + LowByte;

  // merging
  result = word(HighByte, LowByte);

  String lineOut = "1;" + String(micros(), DEC) + ";" + String(result, DEC) + ";" + String(result, DEC);
  previousMicros = micros();
  if (isLogging) {
    dataFile.println(lineOut);
  }

}



void IndicateStatus(struct tc_module *const module_inst) {


  if (deviceStatus == status_awaiting) {
    breathingRate = 1;
    if (breathingIn) {
      intensity = intensity + breathingRate;
      if (intensity >= intensityMax) {
        breathingIn = false;
      }
    }
    else {
      intensity = intensity - breathingRate;
      if (intensity <= intensityMin) {
        breathingIn = true;
      }
    }
    for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      pixels.setPixelColor(i, pixels.Color(0, intensity, 0));
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  if (deviceStatus == status_error) {
    breathingRate = 4;
    if (breathingIn) {
      intensity = intensity + breathingRate;
      if (intensity >= intensityMax) {
        breathingIn = false;
      }
    }
    else {
      intensity = intensity - breathingRate;
      if (intensity <= intensityMin) {
        breathingIn = true;
      }
    }
    for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      pixels.setPixelColor(i, pixels.Color(intensity, 0 , 0));
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  if (deviceStatus == status_logging) {
    breathingRate = 6;
    if (breathingIn) {
      intensity = intensity + breathingRate;
      if (intensity >= intensityMax) {
        breathingIn = false;
      }
    }
    else {
      intensity = intensity - breathingRate;
      if (intensity <= intensityMin) {
        breathingIn = true;
      }
    }
    for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      pixels.setPixelColor(i, pixels.Color(intensity, intensity , intensity));
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
  }


}

// ------------- Timer3 with output, select 1 of 3 -------------
//SAMDtimer timer3_1Hz = SAMDtimer(3, TC_COUNTER_SIZE_16BIT, LED4, 1e6, 9e5); // LED4 1Hz, pulse width 0.9s (this is the constructor)



SAMDtimer mytimer1 = SAMDtimer(3, ReadSensor, 4080);
SAMDtimer mytimer3 = SAMDtimer(5, IndicateStatus, 50000);


//******************************************************************************************
void setup() // test out the several functions:
{
  InteruptsEnable(0);
  Serial.begin(9600);
  pinMode(ss_own, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode (SPI_MODE1) ;


  if (!card.init(SPI_HALF_SPEED, SD_CHIP_SELECT)) {
    Serial.println("initialization failed. Is a card inserted?");
    while (1);
  } else {
    Serial.println("Card is present.");
  }


  bool SDAvailable = SD.begin(SD_CHIP_SELECT);
  if (!SDAvailable) {
    deviceStatus = status_error;
    Serial.println("SD card error.");
  } else {
    Serial.println("SD card available.");
  }

  rtc.begin(); // initialize RTC

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  pixels.begin();
  DoStartAnimation();
  Serial.println("Setup completed.");
  deviceStatus = status_awaiting;
  InteruptsEnable(1);
  

}

void loop()
{
  recvWithEndMarker();
  ProcessNewData();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void ProcessNewData() {
  if (newData == true) {

    String receivedString = String(receivedChars);

    if (receivedString.equals("start")) {
      isLogging = true;
      createFile();
      Serial.print("Logging started\n");
      deviceStatus = status_logging;
    }

    else if (receivedString.equals("stop")) {
      if (isLogging) {
        isLogging = false;
        dataFile.close();
        Serial.print("Logging stopped\n");
        deviceStatus = status_awaiting;
      }
      else {
        Serial.print("Logging was not started\n");
      }
    }
    else if (receivedString.equals("cardinfo")) {
      reportCardInfo();
    }
    else if (receivedString.equals("volume")) {
      reportVolume();
    }
    else if (receivedString.equals("ls")) {
      listFiles();
    }
    else if (receivedString.substring(0, 5).equals("fcopy")) {
      FCopy(receivedString.substring(5));
    }
    else if (receivedString.substring(0, 4).equals("date")) {
      processDateString(receivedString.substring(4));
    }
    else {

      Serial.print("Not recognized: '" + receivedString + "'\n");
    }
    newData = false;
  }
}

void processDateString(String dateString) {

  rtc.setYear(dateString.substring(2, 4).toInt());
  rtc.setMonth(dateString.substring(4, 6).toInt());
  rtc.setDay(dateString.substring(6, 8).toInt());

  rtc.setHours(dateString.substring(9, 11).toInt());
  rtc.setMinutes(dateString.substring(11, 13).toInt());
  rtc.setSeconds(dateString.substring(13, 15).toInt());

  Serial.print("Date on device: "  + getDateTimeString() + "\n");
}

String getDateTimeString()
{
  String DateTimeString = get2digits(rtc.getYear());
  DateTimeString = DateTimeString + "/";
  DateTimeString = DateTimeString + get2digits(rtc.getMonth());
  DateTimeString = DateTimeString + "/";
  DateTimeString = DateTimeString + get2digits(rtc.getDay());
  DateTimeString = DateTimeString + " ";
  DateTimeString = DateTimeString + get2digits(rtc.getHours());
  DateTimeString = DateTimeString + ":";
  DateTimeString = DateTimeString + get2digits(rtc.getMinutes());
  DateTimeString = DateTimeString + ":";
  DateTimeString = DateTimeString + get2digits(rtc.getSeconds());
  return DateTimeString;
}

String getShortDateTimeString()
{
  String DateTimeString  =  get2digits(rtc.getMonth());
  DateTimeString = DateTimeString + get2digits(rtc.getDay());
  DateTimeString = DateTimeString + get2digits(rtc.getHours());
  DateTimeString = DateTimeString + get2digits(rtc.getMinutes());
  return DateTimeString;
}

String get2digits(int number) {

  String twoDigitsNumber = String(number);
  if (number < 10) {
    twoDigitsNumber = "0" + twoDigitsNumber;
  }
  return twoDigitsNumber;
}

void DoStartAnimation() {
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, intensity, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
  }
}

void reportCardInfo() {
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
}

void reportVolume() {
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  root.close();
}

void listFiles() {
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  root.close();
}

void createFile() {
  String fname = getShortDateTimeString() + ".txt";
  dataFile = SD.open(fname, FILE_WRITE);

  if (dataFile) {
    Serial.println(fname + " generated succesfully\n");
  }
  else {
    Serial.println("Error while creating " + fname + "\n");
  }
}

void InteruptsEnable(short enable) {
  mytimer1.enableInterrupt(enable);
  mytimer3.enableInterrupt(enable);
}


void FCopy(String fname) {
  InteruptsEnable(0);  
  File dataFileToCopy = SD.open(fname);
  Serial.print("File to copy: " + fname + "\n");

  // if the file is available, write to it:
  if (dataFileToCopy) {
    Serial.print("Copying...\n");
    while (dataFileToCopy.available()) {
      Serial.write(dataFileToCopy.read());
    }
    dataFileToCopy.close();
    Serial.print("Copy complete\n");
  }
  else
  {
    Serial.print("Copying failed.\n");
  }
  InteruptsEnable(1);
}
