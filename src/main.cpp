#include <SPI.h>
#include <EEPROM.h>
//#include <RHReliableDatagram.h>
#include <RHDatagram.h>
#include <RH_RF95.h>
#include <Wire.h>

#define RFM95_RST 4      //Radio Reset Pin
#define RFM95_INT 7      //Radio GPIO0/IRQ Pin
#define RFM95_CS 8       //Radio Chip Select Pin
#define RFM95_FREQ 915.0 //868 MHz or 915 MHz
#define LED 13           //Built-In LED

//Radio client/server addresses
#define CLIENT_ADDRESS 0 // The remote is the client
#define SERVER_ADDRESS 1 // This feather on the Rover is the server

#define SERVER_ADDRESS_BASE 0x1
#define SERVER_ADDRESS_MIN 1
#define SERVER_ADDRESS_MAX 0xF
#define SERVER_ADDRESS_DEFAULT SERVER_ADDRESS_MIN

//Radio Driver Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Class to manage message delivery and receipt, using the driver (rf95) declared above
//RHReliableDatagram manager(rf95, SERVER_ADDRESS);
RHDatagram manager(rf95, SERVER_ADDRESS);

// MessageOut Struct
struct dataStruct
{
  uint8_t resetConfirmed;
  uint8_t resetTime;
  uint8_t roverMode;
  uint8_t signalAverageMSB;
  uint8_t signalAverageLSB;
} messageReadings;

// messageIn Struct
struct inStruct
{
  bool resetFlag;
  uint8_t statusUpdate;
  uint8_t desiredAngle;
} messageIn;

// Buffer for passing messageReadings via radio
uint8_t bufT[sizeof(messageReadings)] = {0};
//uint8_t bufI[sizeof(messageIn)] = {0};

#define SIGNAL_SAMPLE 50
#define DAY_MILLIS 86400000
//#define DAY_MILLIS 30000
#define MINUTE_MILLIS 60000
// Interrupt variables
volatile uint16_t signalAverage = 0;

// Constants
const int signalCheckInterval = 50;  // number of millisecs before temperature readings
const int recvInterval = 0;          // number of millisecs before checking for messages
const int sendInterval = 700;        // number of millisecs before sending messages
const int temperatureInterval = 300; //number of millisecs before temperature readings
const int parameterInterval = 300;
const int rpiInterval = 300;
const int chooseInterval = 2000;
const unsigned long initialInterval = 180000;   // 3 minutes
const unsigned long sendCheckInterval = 180000; // 3 minutes

const int EepromRemoteAddress = 0;

const int heaterPin = A0;
const int cameraPin = A1;
const int killPin = A2;
const int switchPin = A3; //9;//A3
const int thermistorPin = A4;
const int antennaBoard_CS = A5; // Need to replace logic, unused now
//const int fan2Pin = 10;         //Fan PWM control
const int fan1Pin = 11;         //Fan PWM control
const int auxPin = 12;
const int signalPin = 0;
const int choicePin = 5;
const int programmingPin = 6;

// Variables
unsigned long previousSignalMillis = 0;
unsigned long previousSendMillis = 0;
unsigned long previousRecvMillis = 0;
unsigned long previousTempMillis = 0;
unsigned long previousResetMillis = 0;
unsigned long previousElectronMillis = 0;
unsigned long previousElectronPeriodMillis = 0;
unsigned long previousParameterMillis = 0;
unsigned long previousRpiMillis = 0;
unsigned long previousChooseMillis = 0;
unsigned long lastSendCommand = 0;
unsigned long currentMillis = 0;
int resetConfirmedCounter = 0;
int currentVersion = 1;
uint8_t desiredAntennaAngle = 0;
unsigned long electronResetCounter = 0;
bool resetTimeFlag = true;
byte roverAddress = 1;
bool cameraFlag = false;
bool killFlag = false;
bool switchFlag = false;
bool resetAllFlag = false;
bool cameraSignal = false;
bool killSignal = false;
bool switchSignal = false;
bool sendFlag = false;
bool initialFlag = false;
bool desiredAngleChangeFlag = false;

int thermistorReading = 0;

int piCommand = -1;

void i2cReceive(int howMany) {
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    digitalWrite(LED, c);
    Serial.println(c);
    piCommand = c;
  }
}

void signalMessageSetup() {
  messageReadings.signalAverageMSB = (signalAverage & 0xFF00) >> 8;
  messageReadings.signalAverageLSB = (signalAverage & 0x00FF);
}

//Blink LED Function
void blinkLED(int onDelayTime, int offDelayTime)
{
  digitalWrite(LED, HIGH);
  delay(onDelayTime);
  digitalWrite(LED, LOW);
  delay(offDelayTime);
}

void writeServerAddressToEeprom(uint16_t address)
{
  EEPROM.write(EepromRemoteAddress, address);
}

uint16_t readServerAddressFromEeprom()
{
  return EEPROM.read(EepromRemoteAddress);
}

void setup()
{
  /*digitalWrite(LED, 1);
  digitalWrite(6, LOW);
  digitalWrite(18, 1);
  //digitalWrite(19, 1);
  digitalWrite(20, 1);
  digitalWrite(21, 1);
  digitalWrite(22, 1);
  digitalWrite(23, 1);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  pinMode(auxPin, OUTPUT);
  digitalWrite(auxPin, 0);*/
  //analogWrite(fan1Pin, 150);
  //analogWrite(fan2Pin, 150);

  //Serial Initialization
  Serial.begin(9600);
  // Join I2C bus as slave with address 8
  Wire.begin(0x8);
  // Call receiveEvent when data received                
  Wire.onReceive(i2cReceive);
  //Setting Pin Outputs
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(cameraPin, OUTPUT);
  pinMode(killPin, OUTPUT);
  pinMode(switchPin, OUTPUT);
  pinMode(choicePin, INPUT);
  pinMode(programmingPin, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(signalPin, INPUT);
  pinMode(antennaBoard_CS, OUTPUT);
  digitalWrite(antennaBoard_CS, HIGH);
  //pinMode(fan2Pin, OUTPUT);
  pinMode(fan1Pin, OUTPUT);
  pinMode(auxPin, OUTPUT);

  // Initialize Message Struct
  messageReadings.resetTime = 0;
  messageReadings.signalAverageMSB = (signalAverage & 0xFF00) >> 8;
  messageReadings.signalAverageLSB = (signalAverage & 0x00FF);
  messageReadings.roverMode = 1;
  messageReadings.resetConfirmed = false;
  messageIn.resetFlag = false;
  messageIn.statusUpdate = 0;
  messageIn.desiredAngle = 0;

  //Reseting Radio Before Use
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  //Blink Built-In LED If Driver Initialization Failed
  if (!rf95.init())
  {
    while (true)
    {
      blinkLED(750, 250);
    }
  }

  //Blink Built-In LED If Manager Initialization Failed
  if (!manager.init())
  {
    while (true)
    {
      blinkLED(900, 100);
    }
  }

  //Blink Built-In LED If Selected Frequency Failed to Set
  if (!rf95.setFrequency(RFM95_FREQ))
  {
    while (true)
    {
      blinkLED(250, 750);
    }
  }

  /*
   * Sets Transmission Power. Range is from +5dBm to +23dBM which
   * is about 3.2mW to 200mW respectivley. Refer to 
   * https://en.wikipedia.org/wiki/DBm to get dBm benchmarks.
   */
  rf95.setTxPower(23, false); //(power, useRFO)
  
  // Use this block if we have more than 2 feathers
  // roverAddress = readServerAddressFromEeprom();
  // if ((roverAddress < SERVER_ADDRESS_MIN) || (roverAddress > SERVER_ADDRESS_MAX))
  //   roverAddress = SERVER_ADDRESS_DEFAULT;
  manager.setThisAddress(roverAddress);

  digitalWrite(cameraPin, HIGH);
  //delay(5000); // 5 second delay
  digitalWrite(killPin, HIGH);
  digitalWrite(switchPin, HIGH);
  //delay(2500);
  digitalWrite(auxPin, HIGH);
}

//Maximum size buffer the radio driver can support (251)
//uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];  //Gets filled up from client
//uint8_t len = sizeof(buf);             //Length of buf
//uint8_t from;                          //The address that sent the data

void checkMessages()
{
  uint8_t bufR[sizeof(messageIn)]; //Gets filled up from client
  uint8_t len = sizeof(bufR);      //Length of bufR
  uint8_t from;                    //The address that sent the data

  if (currentMillis - previousRecvMillis >= recvInterval)
  {
    if (manager.available())
    {
      //If the data was retrieved successfully, turn on/off LED
      if (manager.recvfrom(bufR, &len, &from))
      {
        memcpy(&messageIn, bufR, sizeof(messageIn));
        Serial.println("Message ================================= Received");
      }

      if (messageIn.resetFlag)
      {
        messageReadings.resetConfirmed = true;
      }
      if (messageIn.desiredAngle != 0 && messageIn.desiredAngle != desiredAntennaAngle) 
      {
        desiredAntennaAngle = messageIn.desiredAngle;
        desiredAngleChangeFlag = true;
      }
      if (messageIn.desiredAngle != 0)
      {
        desiredAntennaAngle = messageIn.desiredAngle;
        desiredAngleChangeFlag = true;
      }
    }
    previousRecvMillis = currentMillis;
  }
}

void sendMessages()
{
  //uint16_t len = sizeof(messageReadings);
  uint8_t len = sizeof(messageReadings);
  memcpy(bufT, &messageReadings, len);
  //Serial.println("In send function");
  if (sendFlag & (currentMillis - previousSendMillis >= sendInterval))
  {
    //Serial.println("Rover Mode: ");
    //Serial.println(messageReadings.roverMode);
    //Serial.println(messageReadings.resetTime);
    //Serial.println(signalAverage);
    //Serial.println((messageReadings.signalAverageMSB << 8) & messageReadings.signalAverageLSB);
    //Serial.println(signalAverage, HEX);
    //Serial.println(messageReadings.signalAverageMSB, HEX);
    //Serial.println(messageReadings.signalAverageLSB, HEX);
    if (manager.sendto(bufT, len, CLIENT_ADDRESS))
    {
      //Serial.println("Message ================================= sent");
    }
    messageReadings.resetConfirmed = false;
    previousSendMillis = currentMillis;
  }
}

void manageConfirmation()
{
  if (messageReadings.resetConfirmed)
  {
    resetConfirmedCounter++;
  }

  if (resetConfirmedCounter >= 1)
  {
    messageReadings.resetConfirmed = false;
  }
}

void temperatureControl()
{

  if (currentMillis - previousTempMillis >= temperatureInterval)
  {
    //thermistorReading = analogRead(thermistorPin) * 0.003; //.003 volts per unit
    thermistorReading = analogRead(thermistorPin); // * 0.0032; //.0032 volts per unit
    if (thermistorReading <= 0)
    {                             // If thermistor reads > 2.26V (65 degrees F) , fans turn on high speed
      digitalWrite(fan1Pin, 250); //250
      //digitalWrite(fan2Pin, 250); //250
      digitalWrite(heaterPin, LOW);
    }
    else if (thermistorReading <= 614)
    {                           // If thermistor reads <= 1.98V (50 degrees F), turn off fans slow, heater on
      digitalWrite(fan1Pin, 0); //0
      //digitalWrite(fan2Pin, 0);
      digitalWrite(heaterPin, HIGH);
    }
    else if (thermistorReading <= 700)
    {                            // If thermistor reads <= 2.26V (65 degrees F), fans turn on slow speed
      analogWrite(fan1Pin, 175); //150
      //digitalWrite(fan2Pin, 215);
      digitalWrite(heaterPin, LOW);
    }
    else if (thermistorReading <= 820)
    {                            // If thermistor reads <= 2.65V (90 degrees F), fans turn on medium speed
      analogWrite(fan1Pin, 230); //200
      //digitalWrite(fan2Pin, 230);
      digitalWrite(heaterPin, LOW);
    }
    else if (thermistorReading > 820)
    {                            // If thermistor reads > 2.65V (90 degrees F) , fans turn on high speed, heater off
      analogWrite(fan1Pin, 250); //250
      //digitalWrite(fan2Pin, 250);
      digitalWrite(heaterPin, LOW);
    }
    previousTempMillis = currentMillis;
  }
}

void reset()
{
  digitalWrite(cameraPin, LOW);
  digitalWrite(switchPin, LOW);
  digitalWrite(auxPin, LOW);
  //pinMode(auxPin, OUTPUT); // How to set from floating to low
  //digitalWrite(auxPin, 0);
  delay(2500);
  digitalWrite(cameraPin, HIGH);
  digitalWrite(switchPin, HIGH);
  digitalWrite(auxPin, HIGH);
  //pinMode(auxPin, INPUT); // How to set pin to floating
  //Serial.println("Message ================================= RESET");
  messageIn.resetFlag = false;
  resetTimeFlag = true;
  resetConfirmedCounter = 0;
  previousResetMillis = currentMillis;
}

void resetAll()
{
  digitalWrite(cameraPin, LOW);
  digitalWrite(switchPin, LOW);
  delay(2500);
  digitalWrite(cameraPin, HIGH);
  digitalWrite(switchPin, HIGH);
  messageIn.resetFlag = false;
  resetTimeFlag = true;
  resetConfirmedCounter = 0;
  resetAllFlag = false;
  previousResetMillis = currentMillis;
}

void resetCamera()
{
  digitalWrite(cameraPin, LOW);
  delay(2500);
  digitalWrite(cameraPin, HIGH);
  messageIn.resetFlag = false;
  resetTimeFlag = true;
  resetConfirmedCounter = 0;
  cameraFlag = false;
  previousResetMillis = currentMillis;
}

void killMotors()
{
  delay(2500);
  messageIn.resetFlag = false;
  resetTimeFlag = true;
  resetConfirmedCounter = 0;
  killFlag = false;
  previousResetMillis = currentMillis;
}

void resetSwitch()
{
  digitalWrite(switchPin, LOW);
  delay(2500);
  digitalWrite(switchPin, HIGH);
  messageIn.resetFlag = false;
  resetTimeFlag = true;
  resetConfirmedCounter = 0;
  switchFlag = false;
  previousResetMillis = currentMillis;
}

void timeCheck()
{
  uint32_t resetTimeDifference = 0;
  resetTimeDifference = currentMillis - previousResetMillis;
  if (resetTimeFlag && (resetTimeDifference > MINUTE_MILLIS * 60))
  {
    messageReadings.resetTime = 100; // Over a day code
    resetTimeFlag = false;
  }
  else if (resetTimeFlag && (resetTimeDifference < MINUTE_MILLIS * 60))
  {
    messageReadings.resetTime = resetTimeDifference / 60000; // Converts from millis to minutes
  }
}

void rpiResetCheck()
{
  if (currentMillis - previousRpiMillis >= rpiInterval)
  {
    if (currentMillis > initialInterval)
    {
      initialFlag = true;
    }

    if (initialFlag)
    {
      if (piCommand == 1) {
        cameraSignal = true;
        killSignal = true;
        switchSignal = true;
        piCommand = -1;
      }
      else
      {
        cameraSignal = false;
        killSignal = false;
        switchSignal = false;
      }
    }

    if (cameraSignal && killSignal && switchSignal) // 
    {                                                // Camera
      messageIn.resetFlag = true;
      messageReadings.resetConfirmed = true;
      resetAllFlag = true;
    }
    else if (cameraSignal) // 
    {                      // Camera
      messageIn.resetFlag = true;
      messageReadings.resetConfirmed = true;
      cameraFlag = true;
    }
    else if (killSignal) // 
    {                     // Motors
      messageIn.resetFlag = true;
      messageReadings.resetConfirmed = true;
      killFlag = true;
    }
    else if (switchSignal) // 
    {                      //Switch
      messageIn.resetFlag = true;
      messageReadings.resetConfirmed = true;
      switchFlag = true;
    }
    previousRpiMillis = currentMillis;
  }
}

void userSetParameters()
{
  if (currentMillis - previousParameterMillis >= parameterInterval)
  {
    long temp;
    if (Serial && Serial.available() > 0)
    {
      Serial.println("Enter 1 - 15 to set address.");
      Serial.print("  Current Address: ");
      Serial.println(roverAddress, HEX);
      Serial.println();

      /* if a number is waiting, process it */
      if (isdigit(Serial.peek()))
      {
        temp = Serial.parseInt();
        if (temp >= 1 && temp <= 15)
        {
          roverAddress = temp;
          manager.setThisAddress(roverAddress);
          writeServerAddressToEeprom(temp);
        }
      }
      /* else throw it away */
      else
      {
        Serial.read();
      }

      Serial.println("");
    }
    previousParameterMillis = currentMillis;
  }
}

void chooseRoverMode()
{
  if (currentMillis - previousChooseMillis >= chooseInterval)
  {
    float choiceReading = analogRead(choicePin);
    if (choiceReading >= 800 && currentVersion != 2) //1.5 Volts
    {
      currentVersion = 2;
      messageReadings.roverMode = 2;
    }
    else if (choiceReading < 800 && currentVersion != 1)
    {
      currentVersion = 1;
      messageReadings.roverMode = 1;
    }
    previousChooseMillis = currentMillis;
  }
}

void checkSendStatus()
{
  if (messageIn.statusUpdate == 1)
  {
    lastSendCommand = currentMillis;
    messageIn.statusUpdate = 0;
    sendFlag = true;
  }

  if (sendFlag)
  {
    if (currentMillis - lastSendCommand >= sendCheckInterval)
    {
      sendFlag = false;
    }
  }
}

void sendMessageSPI()
{
  if (desiredAngleChangeFlag) 
  {
    Serial.println("Made it!");
    Serial.println(desiredAntennaAngle);
    delay(50);
    digitalWrite(RFM95_CS, HIGH);
    digitalWrite(antennaBoard_CS, LOW);
    SPI.transfer(desiredAntennaAngle);
    delay(200);
    digitalWrite(antennaBoard_CS, HIGH);
    digitalWrite(RFM95_CS, LOW);
    desiredAngleChangeFlag = false;    
  }
}

void loop()
{
  //delay(1000);
  //Serial.println(indicatorI2C);
  currentMillis = millis();
  temperatureControl();
  timeCheck();
  checkMessages();
  rpiResetCheck();
  checkSendStatus();
  sendMessages();
  if (messageIn.resetFlag && resetAllFlag)
  {
    resetAll();
    Serial.println("All Reset ");
  }
  else if (messageIn.resetFlag && cameraFlag)
  {
    resetCamera();
    Serial.println("Camera Reset ");
  }
  else if (messageIn.resetFlag && killFlag)
  {
    killMotors();
    Serial.println("Kill Motors ");
  }
  else if (messageIn.resetFlag && switchFlag)
  {
    resetSwitch();
    Serial.println("Switch Reset ");
  }
  else if (messageIn.resetFlag)
  {
    reset();
    Serial.println("Normal Reset ");
  }
  //sendMessageSPI();
  //userSetParameters();
}
