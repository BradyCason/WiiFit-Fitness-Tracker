
// Accelerometer setup
#include <DFRobot_LIS2DH12.h>
/*!
 * @brief Constructor 
 * @param pWire I2c controller
 * @param addr  I2C address(0x18/0x19)
 */
DFRobot_LIS2DH12 acce(&Wire,0x18);

// Barometer Setup
#include <DFRobot_BMP3XX.h>
DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Setup OLED
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;

// Define gloabal variables

// Step count variables
int steps = 0;
int stepsGoal = 20;
float steadyStateMin;
float steadyStateMax;
float previousAccX;
float currentAccX;

// Barometer Variables
float currentAlt;
float currentLanding;
int flightsClimbed = 0;
float landingHeight = 2;
int flightsGoal = 1;

// Piezo buzzer variables

// Button variables
int buttonHeldLoops = 0;

// OLED variables
int pageNum = 0;
bool screenOn = true;
long screenTime = 10000; // in milliseconds
long startMillis;

void setupOLED()
{
  // Code copied from OLED tutorial
  Wire.begin();
  Wire.setClock(400000L);

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  startMillis = millis();
}

void achievedStepsGoal()
{
  // Turn on screen if it's off
  if (screenOn == false)
  {
    toggleScreenOnOff();
  }

  // Play sound and print OLED screen
  int temp = pageNum;
  printOLEDScreen(2);
  int notes[] = {659,698,784,1046,988,1046,784,659,523,587,659,587};
  int delays[] = {1000,1000,400,400,250,400,400,250,400,400,250,400};
  playBuzzer(12, notes, delays);
  printOLEDScreen(temp);
}

void achievedFlightsGoal()
{
  // Turn on screen if it's off
  if (screenOn == false)
  {
    toggleScreenOnOff();
  }

  // Play sound and print OLED screen
  int temp = pageNum;
  printOLEDScreen(3);
  int notes[] = {659,698,784,1046,988,1046,784,659,523,587,659,587};
  int delays[] = {1000,1000,400,400,250,400,400,250,400,400,250,400};
  playBuzzer(12, notes, delays);
  printOLEDScreen(temp);
}

void printOLEDScreen(int pageToPrint)
{
  // Print the page corresponding to pageToPrint to the OLED

  if (pageToPrint == 0)
  {
    // Print page 0 to OLED
    oled.clear();
    oled.print("Steps: ");
    oled.set2X();
    oled.println(steps);
    oled.set1X();
    oled.print("Goal: ");
    oled.set2X();
    oled.println(stepsGoal);
    oled.set1X();
    oled.print("Calories: ");
    oled.set2X();
    oled.println(0.04 * steps);
    oled.set1X();
  }
  else if (pageToPrint == 1)
  {
    // Print page 1 to OLED
    oled.clear();
    oled.print("Flights Climbed: ");
    oled.set2X();
    oled.println(flightsClimbed);
    oled.set1X();
    oled.print("Goal: ");
    oled.set2X();
    oled.println(flightsGoal);
    oled.set1X();
  }
  else if (pageToPrint == 2)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Congrats!");
    oled.set1X();
    oled.println("You reached your");
    oled.println("goal of:");
    oled.set2X();
    oled.print(stepsGoal);
    oled.println(" steps!");
    oled.set1X();
  }
  else if (pageToPrint == 3)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Congrats!");
    oled.set1X();
    oled.println("You reached your");
    oled.println("goal of:");
    oled.set2X();
    oled.print(flightsGoal);
    oled.println(" flights!");
    oled.set1X();
  }
  else if (pageToPrint == 4)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Please");
    oled.println("wait...");
    oled.set1X();
  }
}

void toggleScreenOnOff()
{
  // If OLED is on, turn it off. If OLED is off, turn it on

  if (screenOn)
  {
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    screenOn = false;
  }
  else
  {
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
    screenOn = true;
  }
}

void OLEDLogic()
{
  // Variable for milliseconds
  unsigned long currentMillis = millis();
  
  if (digitalRead(D5
  ) == LOW)
  {
    // Button is pressed

    // restart timer until turn off
    startMillis = currentMillis;

    // Determine what to do with the input
    if (screenOn)
    {
      // Toggle page number
      if (pageNum == 0 && buttonHeldLoops == 0)
      {
        pageNum = 1;
      }
      else if (pageNum == 1 && buttonHeldLoops == 0)
      {
        pageNum = 0;
      }

      // Print page
      printOLEDScreen(pageNum);
    }
    else
    {
      toggleScreenOnOff();
    }
    
    // Check if button has been held for 5 cycles
    buttonHeldLoops++;
    if (buttonHeldLoops >= 5){
      buttonHeldLoops = 0;
      
      // Reset the data
      resetData();
    }

    // Delay so that button is not considered to be pressed twice if held for a while
    delay(300);
  }
  else{
    buttonHeldLoops = 0;
  }

  // if no button press for a certain time, turn off screen
  if ((currentMillis - startMillis >= screenTime) && screenOn)
  {
    toggleScreenOnOff();
  }
}

void getAccelerometerData()
{
  //Assign the current acceleration to the previous
  previousAccX = currentAccX;
  
  //Get new current acceleration from accelerometer
  currentAccX = acce.readAccY();//Get the acceleration in the x direction (actually getting y, but all the variables are x)
}

void getBarometerData()
{
  // Get currentAlt and altitudeDif
  currentAlt = sensor.readAltitudeM();
  float altitudeDif = currentAlt - currentLanding;

  if (altitudeDif >= landingHeight - 0.15)
  {
    // Increased landing height

    flightsClimbed += 1;
    currentLanding += landingHeight;
    
    // Check if achieved goal
    if (flightsClimbed == flightsGoal)
    {
      achievedFlightsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }
  else if (altitudeDif <= -landingHeight + 0.15)
  {
    // Decreased landing height
    currentLanding -= landingHeight;
  }
}

void incrementSteps()
{
  //Check if acceleration changed from above steady state max to under
  if (previousAccX > steadyStateMax && currentAccX <= steadyStateMax)
  {
    steps++;

    if (steps == stepsGoal)
    {
      achievedStepsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }

  //Check if acceleration changed from below steady state min to above
  if (previousAccX < steadyStateMin && currentAccX >= steadyStateMin)
  {
    steps++;

    if (steps == stepsGoal)
    {
      achievedStepsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }
}

void playBuzzer(int numNotes, int notes[], int delays[])
{
  // Loop through each note and play it for the assigned duration
  for(int i = 0; i < numNotes; i++)
  {
    tone(D8, notes[i], delays[i]);
  	delay(delays[i]);
  }
}

void setupAccelerometer()
{
  //Chip initialization
  while(!acce.begin()){
     //Serial.println("Initialization failed, please check the connection and I2C address settings");
     delay(1000);
  }
  //Get chip id
  //Serial.print("chip id : ");
  //Serial.println(acce.getID(),HEX);
  /**
    set range:Range(g)
              eLIS2DH12_2g,/< ±2g>/
              eLIS2DH12_4g,/< ±4g>/
              eLIS2DH12_8g,/< ±8g>/
              eLIS2DH12_16g,/< ±16g>/
  */
  acce.setRange(/*Range = */DFRobot_LIS2DH12::eLIS2DH12_16g);
  /**
    Set data measurement rate：
      ePowerDown_0Hz 
      eLowPower_1Hz 
      eLowPower_10Hz 
      eLowPower_25Hz 
      eLowPower_50Hz 
      eLowPower_100Hz
      eLowPower_200Hz
      eLowPower_400Hz
  */
  acce.setAcquireRate(/*Rate = */DFRobot_LIS2DH12::eLowPower_10Hz);
  delay(1000);
}

void setupBarometer()
{
  // Code copied from tutorial

  int rslt;
  while( ERR_OK != (rslt = sensor.begin()) )
  {
    if(ERR_DATA_BUS == rslt)
    {
      //Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == rslt)
    {
      //Serial.println("Chip versions do not match!!!");
    }
    delay(3000);
  }
  //Serial.println("Begin ok!");

  while( !sensor.setSamplingMode(sensor.eUltraPrecision) )
  {
    //Serial.println("Set samping mode fail, retrying....");
    delay(3000);
  }

  delay(100);
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  /**
   * Calibrate the sensor according to the current altitude
   * In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). 
   * Please change to the local altitude when using it.
   * If this interface is not called, the measurement data will not eliminate the absolute difference.
   * Notice: This interface is only valid for the first call.
   */
  if( sensor.calibratedAbsoluteDifference(540.0) ){
    //Serial.println("Absolute difference base value set successfully!");
  }
  #endif

  float sampingPeriodus = sensor.getSamplingPeriodUS();
  //Serial.print("samping period : ");
  //Serial.print(sampingPeriodus);
  //Serial.println(" us");

  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  //Serial.print("samping frequency : ");
  //Serial.print(sampingFrequencyHz);
  //Serial.println(" Hz");

  //Serial.println();
}

void getSteadyState(void)
{
  // Get first acceleration
  float x = acce.readAccY();

  // Set original min and max to the first x value read
  steadyStateMin = x;
  steadyStateMax = x;

  // Reapeat 10000 times
  for(int i = 0; i < 1000; i++){

    // Get acceleration
    x = acce.readAccY();

    if (x < steadyStateMin){
      // If x is less than the current min, set the min to x
      steadyStateMin = x;
    }
    if (x > steadyStateMax){
      // If x is more than the current max, set the max to x
      steadyStateMax = x;
    }

    // Delay
    delay(10);
  }

  startMillis = millis();

  // Increase range of not stepping
  //steadyStateMin -= 50;
  //steadyStateMax += 50;

  //Serial.println(steadyStateMin);
  //Serial.println(steadyStateMax);
}

void resetData()
{
  // Run this function at the start and whenever resetting the program

  // Reset steps and flights
  steps = 0;
  flightsClimbed = 0;

  // Get original landing altitude
  currentAlt = sensor.readAltitudeM();
  currentLanding = currentAlt;

  // Get original steps
  currentAccX = acce.readAccY();

  // Print please wait message to oled
  printOLEDScreen(4);

  // Play sound for turn on
  pinMode(D8, OUTPUT);
  int notes[] = {523,587,698,659,587,523,784};
  int delays[] = {250,250,500,500,250,500,1000};
  playBuzzer(7, notes, delays);

  // Calculate the steady state of the accelerometer
  getSteadyState();
  //Serial.print("Max: ");
  //Serial.println(steadyStateMax);
  //Serial.print("Min: ");
  //Serial.println(steadyStateMin);

  // Setup Button
  pinMode(D5, INPUT_PULLUP);

  // Restart timer for turning off OLED
  startMillis = millis();
  printOLEDScreen(pageNum);
}

void setup(void)
{

  // Begin serial communication
  Serial.begin(115200);

  setupAccelerometer();
  setupBarometer();
  setupOLED();

  // Reset steps, flights, etc. and calc steady state
  resetData();
}

void loop(void)
{

  // Get sensor data
  getAccelerometerData();
  getBarometerData();

  // Logic involving oled display (button input and turning on/ off)
  OLEDLogic();

  // Increment steps if satisfying conditions
  incrementSteps();

  //Serial.println(currentAccX);

  delay(100);
}
