#include<HardwareSerial.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// The Stepper pins
#define STEPPER_DIR_PIN 4
#define STEPPER_STEP_PIN 2
#define STEPPER_EN_PIN 15

// The serials
#define DebugSerial Serial
HardwareSerial HmiSerial(1);
HardwareSerial PcSerial(2);

// Digital pin connected to the DHT sensor
#define DHT_PIN 23        
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// The Light Pins
#define TOP_LIGHT_PIN 12
#define BOTTOM_LIGHT_PIN 14

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

byte nextionReturnBuffer[128];                      // Byte array to pass around data coming from the panel
uint8_t nextionReturnIndex = 0;                     // Index for nextionReturnBuffer
bool lcdConnected = false;                          // Set to true when we've heard something from the LCD
static int distance = 0;
static int stepperspeed = 1;
bool toplight = 0;//0 for toplight off,1 for toplight on
bool bottomlight = 0;//0 for bottomlight off,1 for bottomlight on
bool stepperswitch = 1;//0 for stepper off,1 for stepper on
bool stepperstate = 0;//0 for clockwise,1 for otherwise 

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
VL53L0X sensor;

////////////////////////////////////////////////////////
//Tasks

//Task stepper
void StepperTask(void *param){
  while(1){
    if(stepperswitch==1&&distance>=150){
      stepper.runSpeed();      
      }
    //DebugSerial.println("stepper runing");
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
  
}

//Task distance
void DistanceTask(void *param){
  while(1){
    DebugSerial.print(sensor.readRangeContinuousMillimeters());
    if (sensor.timeoutOccurred()) { DebugSerial.print(" TIMEOUT"); }
    DebugSerial.println();   
    distance= sensor.readRangeContinuousMillimeters();
    vTaskDelay(500 / portTICK_PERIOD_MS);   
  }  
}

//Task HmiIn
void HmiInTask(void *param){
  while(1){
   if (nextionHandleInput())
    { // Process user input from HMI
      nextionProcessInput();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }  
}

//Task environment
void EnvironmentTask(void *param){
  while(1){

  }  
}

//Task light
void LightTask(void *param){
  while(1){
    if(toplight==0){
      digitalWrite(TOP_LIGHT_PIN,0);
      DebugSerial.println("Turn TOPLIGHT OFF");
      }
    else{
      digitalWrite(TOP_LIGHT_PIN,1);
      DebugSerial.println("Turn TOPLIGHT ON");      
      }
    if(bottomlight==0){
      digitalWrite(BOTTOM_LIGHT_PIN,0);
      DebugSerial.println("Turn BOTTOM_LIGHT_PIN OFF");
      }
    else{
      digitalWrite(BOTTOM_LIGHT_PIN,1);
      DebugSerial.println("Turn BOTTOM_LIGHT_PIN ON");      
      }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }  
}

///////////////////////////////////////////////////////

bool nextionHandleInput()
{ // Handle incoming serial data from the Nextion panel
  // This will collect serial data from the panel and place it into the global buffer
  // nextionReturnBuffer[nextionReturnIndex]
  // Return: true if we've received a string of 3 consecutive 0xFF values
  // Return: false otherwise
  bool nextionCommandComplete = false;
  static int nextionTermByteCnt = 0;   // counter for our 3 consecutive 0xFFs
  static String hmiDebug = "HMI IN: "; // assemble a string for debug output

  if (HmiSerial.available())
  {
    lcdConnected = true;
    byte nextionCommandByte = HmiSerial.read();
    hmiDebug += (" 0x" + String(nextionCommandByte, HEX));
    // check to see if we have one of 3 consecutive 0xFF which indicates the end of a command
    if (nextionCommandByte == 0xFF)
    {
      nextionTermByteCnt++;
      if (nextionTermByteCnt >= 3)
      { // We have received a complete command
        nextionCommandComplete = true;
        nextionTermByteCnt = 0; // reset counter
      }
    }
    else
    {
      nextionTermByteCnt = 0; // reset counter if a non-term byte was encountered
    }
    nextionReturnBuffer[nextionReturnIndex] = nextionCommandByte;
    nextionReturnIndex++;
  }
  if (nextionCommandComplete)
  {
    DebugSerial.println(hmiDebug);
    hmiDebug = "HMI IN: ";
  }
  return nextionCommandComplete;
}

void nextionProcessInput()
{ // Process incoming serial commands from the Nextion panel
  // Command reference: https://www.itead.cc/wiki/Nextion_Instruction_Set#Format_of_Device_Return_Data
  // tl;dr, command byte, command data, 0xFF 0xFF 0xFF

  if (nextionReturnBuffer[0] == 0x10)
  { // Handle incoming touch command
    // 0x10+stepperswitch+End
    // Return this data when the touch event created by the user is pressed.
    // Definition of TouchEvent: Press Event 0x01, Release Event 0X00
    // Example: 0x01 0x01 0x00 0x00 0xFF 0xFF 0xFF
    if (nextionReturnBuffer[1] == 0x01)
    {
      DebugSerial.println(String(F("HMI IN: [STEPPER ON]")));
      stepperswitch = 1;
    }
    if (nextionReturnBuffer[1] == 0x00)
    {
      DebugSerial.println(String(F("HMI IN: [STEPPER ON]")));
      stepperswitch = 0;
    }
  }
  else if (nextionReturnBuffer[0] == 0x11)
  { // Handle incoming touch command
    // 0x10+stepperstate+End
    // Return this data when the touch event created by the user is pressed.
    // Definition of TouchEvent: Press Event 0x01, Release Event 0X00
    // Example: 0x11 0x01 0x00 0x00 0xFF 0xFF 0xFF
    if (nextionReturnBuffer[1] == 0x01)
    {
      DebugSerial.println(String(F("HMI IN: [STEPPER GO CLOCKWISE]")));
      stepperstate = 1;
      stepper.setSpeed(stepperspeed); 
    }
    if (nextionReturnBuffer[1] == 0x00)
    {
      DebugSerial.println(String(F("HMI IN: [STEPPER GO OTHERWISE]")));
      stepperstate = 0;
      stepper.setSpeed(-stepperspeed); 
    }
  }
  else if (nextionReturnBuffer[0] == 0x12)
  { // Handle incoming touch command
    // 0x12+toplight+End
    // Return this data when the touch event created by the user is pressed.
    // Definition of TouchEvent: Press Event 0x01, Release Event 0X00
    // Example: 0x12 0x01 0x00 0x00 0xFF 0xFF 0xFF
    if (nextionReturnBuffer[1] == 0x01)
    {
      DebugSerial.println(String(F("HMI IN: [TOPLIGHT ON]")));
      toplight = 1;
    }
    if (nextionReturnBuffer[1] == 0x00)
    {
      DebugSerial.println(String(F("HMI IN: [TOPLIGHT OFF]")));
      toplight = 0;
    }
  }
 else if (nextionReturnBuffer[0] == 0x13)
  { // Handle incoming touch command
    // 0x13+bottomlight+End
    // Return this data when the touch event created by the user is pressed.
    // Definition of TouchEvent: Press Event 0x01, Release Event 0X00
    // Example: 0x13 0x01 0x00 0x00 0xFF 0xFF 0xFF
    if (nextionReturnBuffer[1] == 0x01)
    {
      DebugSerial.println(String(F("HMI IN: [BOTTOMLIGHT ON]")));
      bottomlight = 1;
    }
    if (nextionReturnBuffer[1] == 0x00)
    {
      DebugSerial.println(String(F("HMI IN: [BOTTOMLIGHT OFF]")));
      bottomlight = 0;
    }
  }
  else if (nextionReturnBuffer[0] == 0x71)
  { // Handle get int return
    // 0x71+byte1+byte2+byte3+byte4+End (4 byte little endian)
    // Example: 0x71 0x7B 0x00 0x00 0x00 0xFF 0xFF 0xFF
    // Meaning: Integer data, 123
    unsigned long getInt = nextionReturnBuffer[1];
    stepperspeed = getInt*5;
    if(stepperstate){
      stepper.setSpeed(stepperspeed);      
      }
    else{
      stepper.setSpeed(-stepperspeed);
      }
    DebugSerial.print("SET STEPPERSPEED TO:");
    DebugSerial.println(stepperspeed);
  }
  else if (nextionReturnBuffer[0] == 0x65)
  { // Handle get int return
    // 0x71+byte1+byte2+byte3+byte4+End (4 byte little endian)
    // Example: 0x71 0x7B 0x00 0x00 0x00 0xFF 0xFF 0xFF
    // Meaning: Integer data, 123
    if (nextionReturnBuffer[2] == 0x04){
      DebugSerial.println("SET BACKGROUND TO SAKURA");
      PcSerial.println("SET BACKGROUND TO SAKURA");
      }
    else if (nextionReturnBuffer[2] == 0x05){
      DebugSerial.println("SET BACKGROUND TO SEA");
      PcSerial.println("SET BACKGROUND TO SEA");      
      }
    else if (nextionReturnBuffer[2] == 0x06){
      DebugSerial.println("SET BACKGROUND TO FIRE");
      PcSerial.println("SET BACKGROUND TO FIRE");      
      }
    else if (nextionReturnBuffer[2] == 0x07){
      DebugSerial.println("SET BACKGROUND TO LIGHTING");
      PcSerial.println("SET BACKGROUND TO LIGHTING");      
      }
    else if (nextionReturnBuffer[2] == 0x08){
      DebugSerial.println("SET BACKGROUND TO TREE");
      PcSerial.println("SET BACKGROUND TO TREE");      
      }
    else if (nextionReturnBuffer[2] == 0x09){
      DebugSerial.println("SET BACKGROUND TO STAGE");
      PcSerial.println("SET BACKGROUND TO STAGE");      
      }
    else if (nextionReturnBuffer[2] == 0x0A){
      DebugSerial.println("SET BACKGROUND TO STAR");
      PcSerial.println("SET BACKGROUND TO STAR");      
      }
    else if (nextionReturnBuffer[2] == 0x0B){
      DebugSerial.println("SET BACKGROUND TO SNOW");
      PcSerial.println("SET BACKGROUND TO SNOW");      
      }
  }
  nextionReturnIndex = 0; // Done handling the buffer, reset index back to 0
}

//////////////////////////////////////////////////////////////////////////////

void setup(){
  DebugSerial.begin(115200);
  HmiSerial.begin(9600,SERIAL_8N1,16,17);
  PcSerial.begin(115200,SERIAL_8N1,5,18);
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(50);
  pinMode(TOP_LIGHT_PIN, OUTPUT);
  pinMode(BOTTOM_LIGHT_PIN, OUTPUT);
  delay(1000);
  DebugSerial.println("start system!");
  Wire.begin();
  sensor.setTimeout(1000);
  if (!sensor.init())
  {
    DebugSerial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
  
  xTaskCreatePinnedToCore(
    StepperTask
    ,  "StepperTask"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);//Use pro_cpu
  xTaskCreatePinnedToCore(
    DistanceTask
    ,  "DistanceTask"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); //App pro_cpu
  xTaskCreatePinnedToCore(
    HmiInTask
    ,  "HmiInTask"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); //App pro_cpu
  xTaskCreatePinnedToCore(
    EnvironmentTask
    ,  "EnvironmentTask"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); //App pro_cpu
  xTaskCreatePinnedToCore(
    LightTask
    ,  "LightTask"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1); //App pro_cpu
}

void loop(){
  
}
