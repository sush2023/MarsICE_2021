/*
* Pitt SOAR
* This is the comprehensive code for controlling the drill via MATLAB and Arduino
* Updated 08/26/2021
*/

/*THINGS THAT STILL NEED DONE:
*/

#include "CurrentMon.h"
#include<HX711.h> //Load cell library

//SERIAL COMMUNICATION VARIABLES
int sref = -1;

#define loadCellData 21
#define loadCellClk 53

struct dWrite{
    int port;
    int pin;
    int val;
};

Timer<10, micros> T;
HX711 forceSensor;
CurrentMon ISense(0, 0, 1, 250, 1, T);

#define UPPER_LIMIT 8.9 //If the current goes above this stop everything

#define MOTOR_STEPS 800
#define RPM 90 

//calculation for length of delay used to control vertical speed (miliseconds)
unsigned long stepDelay = 4166;
int distance = 0; //step count

// All the wires needed for full functionality; motor 1 (vertical) and motor2 (tool change)
#define vertStepPin 2
#define vertDirPin 48
#define toolChangeStepPin 46
#define toolChangeDirPin 44
//limit switch digital pins
#define topLimit 24
#define botLimit 26
//actuator pins
#define act1 38
#define act2 36
// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper2(MOTOR_STEPS2, toolChangeDirPin, toolChangeStepPin);

//RELAY DEFINITIONS
#define DRILL  13
#define PROBE  12
#define PUMP  11

#define VALVE1  10
#define VALVE2  9
#define VALVE3  8
#define VALVE4  3

const static void (*funcMap[])() = {  &drillDown,     //Command 1 Turns Drill on and goes down
                                      &drillUp,       //Command 2 Turns Drill on and goes up
                                      &heaterOn,      //Command 3 Turns Heater on and waits
                                      &drillOn,       //Command 4 Turns the drill on and waits
                                      &pumpOn,        //Command 5 Turns the pump on and waits
                                      &toolChangeCW,  //Command 6 Turns the tool changer Clockwise
                                      &toolChangeCC,  //Command 7 Turns the tool changer CounterClockwise
                                      &toolRealease,  //Command 8 Pulls the node cone up and releases the tool
                                      &toolGrab,      //Command 9 Puts the nose cone down to grab tool
                                      &heaterDown,    //Command 10 Turns the heater on and moves down
                                      &heaterUp,      //Command 11 Turns the heater on and moves up
                                      &goUp,          //Command 12 Move up with nothing on
                                      &goDown,        //Command 13
                                      &compresserDown //Command 14
                                      &loadCellCal,   //Command 15
                                      &ampMetterCal,  //16
                                      [](void*){forceSensor.tare();}};//Command 17 Move down with nothing on    

void setup() {

    Serial.begin(115200);

    //initialize stepper motors
    pinMode(vertStepPin,OUTPUT);
    pinMode(vertDirPin,OUTPUT);
    pinMode(act1,OUTPUT);
    pinMode(act2,OUTPUT);
    pinMode(toolChangeStepPin,OUTPUT);
    pinMode(toolChangeDirPin,OUTPUT);

    //initializes limit switches
    pinMode(topLimit,INPUT);
    pinMode(botLimit,INPUT);

    //initializes relays
    pinMode(DRILL,OUTPUT);
    pinMode(PUMP,OUTPUT);
    pinMode(PROBE,OUTPUT);
    pinMode(VALVE1,OUTPUT);
    pinMode(VALVE2,OUTPUT);
    pinMode(VALVE3,OUTPUT);
    
    //Turns everything off
    stopAll();

    //This code initializes force sensor
    forceSensor.begin(loadCellData, loadCellClk);
    forceSensor.set_scale(4540.09554733474); // loadcell factor 5 KG
    forceSensor.tare(); //zeroes load cell

}//end setup

void loop()
{  
  T.tick();//Update the timer
  ISense.updateIrms();//Get the latest current reading
  checkIrms();//Check to make sure we're not drawing too much current
  updateState();//update what we're doing
} 

void updateState(){
    if(!Serial){
      stopAll();//If the computer is disconeted STOP EVERYTHING
    }else{
      if(Serial.available() > 0){//If a new state is being commanded from matlab
        int srefTemp = Serial.parseInt();
        Serial.read();//CLEAR THE BUFFER 

        if(sref != srefTemp){//If the state is new

          sref = srefTemp; //Update the real sref
          stopAll(); // if the state changed go back to the safe state this makes it easy to go to any other state

        }
      }
    }

    for(size_t i; !T.empty(); i++){
        T.tick();//Wait for the last task to finish before starting the next one
        delayMicroseconds(1);
        if(i > 10000000){
          stopAll();//If we're waiting more than a full second for a task to complete turn everything off
        }
    }

    if(sref <= 16 && sref >= 1){
      funcMap[sref-1](); //Call the function according to the state we're in based on the map at the top of the code
    }else{
      stopAll();//If the state is invalid turn everything off This shouldn't be nessary it should happen on the state change but it's here for redundancy
    }
}

void drillDown(){
  if(forceSensor.is_ready()){
    printDigitalCore();
  }
  drillOn();
  goDown();
}

void compresserDown(){
  pumpOn();
  goDown();
}

void heaterDown(){
  heaterOn();
  goDown();
}

void drillUp(){
  drillOn();
  goUp();
}

void heaterUp(){
  heaterOn();
  goUp();
}

void toolChangeCW(){
  digitalWrite(toolChangeDirPin, HIGH);//Low for down
  stepMotor(toolChangeStepPin);
}

void toolChangeCC(){
  digitalWrite(toolChangeDirPin, LOW);//Low for down
  stepMotor(toolChangeStepPin);
}

void loadCellCal(){
  Serial.println(forceSensor.get_units(5));
}

void ampMetterCal(){
  Serial.println(ISense.getLastIrms());
}

void checkIrms() {
    while(ISense.getLastIrms() > UPPER_LIMIT) {
      stopAll();
      ISense.updateIrms();
      Serial.print("CURRENT EXCEEDED UPPED LIMIT :");
      Serial.println(ISense.getLastIrms());
      sref = -1;
    }
}

void stopAll(){
    digitalWrite(DRILL,HIGH);//all relays should be HIGH to be off
    digitalWrite(PUMP,HIGH);
    digitalWrite(PROBE,HIGH);
    digitalWrite(VALVE1, HIGH);
    digitalWrite(VALVE2, HIGH);
    digitalWrite(VALVE3, HIGH);
}

void goDown(void){

  //continue until signal change or limit switch
  if(!digitalRead(botLimit)){
    stepDrillDown();
  }else{
    //TODO set dist to lower val
  }
}

void stepMotor(int motorPin){
  
  dWrite *dat = malloc(sizeof(dWrite));

  dat->pin = motorPin;
  dat->val = LOW;

  digitalWrite(motorPin, HIGH);

  T.in(stepDelay, &stepDown, (void*)dat);
  T.in(stepDelay*2, [](dWrite * d){free(d);}, dat);
  
}

bool stepDown(void *dat){

  dWrite args = *((dWrite *)dat);

  digitalWrite(args.pin, args.val);
  return false;
}

void stepDrillDown(){

  digitalWrite(vertDirPin, HIGH);//High for descent
  stepMotor(vertStepPin);
  distance++;
}

void stepDrillUp(){

  digitalWrite(vertDirPin, LOW);//Low for down
  stepMotor(vertStepPin);
  distance--;

}

void goUp(void)
{
  if(!digitalRead(topLimit)){ //reverses unless stopped or limit switch is hit
    stepDrillUp();
  }else{
    distance = 0;
  }
}

void heaterOn(void){
  digitalWrite(PROBE,LOW); 
}

void pumpOn(void){
  digitalWrite(PUMP, LOW);
}

void drillOn(void){
  digitalWrite(DRILL,LOW);
}

void valveOpen(int valvePin){
  digitalWrite(valvePin, LOW);
}

void printDigitalCore(void)
{
    if(forceSensor.is_ready()){
        Serial.print(distance);
        Serial.print(" ");
        Serial.print(forceSensor.get_units(1));
        Serial.print(" ");
        Serial.print(ISense.getLastIrms());
        Serial.print(" ");
        Serial.println(millis());

    }
}

void toolRealease(){
    digitalWrite(act1,LOW);
    digitalWrite(act2, HIGH);
}

void toolGrab(){
    digitalWrite(act1,HIGH);
    digitalWrite(act2,LOW);
}
