#include <SharpIR.h>

#include <x10.h>

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 115

//Digital pin numbers for pitch and roll
#define CLAW_PIN 5

//For distance sensor
#define DISTANCE_THRESHOLD 700
#define TIME_CLAW_SPENT_OPEN 15

//For distance sensor
#define IRPin A1
#define model 1080

SharpIR mySensor = SharpIR(IRPin, model);

class ServoWrapper{
  Servo claw;
  int clawVal;

  void updateServos(){
    if(claw.attached()){
      claw.write(clawVal);
    }
  }
public:

  ServoWrapper(){
    clawVal = 0;
    claw.attach(CLAW_PIN);
    updateServos();
  }

  ~ServoWrapper(){
    claw.detach();
  }

  void relativeChangeClaw(int val){
    clawVal += val;
    if(clawVal > 60){clawVal = 60;}
    if(clawVal < 0){clawVal = 0;}
    updateServos();
  }
  void closeClaw(){
    clawVal = 60;
    updateServos();
  }
  void openClaw(){
    clawVal = 0;
    updateServos();
  }
  void absoluteChangeClaw(int val){
    clawVal = val;
    updateServos();
  }
  int getClawVal(){
    return clawVal;
  }
};

ServoWrapper *ser;

void setup() {
    delay(1000);
    //initalize serial monitor at 9600 baud
    Serial.begin(9600);
    ser = new ServoWrapper();
}

void loop() {
    bool overThreshold = false;
    int counter = 0;
    while(true){
        int dis = analogRead(IRPin);
        Serial.print("Distance Reading: ");
        Serial.println(dis);
        if(dis > DISTANCE_THRESHOLD || overThreshold){
          Serial.print("Counter: ");
          Serial.println(counter);
          //Serial.println(overThreshold);
          if(dis > DISTANCE_THRESHOLD){
            ser->openClaw();
            overThreshold = true;
            counter = 0;
          }
          else{
            counter += 1;
            if(counter >= TIME_CLAW_SPENT_OPEN){
              overThreshold = false;
              counter = 0;
            }
          }
        }
        else{
          //ser->closeClaw();
            ser->relativeChangeClaw(10);
        }

        //delay for servos to turn and BNO055 to process
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

}

