#include <SharpIR.h>

#include <x10.h>

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 115

//Digital pin numbers for pitch and roll
#define PITCH_PIN 2
#define ROLL_PIN 3
#define CLAW_PIN 5

//For distance sensor
#define DISTANCE_THRESHOLD 700
#define TIME_CLAW_SPENT_OPEN 15

//For response curve and pitch roll motors
#define THRESHOLD 5
#define STEP 5

//For distance sensor
#define IRPin A1
#define model 1080

SharpIR mySensor = SharpIR(IRPin, model);

//Initalize bno and servo motors
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);


class ServoWrapper{
  Servo pitch;
  Servo roll;
  Servo claw;
  int pitchVal;
  int rollVal;
  int clawVal;

  void updateServos(){
    if(pitch.attached()){
      pitch.write(pitchVal);
    }
    if(roll.attached()){
      roll.write(rollVal);
    }
    if(claw.attached()){
      claw.write(clawVal);
    }
  }
public:

  ServoWrapper(){
    pitchVal = 90;
    rollVal = 90;
    clawVal = 0;
    pitch.attach(PITCH_PIN);
    roll.attach(ROLL_PIN);
    claw.attach(CLAW_PIN);
    updateServos();
  }

  ~ServoWrapper(){
    pitch.detach();
    roll.detach();
    claw.detach();
  }

  void relativeChangePitch(int val){
    pitchVal += val;
    if(pitchVal > 180){pitchVal = 180;}
    if(pitchVal < 0){pitchVal = 0;}
    updateServos();
  }
  void absoluteChangePitch(int val){
    pitchVal = val;
    updateServos();
  }
  void relativeChangeRoll(int val){
    rollVal += val;
    if(rollVal > 180){rollVal = 180;}
    if(rollVal < 0){rollVal = 0;}
    updateServos();
  }
  void absoluteChangeRoll(int val){
    rollVal = val;
    updateServos();
  }
  void relativeChangeClaw(int val){
    clawVal += val;
    if(clawVal > 95){clawVal = 95;}
    if(clawVal < 0){clawVal = 0;}
    updateServos();
  }
  void closeClaw(){
    clawVal = 180;
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
  int getPitchVal(){
    return pitchVal;
  }
  int getRollVal(){
    return rollVal;
  }
  int getClawVal(){
    return clawVal;
  }
};

ServoWrapper *ser;

float getResponseCurve(int val){
  return (val-90.0)*(val-90.0)*(val-90.0)*0.0001;
}


void bnoSetup(){
    //check to make sure sensor is detected
    if(!bno.begin()){
        Serial.print("BNO055 not detected");
        while(1);
    }

    //use external crystal for faster readings
    bno.setExtCrystalUse(true);
}

void servoCalibrate(){
    //Attach servos to correct digital pins
    //pitch.attach(PITCH_PIN);
    //roll.attach(ROLL_PIN);
    //Set servos to neutral position (aka flat)
    //pitch.write(90);
    //roll.write(90);
    ser = new ServoWrapper();
}


void setup() {
    delay(1000);
    //initalize serial monitor at 9600 baud
    Serial.begin(9600);
    servoCalibrate();
    bnoSetup();
}

void loop() {
    int pitchVal, rollVal;
    bool overThreshold = false;
    int counter = 0;
    while(true){
        //get euler vectors
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        rollVal = map(euler.y(), -90, 90, 0, 180);
        pitchVal = euler.z() + 90;

        Serial.println();
        Serial.print("Roll: ");
        Serial.println(rollVal, DEC);
        Serial.print("Pitch: ");
        Serial.println(pitchVal, DEC);


        //threshold code
        if(ser->getPitchVal() < pitchVal-THRESHOLD || ser->getPitchVal() > pitchVal+THRESHOLD){
          ser->absoluteChangePitch(pitchVal);
        }
        if(ser->getRollVal() < rollVal-THRESHOLD || ser->getRollVal() > rollVal+THRESHOLD){
          ser->absoluteChangeRoll(rollVal);
        }


        //printouts to test response curve
        /*  Serial.print(ser->getPitchVal());
          Serial.print("     ");
          Serial.println(pitchVal+THRESHOLD);
          Serial.println(getResponseCurve(pitchVal));
          Serial.println(pitchVal - ser->getPitchVal());*/

       /*   if(ser->getPitchVal() < pitchVal-THRESHOLD){
            //ser->relativeChangePitch(STEP);
            ser->relativeChangePitch(max((int)getResponseCurve(pitchVal),1));
          }
          else if(ser->getPitchVal() > pitchVal+THRESHOLD){
            ser->relativeChangePitch(min((int)getResponseCurve(pitchVal),-1));
            //ser->relativeChangePitch(-1*STEP);
          }*/

       /*   if(ser->getRollVal() < rollVal-THRESHOLD){
            //ser->relativeChangeRoll(STEP);
            ser->relativeChangeRoll(max((int)getResponseCurve(rollVal),1));
          }
          else if(ser->getRollVal() > rollVal+THRESHOLD){
            ser->relativeChangeRoll(min((int)getResponseCurve(rollVal),-1));
            //ser->relativeChangeRoll(-1*STEP);
          }*/



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
          ser->closeClaw();
        }

        //delay for servos to turn and BNO055 to process
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

}
