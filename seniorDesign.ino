#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 50

//Digital pin numbers for pitch and roll
#define PITCH_PIN 2
#define ROLL_PIN 3
#define THRESHOLD 4

//Initalize bno and servo motors
Adafruit_BNO055 bno = Adafruit_BNO055(-1,0x28);
Servo pitch;
Servo roll;

void bnoCalibrate(){
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
    pitch.attach(PITCH_PIN);
    roll.attach(ROLL_PIN);
    //Set servos to neutral position (aka flat)
    pitch.write(60);
    roll.write(60);
}

void setup() {
    //initalize serial monitor at 9600 baud
    Serial.begin(9600);
    servoCalibrate();
    bnoCalibrate();
}

void loop() {
    int pitchVal, rollVal;

    while(true){
        //get euler vectors
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        rollVal = map(euler.y(), -90, 90, 0, 120);
        pitchVal = map(euler.z(), -180, 180, 0, 120);

        Serial.print("Roll: ");
        Serial.print(rollVal, DEC);
        Serial.print("Pitch: ");
        Serial.print(pitchVal, DEC);

        //Threshold so not getting perfectly to 0 everytime
        if(pitchVal > THRESHOLD || rollVal > THRESHOLD){
            pitch.write(-1*(pitchVal-60));
            roll.write(-1*(rollVal-60));
        }

        //delay for servos to turn and bno055 sensor
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
}
