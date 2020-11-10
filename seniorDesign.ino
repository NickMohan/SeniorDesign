#include <Servo.h>
#include <Wire.h>
#include <Adafruit_sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

//Digital pin numbers for pitch and roll
#define PITCH_PIN 2
#define ROLL_PIN 3


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
    //calibrate the sensor
    uint8_t sys, gyro, accel, mag;
    sys = gyro = accel = mag = 0;
    bno.getCalibration(sys, gyro, accel, mag);
    //While waiting for calibration of gyro
    while(!gyro || !sys){
        Serial.print("Waiting for calibration");
        delay(15);
    }
    //print final calibration status on gyro
    Serial.print("Gyro Calibration Status:");
    Serial.print(gyro, DEC);

    //use external crystal for faster readings
    bno.setExtCrystalUse(true);

}

void servoCalibrate(){
    //Attach servos to correct digital pins
    pitch.attach(PITCH_PIN);
    roll.attach(ROLL_PIN);
    //Set servos to neutral position (aka flat)
    pitch.write(90);
    roll.write(90);
}


void setup() {
    //initalize serial monitor at 9600 baud
    Serial.begin(9600);
    servoCalibrate();
    bnoCalibrate();
}

void loop() {
    int pitchVal, rollVal;
//    int yawVal;

    while(true){
        //get euler vectors
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//        yawVal = map(euler.x(), 0, 360, 0, 180);
        rollVal = map(euler.y(), -90, 90, 0, 180);
        pitchVal = map(euler.z(), -180, 180, 0, 180);

        Serial.print("Roll: ");
        Serial.print(rollVal, DEC);
        Serial.print("Pitch: ");
        Serial.print(pitchVal, DEC);

        pitch.write(pitchVal);
        roll.write(rollVal);
        //delay for servos to turn
        delay(15)

        delay(BNO055_SAMPLERATE_DELAY_MS);
    }


}
