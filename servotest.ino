#include <Servo.h>
#include <Wire.h>

//Digital pin numbers for pitch and roll
#define PITCH_PIN 2
#define ROLL_PIN 3

//Initalize bno and servo motors
Servo pitch;
Servo roll;

void servoCalibrate(){
    //Attach servos to correct digital pins
    pitch.attach(PITCH_PIN);
    roll.attach(ROLL_PIN);
    //Set servos to neutral position (aka flat)
    pitch.write(0);
    roll.write(0);
}


void setup() {
    //initalize serial monitor at 9600 baud
    Serial.begin(9600);
    servoCalibrate();
}

void loop() {
    int pitchVal = 0;
    int rollVal = 0;

    for( ; pitchVal < 180; pitchVal += 15){
        pitch.write(pitchVal);
        roll.write(180 - pitchVal);
    }

    for( ; pitchVal >= 0; pitchVal -= 15){
        pitch.write(pitchVal);
        roll.write(180 - pitchVal);
    }

}
