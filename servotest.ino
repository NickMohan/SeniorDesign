#include <Servo.h>
#include <Wire.h>

//Digital pin numbers for pitch and roll
#define PITCH_PIN 2

//Initalize bno and servo motors
Servo pitch;

void setup() {
    //initalize serial monitor at 9600 baud
    delay(1000);
    Serial.begin(9600);
    pitch.attach(PITCH_PIN);
    pitch.write(0);

}

void loop() {
   /* int pitchVal = 0;

    for( ; pitchVal < 180; pitchVal += 15){
        pitch.write(pitchVal);
    }

    for( ; pitchVal >= 0; pitchVal -= 15){
        pitch.write(pitchVal);
    }*/
  pitch.write(90);
  delay(1000);
  Serial.println("HELLO");
}
