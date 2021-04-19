#include <Servo.h>
#include <Wire.h>

//Digital pin numbers for pitch and roll
#define CLAW_PIN 5

//Initalize bno and servo motors
Servo claw;

void setup() {
    //initalize serial monitor at 9600 baud
    delay(1000);
    Serial.begin(9600);
    claw.attach(CLAW_PIN);
    claw.write(0);

}

void loop() {
    int clawVal = 0;

    for( ; clawVal < 180; clawVal += 15){
        claw.write(clawVal);
    }

    for( ; clawVal > 0; clawVal -= 15){
        claw.write(clawVal);
    }
  delay(1000);
}
