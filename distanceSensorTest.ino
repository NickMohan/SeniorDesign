#include <SharpIR.h>

#define IRPin A1
#define model 1080

int distance_cm;

SharpIR mySensor = SharpIR(IRPin, model);

void setup() {
  Serial.begin(9600);
}

void loop() {
  distance_cm = mySensor.getDistance();

  Serial.print("Distance: ");
  Serial.println(distance_cm);
  delay(1000);
}
