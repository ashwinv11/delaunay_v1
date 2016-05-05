#include <SharpIR.h>

const int numReadings = 25;

int readings_pot1[numReadings];
int readings_pot2[numReadings];

int readIndex = 0;
int total_pot1 = 0;
int total_pot2 = 0;
int average_pot1 = 0;
int average_pot2 = 0;

int precision = 93;

#define pot1 A0 // left pot
#define pot2 A1 // right pot
#define ir1 A2 // top right
#define ir2 A3 // top left
#define ir3 A4 // bot left
#define ir4 A5 // bot right
#define model 20150

SharpIR sharp1(ir1, numReadings, precision, model);
SharpIR sharp2(ir2, numReadings, precision, model);
SharpIR sharp3(ir3, numReadings, precision, model);
SharpIR sharp4(ir4, numReadings, precision, model);

int mean1 = 0;
int mean2 = 0;
int mean3 = 0;
int mean4 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings_pot1[thisReading] = 0;
    readings_pot2[thisReading] = 0;
  }
  
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
}

void loop() {   

  int pot1_read = analogRead(pot1);
  int pot2_read = analogRead(pot2);
  
  int pot1_map = map(pot1_read, 0, 1023, 0, 5);
  int pot2_map = map(pot2_read, 0, 1023, 0, 5);
  int sensor1_map = map(mean1, 0, 1023, 0, 5);
  int sensor2_map = map(mean2, 0, 1023, 0, 5);
  int sensor3_map = map(mean3, 0, 1023, 0, 5);
  int sensor4_map = map(mean4, 0, 1023, 0, 5);

  //smooth();
  
  serialPrint(pot1_read, pot2_read, mean1, mean2, mean3, mean4);
  //serialPrint(pot1_map, pot2_map, sensor1_map, sensor2_map, sensor3_map, sensor4_map);
  //checkVoltage();
  delay(1000);
  calculateMeanDist();
}

void serialPrint(int a, int b, int c, int d, int e, int f){
  Serial.print("[");
  Serial.print(a);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.print(c);
  Serial.print(",");
  Serial.print(d);
  Serial.print(",");
  Serial.print(e);
  Serial.print(",");
  Serial.print(f);
  Serial.print("]");
  Serial.println();
}

void calculateMeanDist(){
  
  unsigned long pepe1=millis();  // takes the time before the loop on the library begins

  mean1 = sharp1.distance();  // this returns the distance to the object you're measuring
  mean2 = sharp2.distance();
  mean3 = sharp3.distance();
  mean4 = sharp4.distance();
  
}

void smooth(){
  
  // subtract the last reading:
  total_pot1 = total_pot1 - readings_pot1[readIndex];
  total_pot2 = total_pot2 - readings_pot2[readIndex];
  // read from the sensor:
  readings_pot1[readIndex] = analogRead(pot1);
  readings_pot2[readIndex] = analogRead(pot2);
  // add the reading to the total:
  total_pot1 = total_pot1 + readings_pot1[readIndex];
  total_pot2 = total_pot2 + readings_pot1[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average_pot1 = total_pot1 / numReadings;
  average_pot2 = total_pot2 / numReadings;

}

