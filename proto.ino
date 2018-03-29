#include <Adafruit_BME280.h>
#include <Adafruit_MMA8451.h>
#include <SD.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const int chipSelect = 10;

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BME280 bme;


// VARS
const int channelA = 3;
const int channelB = 4;
int encoderTicks = 0;                // ticks for encoder
bool hasLaunched = false;
bool hasBurnedOut = false;
bool flightDone = false;

float accelZ;                               //z acceleration
float altCurrent = 0;                       //current altitude
float altInitial = 0;                       //initial calculated altitude
float altPrevious = 0;                      //previous altitude
float startTimer;                           //start of rocket launch
float projectedAltitude = 0;
float loopCtr = 0;
float timeDiff = 0;
float velTimer1 = 0;                        //initial velocity timer
float velTimer2 = 0;                        //final velocity timer
float velZ = 0;                             //z velocity

File dataFile;

void setup() {
  // Initialize the sensor 
  Serial.begin(9600);

  // initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card fail");
  }
  dataFile = SD.open("launch.txt", FILE_WRITE);
  if (dataFile) {
    Serial.print("Writing to launch.txt...");
    dataFile.println("testing 1, 2, 3.");
    dataFile.println("-------------- LAUNCH DATA --------------");
    dataFile.println();
    // close the file:
    dataFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening launch.txt");
  }
  
  // test pressure sensor 
  bool status;
  status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    
  // test acceleration sensor
  if (! mma.begin()) {
    Serial.println("Couldnt start mma !!!! Wiring check.");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);

  // set initial Alt... averaged from 20 readings
  for (int i = 0; i < 20; i++) {
    altInitial += calcAlt();
  }
  altInitial /= 20;
}

// main execution
void loop() {
  loopCtr++;
 
  // PHASE 1 (PRE-LAUNCH)
  while (!hasLaunched) {
    accelZ = getAccelZ();
//    Serial.println(accelZ);
    //if accel is pos, we have launched
    if (accelZ > 0) {
      hasLaunched = true;
      // start timer
      startTimer = millis();
      Serial.println("Phase ONE HAS PASSED");  
    }
  }
  
  // PHASE 2 (Motor-Burn)
  while (!hasBurnedOut) {
    accelZ = getAccelZ();
    Serial.println(accelZ);
    //once accel is neg, the motor has burned out
    if (accelZ < 0) {
      hasBurnedOut = true;
      Serial.println("PHASE TWO HAS PASSED");
        
    }
  }

  // PHASE 3 (Post-Burn Action)
  altPrevious = altCurrent;
  altCurrent = calcAlt() - altInitial;

  //calculate velocity
  velTimer2 = millis();
  timeDiff = (velTimer2 - velTimer1) / 1000;
  velZ = calcVelZ(velTimer1, velTimer2, altCurrent, altPrevious);
  velTimer1 = millis();
  
  
  // PHASE 4 (Post-Action Regress)
  // detect significant drop in altitude and then return system home and shut off controller
  if (velZ < 0) {
    // return home and shut down
  }

  printData(loopCtr, 0.0, altCurrent, velZ, getAccelZ(), timeDiff);
    
  delay(500);

}

// returns current altitude
float calcAlt() {
  
  float alt;
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.280840;

  return alt;
}

// calculates velocity for the Z-axis
float calcVelZ(float timer1, float timer2, float altCurrent, float altPrevious) {
  float velocity;
  float timerDiff;

  //loop timer
  timerDiff = (velTimer2 - velTimer1) / 1000;
  
  //calc vel
  velocity = (altCurrent - altPrevious) / timerDiff;

  return velocity;
}

// calculates acceleration for the z-axis 
float getAccelZ () {

  float zAccel;
  
  mma.read();
  
  sensors_event_t event; 
  mma.getEvent(&event);

  zAccel = event.acceleration.z;
  //calc to ft/s
  zAccel *= -3.280840;

  return zAccel;
  
}

// prints out flight data to micro sd card breakout
void printData(int loopCtrIn, float posIn, float altIn, float velIn, float accelIn, float timeDiffIn) {
  File df = SD.open("launch.txt", FILE_WRITE);
  if (df) {
    Serial.println("Works");
  }
  else {
    Serial.println("Nope");
  }
  df.print("--------------   LOOP ");
  df.print(loopCtrIn);
  df.println("    --------------");
  df.print("plate position: ");
  df.println(posIn);
  df.print("current altitude: ");
  df.println(altIn);
  df.print("velocity: ");
  df.println(velIn);
  df.print("acceleration: ");
  df.println(accelIn);
  df.print("projected apogee: ");
  df.println(projectedAltitude);
  df.print("time difference: ");
  df.println(timeDiffIn);
  df.close();
}
