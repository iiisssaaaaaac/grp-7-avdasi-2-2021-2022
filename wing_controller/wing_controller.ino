/*Grp 7 AVDASI 2 wing mounted controller
  Controlling an aileron through a servo and a flap through a DC motor and potentiometer.
  Requested flap position and real flap position will be logged onto an onboard SD card.
  Current flap position will be sent to grnd station Requested flap and aileron positions
  will be sent wirelessly from a ground station, so will Kp, Ki, Kd, PID parameters.

  variable names for ground controller:
  Aileron angle potentiometer - Aileron_angle
  Flap angle potentiometer - setpoint
  Kp - Kp
  Ki - Ki
  Kd - Kd


  it is possible that the response will be the wrong way around, so need to change some +ves to -ves.
  be ready to unplug things fast!!



  PINS n SHIT
  PWM to motor controller - 4,5,6,7
  Flap potentiometer - A1
*/


#include <SPI.h>
#include <PID_v1.h>
#include <SD.h>
#include <RH_RF69.h>

//--------------------------------------------- RADIO SHIT ------------------------------------

// Variables that remain constant
#define RF69_FREQ 433.0
#define RFM69_CS 10
#define RFM69_INT digitalPinToInterrupt(2)
#define RFM69_RST 3

// Instances a packet radio object from the library
RH_RF69 radio(RFM69_CS, RFM69_INT);

/* Some transmitted variables are Txxxxxxxxx because it is better to send variables as bytes,
  then convert them to the required format here*/

struct transmission
{
  byte Aileron_angle; // Values 10 - 170 (for this particular servo)
  byte Tsetpoint; //
  byte TKp;
  byte TKi;
  byte TKd;
} nodeTX;


byte TFa; // flap angle for transmission

// some variables for rate of transmission back to ground controller (5Hz @ 200ms)
const long returnrate = 200;
unsigned long previousMillis = 0;
unsigned long PreviousTimefifty = 0;
//------------------------------ NOT RADIO SHIT ----------------------------

//#define Fpotpin A1
//#define servopin 33

const byte Fpotpin = A1;
const byte servopin = 33;
unsigned long runTime; // timestamp

const int a1 = 4;
const int a2 = 5;
const int b1 = 6;
const int b2 = 7;
const int chipSelect = BUILTIN_SDCARD;

double Fa, PIDout;
//double setpoint, Fa, PIDout; // Desired angle, Actual flap angle, PID output

double setpoint;
double Kp = 1, Ki = 0.05, Kd = 0.0;

double Kpmax = 8, Kimax = 3, Kdmax = 1;

// HOW FAR ON EACH SIDE IS ALLOWED
double Tolerance = 2;

PID flap(&Fa, &PIDout, &setpoint, Kp, Ki, Kd, DIRECT);  //PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction, +- tolerance)

void setup()
{
  // Initialise Feather M0 reset pin
  pinMode(RFM69_RST, OUTPUT);
  // Reset the Feather M0's radio IC
  digitalWrite(RFM69_RST, LOW);

  // Only needed for debugging with computer USB connection
  Serial.begin(115200);
  if (!radio.init())
  {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  // Only needed for debugging with computer USB connection
  if (!radio.setFrequency(RF69_FREQ))
  {
    Serial.println("RFM69 radio setFrequency failed");
  }

  // The radio's power in dBm; valid values are -2 to +20; always
  // set to the lowest power level needed
  radio.setTxPower(20, true);
  //Define an encryption key; it must be the same for TX and RX
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  // Use the encryption key defined above
  radio.setEncryptionKey(key);

  // 20mS between samples = 50Hz
  flap.SetSampleTime(20);

  // PWM output limits. 60% duty cycle maximum (~3V)
  flap.SetOutputLimits(-200, 200);

  //turn on PID
  flap.SetMode(AUTOMATIC);

  // set some safe initial value
  setpoint = 700;

  // set the motor controller digital pins to output mode
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(servopin, OUTPUT);
//-----------------------------------SD CARD SHIT---------------------------------
  // begin serial output for debugging and shit
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  File dataFile = SD.open("flaplog.txt", FILE_WRITE);
  String dataFileHeader = "Elapsed time (ms),Measured flap angle" //......
  ",Requested flap angle,Aileron angle,Kp,Ki,Kd";

  if (dataFile) {
    dataFile.println(dataFileHeader);
    dataFile.close();
  }
  else {
    Serial.println("error doing that shit man");
  }
}
void loop()
{
  // set the clock ticking
  runTime = millis();

  // Use the packet radio object's method that checks if some data
  // transmission has arrived from TX
  if (radio.available())
  {
    //    Serial.println("WE GOT MAIL BABY \t \t");
    // Seek the struct's space taken up in memory
    uint8_t len = sizeof(nodeTX);
    if (radio.recv((uint8_t *)&nodeTX, &len))
    {
      // assign the doubles their byte transmitted partner
      setpoint = map(nodeTX.Tsetpoint, 0, 100, 700, 460);
      Kp = Kpmax * 0.001 * nodeTX.TKp;
      Ki = Kimax * 0.001 * nodeTX.TKi;
      Kd = Kdmax * 0.001 * nodeTX.TKd;

      // A call to this function rotates the servo based on the angle
      // value that was transmitted and received; it puts into the
      // struct variable nodeTX.Aileron_angle
      rotateServo(servopin, nodeTX.Aileron_angle);
    }
  }

  // Set the new gains (from wireless transmission)
  flap.SetTunings(Kp, Ki, Kd);

  // reads the value of the Aileron_angle (value between 0 and 1023)
  Fa = analogRead(Fpotpin);
//Fa=600;
  //  Fa = map(Fa, 0, 1023, 0, 40);

  // compute the PID response to the current system state
  flap.Compute();

  if (abs(Fa - setpoint) <= Tolerance) PIDout = 0; // set the response when within angle tolerance
  //-------------------------------- FLAP MOTOR MOVEMENT -----------------------------------------
  // sets range for accepted error ( +-2) and then applies the PID output to the motor controller
  if (Fa > setpoint + 2)
  {
    analogWrite(a1, -PIDout);
    analogWrite(a2, 0);
    analogWrite(b1, -PIDout);
    analogWrite(b2, 0);
  }
  else if (Fa < setpoint - 2)
  {
    analogWrite(a1, 0);
    analogWrite(a2, PIDout);
    analogWrite(b1, 0);
    analogWrite(b2, PIDout);
  }
  else
  {
    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
  }


  //---------------------------50Hz loop-------------------------------

  if (runTime - PreviousTimefifty >= int(20))
  {
    PreviousTimefifty = runTime;
    printserial();
    writetoSD();
  }


  //______________________________ SEND SHIT BACK TO GROUND ________________________________
  TFa = map(Fa, 700, 460, 0, 100);
  if (runTime - previousMillis >= returnrate)
  {
    previousMillis = runTime;
    radio.send((uint8_t *)&TFa, sizeof(TFa));

    // Use the packet radio object's method that waits until the data
    // was successfully transmitted to the receiver (RX)
    radio.waitPacketSent();
  }

// delay(1);

}


// my own functions
void rotateServo(byte pin, byte angle)
{
  // The Arduino Servo.h library is incompatible with the RadioHead
  // library, as they both use the same Feather M0 internal timer.
  // Instead, the servo is simply driven programmatically, without a
  // library, useful for other timer conflict projects, too. First,
  // the angle value received from TX is mapped onto a microseconds
  // range, where for this particular servo, 640 equals 10° and 2120
  // equals 170°. These values are different for every servo, so one
  // should check by trial before
  int pulseDelay = map(angle, 10, 170, 640, 2120);
  // Now enable the servo
  digitalWrite(pin, HIGH);
  // Wait
  delayMicroseconds(pulseDelay);
  // Then disable the servo
  digitalWrite(pin, LOW);
  // Finally wait again (50Hz PWM = pulse period of 20ms)
  delay(20);
}

void printserial() {
  Serial.print("runTime: "); Serial.println(String(runTime));
  Serial.print("Measured flap angle: "); Serial.println(Fa);
  Serial.print("Requested flap angle: "); Serial.println(setpoint);
  Serial.print("Aileron angle: "); Serial.println(nodeTX.Aileron_angle);
  Serial.print("PID PWM value: "); Serial.println(PIDout);
  Serial.print("Kp, Ki, Kd : "); Serial.print(Kp); Serial.print(" ,");
  Serial.print(Ki); Serial.print(", ");
  Serial.println(Kd);
}

void writetoSD() {


  String dataString = ""; // this is the string that the line of data will be written to
  dataString += String(runTime) + ",";
  dataString += String(Fa) + ",";
  dataString += String(setpoint) + ",";
  dataString += String(nodeTX.Aileron_angle) + ",";
  dataString += String(Kp) + ",";
  dataString += String(Ki) + ",";
  dataString += String(Kd);

  // Some shit for writing to SD card
  File dataFile = SD.open("flaplog.txt", FILE_WRITE);

  if (dataFile) {
    /*
    dataFile.print("Timestamp (ms):\t");
    dataFile.print(runTime); dataFile.println(",\t");
    dataFile.print("True Flap Angle (deg):\t");
    dataFile.print(Fa); dataFile.print(",\t");
    dataFile.print("Requested Flap Angle (deg):\t");
    dataFile.print(setpoint); dataFile.print(",\t");
    dataFile.print("Kp :\t");
    dataFile.print(Kp); dataFile.print(",\t");
    dataFile.print("Ki :\t");
    dataFile.print(Ki); dataFile.print(",\t");
    dataFile.print("Kd :\t");
    dataFile.print(Kd); dataFile.println(",\t");
    */
    dataFile.println(dataString);
    dataFile.close();
  }
  else {
    Serial.println("error opening flaplog.txt");
  }
}
