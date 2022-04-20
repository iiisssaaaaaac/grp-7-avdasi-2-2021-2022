
#include <SPI.h>
#include <RH_RF69.h>
#include <LiquidCrystal.h>

// Variables that remain constant
#define RF69_FREQ 433.0
#define RFM69_CS 10
#define RFM69_INT digitalPinToInterrupt(2)
#define RFM69_RST 3

LiquidCrystal lcd(23,22,9,8,7,6);

// Instances a packet radio object from the library
RH_RF69 rf69(RFM69_CS, RFM69_INT);

const byte potpin0 = A0; // Analog input pin from potentiometer, flap
const byte potpin1 = A1; // aileron
const byte potpin2 = A2; // kp
const byte potpin3 = A3; // ki
const byte potpin4 = A4; // kd

double Fa;
byte TFa;

int outputSetpoint, outputAileron, outputKp, outputKi, outputKd;
double Kpmax=3, Kimax=1, Kdmax=1;

struct transmission
{
  byte Aileron_angle; // Values 10 - 170 (for this particular servo)
  byte Tsetpoint; //
  byte TKp;
  byte TKi;
  byte TKd;
} nodeTX;

void setup()
{
  // Initialise Feather M0 reset pin
  pinMode(RFM69_RST, OUTPUT);
  // Reset the Feather M0's radio IC
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Only needed for debugging with computer USB connection
  Serial.begin(115200);
  // Only needed for debugging with computer USB connection
  if (!rf69.init())
  {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  // Only needed for debugging with computer USB connection
  if (!rf69.setFrequency(RF69_FREQ))
  {
    Serial.println("RFM69 radio setFrequency failed");
  }

  // The radio's power in dBm; valid values are -2 to +20; always
  // set to the lowest power level needed
  rf69.setTxPower(20, true);
  //Define an encryption key; it must be the same for TX and RX
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  // Use the encryption key defined above
  rf69.setEncryptionKey(key);

  // Initialise potentiometer pin with an internal pull-up resistor
  pinMode (potpin0, INPUT_PULLUP);
  pinMode (potpin1, INPUT_PULLUP);
  pinMode (potpin2, INPUT_PULLUP);
  pinMode (potpin3, INPUT_PULLUP);
  pinMode (potpin4, INPUT_PULLUP);

// initialise LCD
  lcd.begin(16, 2);
  lcd.clear();

}

void loop()
{
  delay(20);

  // A call to this function fetches a reading from the potentiometer pin
  readPotentiometer();

  Serial.print("Requested flap angle: "); Serial.println(nodeTX.Tsetpoint);
  Serial.print("Requested Aileron angle: "); Serial.println(nodeTX.Aileron_angle);
  Serial.print("Kp, Ki, Kd : "); Serial.print(nodeTX.TKp); Serial.print(" ,");
  Serial.print(nodeTX.TKi); Serial.print(", ");
  Serial.println(nodeTX.TKd);

  rf69.send((uint8_t *)&nodeTX, sizeof(nodeTX));

  // Use the packet radio object's method that waits until the data
  // was successfully transmitted to the receiver (RX)
  rf69.waitPacketSent();

  // Use the packet radio object's method that checks if some data
  // transmission has arrived from TX
  if (rf69.available())
  {

    // Seek the struct's space taken up in memory
    uint8_t len = sizeof(TFa);
    if (rf69.recv((uint8_t *)&TFa, &len))
    {
      Fa=TFa;
    }
  }
  Serial.println(Fa);
  //----------------------LCD SHIT-------------------------
// first transform read values into meaningfuck ones
outputSetpoint = map(nodeTX.Tsetpoint, 100, 0, 45, 0);
outputAileron = map(nodeTX.Aileron_angle, 100, 180, -30, 30);
outputKp = nodeTX.TKp;
outputKi = nodeTX.TKi;
outputKd = nodeTX.TKd;
/*
  lcd.setCursor(0,0); // Sets the cursor to col 0 and row 0
  lcd.print("Fr:");
  lcd.print(outputSetpoint);

  lcd.setCursor(10,0);
  lcd.print("Ai:");
  lcd.print(outputAileron);

  lcd.setCursor(0,1); // Sets the cursor to col 0 and row 1
  lcd.print("Kp:"); // Prints Sensor Val: to LCD
  lcd.print(outputKp);

  lcd.setCursor(5,1);
  lcd.print("Ki:"); // Prints Sensor Val: to LCD
  lcd.print(outputKi);

  lcd.setCursor(10,1);
  lcd.print("Kd:"); // Prints Sensor Val: to LCD
  lcd.print(outputKd);
*/

lcd.setCursor(0,0); // Sets the cursor to col 0 and row 0
lcd.print("Fr:");
lcd.print(outputSetpoint);

lcd.setCursor(5,0);
lcd.print("Fa:");
lcd.print(Fa);

lcd.setCursor(10,0);
lcd.print("Ai:");
lcd.print(outputAileron);

lcd.setCursor(0,1);
lcd.print("K:");

lcd.setCursor(2,1); // Sets the cursor to col 0 and row 1
lcd.print("p:"); // Prints Sensor Val: to LCD
lcd.print(outputKp);

lcd.setCursor(7,1);
lcd.print("i:"); // Prints Sensor Val: to LCD
lcd.print(outputKi);

lcd.setCursor(12,1);
lcd.print("d:"); // Prints Sensor Val: to LCD
lcd.print(outputKd);

}


void readPotentiometer()
{
  // Read the voltage from the potentiometer pin and map the output
  // to some simple values cos that's easiest. means they can be stored as bytes,
  // which is smaller to transmit 👍👍👌
  nodeTX.Tsetpoint = map(analogRead(potpin0), 0, 1023, 100, 0);
  nodeTX.Aileron_angle = map(analogRead(potpin1), 30, 1023, 180, 100);
  nodeTX.TKp = map(analogRead(potpin2), 0, 1023, 100, 0);
  nodeTX.TKi = map(analogRead(potpin3), 0, 1023, 100, 0);
  nodeTX.TKd = map(analogRead(potpin4), 0, 1023, 0, 100);
}
