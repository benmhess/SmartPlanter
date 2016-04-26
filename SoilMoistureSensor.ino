//LIBRARY FOR HUMD/TEMP SENSOR
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

class HTU21D {

public:
  HTU21D();

  //Public Functions
  void begin();
  float readHumidity(void);
  float readTemperature(void);
  void setResolution(byte resBits);

  //Public Variables

private:
  //Private Functions

  byte read_user_register(void);
  byte check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

  //Private Variables

};
#include <Wire.h>

HTU21D::HTU21D()
{
  //Set initial values for private vars
}

//Begin
/*******************************************************************************************/
//Start I2C communication
void HTU21D::begin(void)
{
  Wire.begin();
}

//Read the humidity
/*******************************************************************************************/
//Calc humidity and return it to the user
//Returns 998 if I2C timed out 
//Returns 999 if CRC is wrong
float HTU21D::readHumidity(void)
{
  //Request a humidity reading
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_HUMD_MEASURE_NOHOLD); //Measure humidity with no bus holding
  Wire.endTransmission();

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
  delay(55);

  //Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  int counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) return 998; //Error out
  }

  byte msb, lsb, checksum;
  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();
  
  unsigned int rawHumidity = ((unsigned int) msb << 8) | (unsigned int) lsb;

  if(check_crc(rawHumidity, checksum) != 0) return(999); //Error out

  //sensorStatus = rawHumidity & 0x0003; //Grab only the right two bits
  rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place
  
  //Given the raw humidity data, calculate the actual relative humidity
  float tempRH = rawHumidity / (float)65536; //2^16 = 65536
  float rh = -6 + (125 * tempRH); //From page 14
  
  return(rh);
}

//Read the temperature
/*******************************************************************************************/
//Calc temperature and return it to the user
//Returns 998 if I2C timed out 
//Returns 999 if CRC is wrong
float HTU21D::readTemperature(void)
{
  //Request the temperature
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);
  Wire.endTransmission();

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
  delay(55);

  //Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  int counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) return 998; //Error out
  }

  unsigned char msb, lsb, checksum;
  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();
  unsigned int rawTemperature = ((unsigned int) msb << 8) | (unsigned int) lsb;

  if(check_crc(rawTemperature, checksum) != 0) return(999); //Error out

  //sensorStatus = rawTemperature & 0x0003; //Grab only the right two bits
  rawTemperature &= 0xFFFC; //Zero out the status bits but keep them in place

  //Given the raw temperature data, calculate the actual temperature
  float tempTemperature = rawTemperature / (float)65536; //2^16 = 65536
  float realTemperature = (float)(-46.85 + (175.72 * tempTemperature)); //From page 14

  return(realTemperature);  
}

void HTU21D::setResolution(byte resolution)
{
  byte userRegister = read_user_register(); //Go get the current register state
  userRegister &= B01111110; //Turn off the resolution bits
  resolution &= B10000001; //Turn off all other bits but resolution bits
  userRegister |= resolution; //Mask in the requested resolution bits
  
  //Request a write to user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(WRITE_USER_REG); //Write to the user register
  Wire.write(userRegister); //Write the new resolution bits
  Wire.endTransmission();
}

//Read the user register
byte HTU21D::read_user_register(void)
{
  byte userRegister;
  
  //Request the user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(READ_USER_REG); //Read the user register
  Wire.endTransmission();
  
  //Read result
  Wire.requestFrom(HTDU21D_ADDRESS, 1);
  
  userRegister = Wire.read();

  return(userRegister);  
}

#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

byte HTU21D::check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, checkvalue is 0x79
  //message = 0x683A, checkvalue is 0x7C
  //message = 0x4E85, checkvalue is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {

    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return (byte)remainder;
}
//CODE BEGINS HERE
// Sketch reads sensor and desplays level and whether soil is wet or dry
// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

#define PIN_SCK  13
#define PIN_MISO 12
#define  PIN_MOSI 11
#define PIN_SS   10

int myTxPin = 2;  // Chosen pin for transmitting data
int myRxPin = 3;  // Chosen pin for receiving data
int servoPin = 9;
Servo servo;
SoftwareSerial softSerial(myRxPin, myTxPin);
int angle = 0;
HTU21D myHumidity;
// Attach the serial display's RX line to digital pin 2
SoftwareSerial mySerial(3,2); // pin 2 = TX, pin 3 = RX (unused)
// Here we are setting up some water thersholds that we will use later 
int thresholdUp = 750;
int thresholdDown = 250;
// We are setting up the pin A0 on the redboard to be our sensor input
int sensorPin = A0;
int lightPin = A1;

// Send a single byte via SPI
void sendChar(char cData){
  SPDR = cData;
  while(!(SPSR&(1<<SPIF)));
}

// Send a full frame to the LED matrix
void sendFrame(char *frame) {
  // Assert SS
  digitalWrite(PIN_SS, LOW);
  // delay as the LED Matrix datasheet's recommends
  delayMicroseconds(500);
  // send the full buffer
  for(int i=0;i<64;i++) {
    char c;
    c=*(frame+i);
    if('%'==c)
      sendChar((2<<5)&(1<<2)&1); // similar color
    else
      sendChar(c);
  }
  // de-assert SS
  digitalWrite(PIN_SS, HIGH);
}


void setup(){
  softSerial.begin(9600);  // Start the software serial at 9600
  pinMode(lightPin, OUTPUT);
  myHumidity.begin();
  servo.attach(servoPin);
  Serial.begin(9600); // set up serial port for 9600 baud (speed)
  delay(500); // wait for display to boot up

 SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
  // Status register
  SPSR = SPSR & B11111110;

  // setup pins
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW); // ensure clock low
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH); // de-assert SS
  delayMicroseconds(500); // delay as the LED Matrix datasheet's recommends

  // This section reconfigure the board to no daisy chain
  // operation.
  // This can be confirmed by an RRGGBB pattern beginning at a
  // corner, with no black LEDs before the first LED, when the
  // matrix is powered on.
  // Warning: this will take effect only after first run, power
  // off and power on
  digitalWrite(PIN_SS, LOW);
  delayMicroseconds(500);
  sendChar('%');
  sendChar(1);
  digitalWrite(PIN_SS, HIGH);
  delayMicroseconds(10);
  servo.write(0);
}

void loop(){

  char frameBuffer[64];
  
  String DisplayWords; //print out for testing reasons only
  
  // We need to set up a pin to get the value that the soil 
  // moisture sensor is outputting, so sensorValue will get the
  // analog value from the sensor pin A0 
  int sensorValue;
  sensorValue = analogRead(sensorPin);
  int lightValue=analogRead(lightPin);

  Serial.println(sensorValue); //Using .print instead of .write for values
  Serial.println(lightValue);
  // Now we are going to check if the water level is below a 
  // out thresholdDown value we set earlier, and if it is have 
  // words "Dry, Water it!" display one column over on the first 
  // row:
  
  if (sensorValue <= thresholdDown){

    //LED code - red zone

    for(int color=0;color<=255;color++) {
    // And populate each position of the buffer with one color
      for(int j=0;j<64;j++) {
        frameBuffer[j]=160;
      }
      // Send the frame to the LED Matrix
    sendFrame(frameBuffer);
    // Colors are made by blinking the LEDs very fast.
    // Decreasing this delay too much (or removing it) may lead
    // to corrupted images.
    delay(200);
    break;
    }
      
    
//    DisplayWords = "Dry, Water it!";
//    Serial.println(DisplayWords);
    float humd = myHumidity.readHumidity();
    float temp = myHumidity.readTemperature();
    //CODE TO STORE CURRENT lightValue, sensorValue, & humd+temp
    //connects to beagle bone and is saved to a database
    softSerial.print("temp");
    String myString = String(temp);
    softSerial.print(myString+'\n');
    softSerial.print("humd");
    String myString2 = String(humd);
    softSerial.print(myString2+'\n');

    softSerial.print("light");
    String myString3 = String(lightValue);
    Serial.println(myString3);
    Serial.println(lightValue);
    softSerial.print(myString3+'\n');
    softSerial.print("water");
    String myString4 = String(sensorValue);
    Serial.println(myString4);
    Serial.println(sensorValue);
    softSerial.print(myString4+'\n');
    //print outs for serial monitor testing
//    Serial.println(lightValue);
//    Serial.println(temp);
//    Serial.println(humd);
    servo.write(150);//value to rotate to so the metal ball will adjust
    while(sensorValue<=thresholdUp){
      sensorValue=analogRead(sensorPin);
      }
    servo.write(0);//return when above the threshold change
    Serial.println("Done watering!");
  // If the value is not below our thresholdDown value we want to 
  // check if it is above our thresholdUp value



  } else if (sensorValue >= thresholdUp){


    for(int color=0;color<=255;color++) {
    // And populate each position of the buffer with one color
      for(int j=0;j<64;j++) {
        frameBuffer[j]=120;
      }
      // Send the frame to the LED Matrix
    sendFrame(frameBuffer);
    // Colors are made by blinking the LEDs very fast.
    // Decreasing this delay too much (or removing it) may lead
    // to corrupted images.
    delay(200);
    break;
    }
   
    DisplayWords = "Wet, Leave it!";
    Serial.println(DisplayWords);
    //this is where code for LED goes - green zone
    //nothing else should be done as this is the desired state

  // Otherwise if it is inbetween the two values we want it to display the yellow warning light

  } else {


    Serial.println(DisplayWords);


    for(int color=0;color<=255;color++) {
    // And populate each position of the buffer with one color
      for(int j=0;j<64;j++) {
        frameBuffer[j]=140;
      }
      // Send the frame to the LED Matrix
    sendFrame(frameBuffer);
    // Colors are made by blinking the LEDs very fast.
    // Decreasing this delay too much (or removing it) may lead
    // to corrupted images.
    delay(200);
    break;
    }
    
    //LED code - yellow zone
    //just the warning phase
        Serial.println("almost watering time!");

  }

  delay(500);
}

