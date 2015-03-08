#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <SPI.h>

LiquidCrystal lcd(15, 14, 5, 4, 3, 2);
SoftwareSerial RFID(8, 9); // RX and TX
int CS=10;
int data1 = 0;
int ok = -1;
int yes = 16;
int no = 7;

char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

// rfid code based on http://www.instructables.com/id/Arduino-Tutorials-RFID/
//2 52 55 48 48 56 51 52 48 67 54 52 50 3 2 52 55 48 48 56 51 52 48 67 54 52 50 3 
int tag1[14] = {2,52,55,48,48,56,51,52,48,67,54,52,50,3};
int tag2[14] = {2,52,48,48,48,56,54,67,54,54,66,54,66,3};
int newtag[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // used for read comparisons

void setup()


{
  
   //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  
RFID.begin(9600); // start serial to RFID reader
Serial.begin(9600); // start serial to PC 
pinMode(yes, OUTPUT); // for status LEDs
pinMode(no, OUTPUT);
  lcd.begin(16, 2);
  
  
}
boolean comparetag(int aa[14], int bb[14])
{
boolean ff = false;
int fg = 0;
for (int cc = 0 ; cc < 14 ; cc++)
{
  if (aa[cc] == bb[cc])
{
fg++;
}
}
if (fg == 14)
{
ff = true;
}
return ff;
}
void checkmytags() // compares each tag against the tag just read
{
ok = 0; // this variable helps decision-making,
// if it is 1 we have a match, zero is a read but no match,
// -1 is no read attempt made
if (comparetag(newtag, tag1) == true)
{
ok++;
}
if (comparetag(newtag, tag2) == true)
{
ok++;
}
}
void readTags()
{
ok = -1;
if (RFID.available() > 0) 
{
// read tag numbers
delay(100); // needed to allow time for the data to come in from the serial buffer.
for (int z = 0 ; z < 14 ; z++) // read the rest of the tag
{
data1 = RFID.read();
newtag[z] = data1;
}
RFID.flush(); // stops multiple reads
// do the tags match up?
checkmytags();
}
// now do something based on tag type
if (ok > 0) // if we had a match
{
lcd.print("accepted"); 
delay(500);
lcd.noDisplay();
delay(500);
lcd.display();
delay(500);
lcd.clear();
Serial.println("Accepted");
digitalWrite(yes, HIGH);
delay(1000);
digitalWrite(yes, LOW);
ok = -1;
}
else if (ok == 0) // if we didn't have a match
{
lcd.print("rejected"); 
delay(500);
lcd.noDisplay();
delay(500);
lcd.display();
delay(500);
lcd.clear();
Serial.println("Rejected");
digitalWrite(no, HIGH);
delay(1000);
digitalWrite(no, LOW);
ok = -1;
}
}
void loop()



  {
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
  
  //map the raw output to a reange usable for analogWriten
  x=map(x,-150,150,0,255);
  y=map(y,-150,150,0,255);
  z=map(z,-150,150,0,255);
    
  //USE PWM to light up the different color of an RGB LED
  analogWrite(3,x);
  analogWrite(5,y);
  analogWrite(6,z);
   
  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);      
  delay(10); 
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);

readTags();

}
