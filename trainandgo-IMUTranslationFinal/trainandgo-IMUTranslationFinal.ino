/*
 * This file translates ISM330DHCX IMU data into 
 * bluetooth controller axes using an ESP32 microcontroller.
 *
 * It combines an example from each library with mathematical 
 * modifications necessary to translate the orientation data.
 */
#include <BleGamepad.h>
#include <Adafruit_ISM330DHCX.h>

#define bleAxisMin 0
#define bleAxisMax 32767

#define imuDPSMin -10.01
#define imuDPSMax 10.01

//BLEGamepad variables
BleGamepad bleGamepad("Train and Go", "Train and Go", 100);

//IMU variables
Adafruit_ISM330DHCX ism330dhcx;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
void calibrateIMU(); //Fucntion declaration for calibration

//Calculation variables
float gyroX, gyroY, gyroZ, gyroRange;
float currentAngleRad;
int bleX, bleY;
float calibX, calibY, calibZ;
unsigned long prevTime, currentTime;

//Reset Function
void(* resetFunc) (void) = 0;

void setup()
{
    //Serial Monitor setup for testing
    //Serial.begin(115200);
    //BLE Gamepad Setup
    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setAutoReport(false); //Have to manually sendReport, allows us to change things a few times before sending any update
    bleGamepadConfig.setButtonCount(16);
    bleGamepadConfig.setHatSwitchCount(2);
    bleGamepad.begin(&bleGamepadConfig);

    //IMU Input Setup
    if(!ism330dhcx.begin_I2C()) //Check if an IMU is connected, if not, don't try to move on
    {
      while(1)
      {
        delay(1000);
        resetFunc();
      }
    }
    //Set up the ISM -- see IMUTest for options
    ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    ism330dhcx.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    ism330dhcx.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);

    //Configure interrupts to know when data is ready
    ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
    ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2

    //Initialize Gyro Range
    gyroRange = imuDPSMax - imuDPSMin;
    
    //Calibrates IMU *and* initializes IMU variables
    calibrateIMU();

    bleGamepad.setAxes(16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384);
    bleGamepad.sendReport();
    pinMode(34, INPUT);
}

/*
 * Initialize:
 * float gyroX, gyroY, gyroZ;
 * int bleX, bleY;
 * float calibX, calibY, calibZ;
 * float currentAngleRad;
 * 
 * calibX and calibY depending on gravity calculations
 * 
 * The force of gravity can be calculated via the 
 * pythagorean theorem in the third dimension.
 * Each side length contributes its square to the overall gravity sum
 * This square sum can be square rooted to approximate the gravitational measurement.
 * If the gravity is felt entirely on the z axis, only z axis rotation should be considered.
 * Thus, the percent that each axis grants to gravity should dictate its consideration.
 */
void calibrateIMU()
{
  //Initialize variables to use in calculations
  float xSum = 0; float ySum = 0; float zSum = 0; float gravSum = 0;
  for(int i = 0; i < 20; i++)
  {
    ism330dhcx.getEvent(&accel, &gyro, &temp);
    xSum = xSum + (accel.acceleration.x*accel.acceleration.x);
    ySum = ySum + (accel.acceleration.y*accel.acceleration.y);
    zSum = zSum + (accel.acceleration.z*accel.acceleration.z);
  }
  gravSum = xSum + ySum + zSum;
  calibX = xSum/gravSum; calibY = ySum/gravSum; calibZ = zSum/gravSum;
  bleX = 0; bleY = 0;
  gyroX = 0; gyroY = 0; gyroZ = 0;
  currentAngleRad = 0;
}

void readIMU(float &gyroX, float &gyroY, float &gyroZ)
{
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;
  //Might use a timer to get an accurate time, for now assuming 0.1s delay from the delay commmand
  currentTime = millis();
  currentAngleRad = currentAngleRad - ((gyroX*calibX + gyroY*calibY + gyroZ*calibZ)*((currentTime - prevTime)/1000.0));
  prevTime = millis();
  if(currentAngleRad > 6.2831852)
  {
    currentAngleRad -= 6.2831852;
  }
  else if(currentAngleRad < 0)
  {
    currentAngleRad += 6.2831852;
  }
}

void currentAngleto2D(float currentAngleRad, int &bleX, int &bleY)
{
  //Hypotenuse = linear (forward) speed, default to 32767 (bleAxisMax)
  float accelAtRot = 16383;
  bleY = 16383 + (int)(sin(currentAngleRad)*accelAtRot); //Y value
  bleX = 16383 + (int)(cos(currentAngleRad)*accelAtRot); //X value
}

// Output integer values on x, y, and z as bluetooth gamepad axes
void bleOutput(int bleX, int bleY)
{
  if(bleGamepad.isConnected())
  {
    bleGamepad.setX(bleX);
    bleGamepad.setY(bleY);
    bleGamepad.sendReport();
  }
}

void loop()
{
    if(digitalRead(34))
    {
      calibrateIMU();
    }
    readIMU(gyroX, gyroY, gyroZ);
    currentAngleto2D(currentAngleRad, bleX, bleY);
    bleOutput(bleX, bleY);
//    Serial.print("\t\tgyroX: ");
//    Serial.print(gyroX);
//    Serial.print(" \tgyroY: ");
//    Serial.print(gyroY);
//    Serial.print(" \tgyroZ: ");
//    Serial.print(gyroZ);
//    Serial.println();
//    
//    Serial.print("\t\tbleX: ");
//    Serial.print(bleX);
//    Serial.print(" \tbleY: ");
//    Serial.print(bleY);
//    Serial.print(" \tbleZ: ");
//    Serial.print(bleZ);
//    Serial.println();
//    Serial.println();
    delay(100); //Delays for 0.1s ==> update current angle accordingly via dps*0.1
}
