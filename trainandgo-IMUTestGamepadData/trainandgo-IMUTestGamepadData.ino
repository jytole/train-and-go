// Basic demo for accelerometer/gyro readings from Adafruit ISM330DHCX

#include <Adafruit_ISM330DHCX.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

//Variables to calculate an average gravity
float gyroX, gyroY, gyroZ, gyroRange;
int bleX, bleY;
float calibX, calibY, calibZ;
float currentAngleDeg, currentAngleRad;

void calibrateIMU();

Adafruit_ISM330DHCX ism330dhcx;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

unsigned long currentTime;
unsigned long prevTime;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ISM330DHCX test!");

  if (!ism330dhcx.begin_I2C()) {
    // if (!ism330dhcx.begin_SPI(LSM_CS)) {
    // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ISM330DHCX Found!");

  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  Serial.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  ism330dhcx.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;

  }

  ism330dhcx.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2

  gyroRange = 20.02;

  calibrateIMU();
}

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
  calibX = xSum/gravSum; calibY = ySum/gravSum; calibZ = zSum / gravSum;
  bleX = 0; bleY = 0;
  gyroX = 0; gyroY = 0; gyroZ = 0;
  currentAngleDeg = 0;
  currentAngleRad = 0;
  Serial.print("\t\tAxis Sums: x: ");
  Serial.print(xSum);
  Serial.print(" \ty: ");
  Serial.print(ySum);
  Serial.print(" \tz: ");
  Serial.print(zSum);
  Serial.print("\tGrav: ");
  Serial.println(gravSum);
  
  Serial.print("\t\tCalibration Values: x: ");
  Serial.print(calibX);
  Serial.print(" \ty: ");
  Serial.print(calibY);
  Serial.print(" \tz: ");
  Serial.println(calibZ);
}

void readIMU(float &gyroX, float &gyroY, float &gyroZ)
{
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;
  //Might use a timer to get an accurate time, for now assuming 0.1s delay from the delay commmand
  currentTime = millis();
  currentAngleRad = currentAngleRad + ((gyroX*calibX + gyroY*calibY + gyroZ*calibZ)*((currentTime - prevTime) / 1000.0));
  prevTime = millis();
  if(currentAngleRad > 6.2831852)
  {
    currentAngleRad -= 6.2831852;
  }
  else if(currentAngleRad < 0)
  {
    currentAngleRad += 6.2831852;
  }
  currentAngleDeg = currentAngleRad * (180/3.1415926);
  Serial.print("\t\tcurrentAngleDeg: ");
  Serial.println(currentAngleDeg);
}

void currentAngleto2D(float currentAngleDeg, int &bleX, int &bleY)
{
  //Hypotenuse = linear (forward) speed, default to 32767 (bleAxisMax)
  float accelAtRot = 16383;
  float currentAngleRad = currentAngleDeg * (3.1415926/180);
  bleY = 16383 + (int)(sin(currentAngleRad)*accelAtRot); //Y value
  bleX = 16383 + (int)(cos(currentAngleRad)*accelAtRot); //X value
  Serial.print("\t\tcurrentAngleRad: ");
  Serial.println(currentAngleRad);
  Serial.print("\t\tbleX: ");
  Serial.print(bleX);
  Serial.print(" \tbleY: ");
  Serial.print(bleY);
}

void loop() {
  //  /* Get a new normalized sensor event */
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
  
  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  readIMU(gyroX, gyroY, gyroZ);
  currentAngleto2D(currentAngleDeg, bleX, bleY);
  
  delay(100);

  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
}
