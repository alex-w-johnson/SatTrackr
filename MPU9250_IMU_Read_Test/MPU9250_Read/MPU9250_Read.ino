#include "quaternionFilters.h"
#include "MPU9250.h"

#define debugYaw true
#define imuDebugMode false
#define IMU1_ADDRESS 0x68
//#define IMU2_ADDRESS 0x69


MPU9250 IMU1; // Name IMU 1 (on main board)
//MPU9250 IMU2;
int intPin = 12;
int myLed = 13;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  //Find IMU! Dev_id
  byte imu1_id = IMU1.readByte(IMU1_ADDRESS, WHO_AM_I_MPU9250);
  //byte imu2_id = IMU2.readByte(IMU2_ADDRESS, WHO_AM_I_MPU9250);

  //Perform IMU1 Self-test
  IMU1.MPU9250SelfTest(IMU1.SelfTest, IMU1_ADDRESS);
  printSelfTestResults(IMU1, IMU1_ADDRESS, imuDebugMode);

  //Perform IMU2 Self-test
  //IMU2.MPU9250SelfTest(IMU2.SelfTest, IMU2_ADDRESS);
  //(IMU2, IMU2_ADDRESS, imuDebugMode);

  //Calibrate MPU gyro and axlmtr
  IMU1.calibrateMPU9250(IMU1.gyroBias, IMU1.accelBias, IMU1_ADDRESS);
  printCalibrationResults(IMU1, IMU1_ADDRESS, imuDebugMode);

  //IMU2.calibrateMPU9250(IMU2.gyroBias, IMU2.accelBias, IMU2_ADDRESS);
  //printCalibrationResults(IMU2, IMU2_ADDRESS, imuDebugMode);

  //Initialize gyro and accel
  IMU1.initMPU9250(IMU1_ADDRESS);
  //IMU2.initMPU9250(IMU2_ADDRESS);

  byte imu_mag_id = IMU1.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  IMU1.initAK8963(IMU1.magCalibration);
  if (imuDebugMode) {
    Serial.println("Magnetometer Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(IMU1.magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(IMU1.magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(IMU1.magCalibration[2], 2);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //imuRead(IMU1,IMU1_ADDRESS,imuDebugMode,debugYaw);
  if (IMU1.readByte(IMU1_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU1.readAccelData(IMU1.accelCount, IMU1_ADDRESS);
    IMU1.getAres();

    IMU1.ax = (float)IMU1.accelCount[0] * IMU1.aRes; // - accelBias[0];
    IMU1.ay = (float)IMU1.accelCount[1] * IMU1.aRes; // - accelBias[1];
    IMU1.az = (float)IMU1.accelCount[2] * IMU1.aRes; // - accelBias[2];

    IMU1.readGyroData(IMU1.gyroCount, IMU1_ADDRESS);
    IMU1.getGres();

    IMU1.gx = (float)IMU1.gyroCount[0] * IMU1.gRes;
    IMU1.gy = (float)IMU1.gyroCount[1] * IMU1.gRes;
    IMU1.gz = (float)IMU1.gyroCount[2] * IMU1.gRes;

    IMU1.readMagData(IMU1.magCount);  // Read the x/y/z adc values
    IMU1.getMres();
    IMU1.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU1.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU1.magbias[2] = +125.;

    IMU1.mx = (float)IMU1.magCount[0] * IMU1.mRes * IMU1.magCalibration[0] -
              IMU1.magbias[0];
    IMU1.my = (float)IMU1.magCount[1] * IMU1.mRes * IMU1.magCalibration[1] -
              IMU1.magbias[1];
    IMU1.mz = (float)IMU1.magCount[2] * IMU1.mRes * IMU1.magCalibration[2] -
              IMU1.magbias[2];

    IMU1.updateTime();
    // Update Quaternions
    MahonyQuaternionUpdate(IMU1.ax, IMU1.ay, IMU1.az, IMU1.gx * DEG_TO_RAD,
                           IMU1.gy * DEG_TO_RAD, IMU1.gz * DEG_TO_RAD, IMU1.my,
                           IMU1.mx, IMU1.mz, IMU1.deltat);
    //Tait Bryan Angle Calculation
    IMU1.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                               *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
    IMU1.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                               *(getQ() + 2)));
    IMU1.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                               *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
    IMU1.pitch *= RAD_TO_DEG;
    IMU1.yaw   *= RAD_TO_DEG;
    // Declination of San Luis Obispo (35°17'37"N 120°40'05"W) is
    //   12° 36' E  ± 0° 20' (or 12.5°) on 2018-12-01
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    IMU1.yaw   -= 12.5;
    IMU1.roll  *= RAD_TO_DEG;

    printAnglesIMU(IMU1, IMU1_ADDRESS, imuDebugMode);
    IMU1.count = millis();
    IMU1.sumCount = 0;
    IMU1.sum = 0;
    /*
    if(debugYaw){
      printYaw(IMU1.yaw);
    }
    */
    plotAngles(IMU1.pitch,IMU1.yaw);
    //plotQuats();
  }
}

void printSelfTestResults(MPU9250 imu, byte imu_address, bool debugMode) {
  if (debugMode) {
    Serial.print("For IMU located at device address "); Serial.print(imu_address, HEX); Serial.println(":");
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(imu.SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(imu.SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(imu.SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(imu.SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(imu.SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(imu.SelfTest[5], 1); Serial.println("% of factory value");
  }
}

void printCalibrationResults(MPU9250 imu, byte imu_address, bool debugMode) {
  if (debugMode) {
    Serial.print("For IMU located at device address "); Serial.print(imu_address, HEX); Serial.println(":");
    Serial.println("Gyro biases:");
    Serial.print("X: "); Serial.println(imu.gyroBias[1]);
    Serial.print("Y: "); Serial.println(imu.gyroBias[2]);
    Serial.print("Z: "); Serial.println(imu.gyroBias[3]);
    Serial.println("Accel. biases:");
    Serial.print("X: "); Serial.println(imu.accelBias[1]);
    Serial.print("Y: "); Serial.println(imu.accelBias[2]);
    Serial.print("Z: "); Serial.println(imu.accelBias[3]);
  }
}

void plotAngles(float yaw, float pitch){
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(yaw);
}
/*
void imuRead(MPU9250 imu, byte imu_address, bool debugMode, bool debugY) {
  if (imu.readByte(imu_address, INT_STATUS) & 0x01)
  {
    imu.readAccelData(imu.accelCount, imu_address);
    imu.getAres();

    imu.ax = (float)imu.accelCount[0] * imu.aRes; // - accelBias[0];
    imu.ay = (float)imu.accelCount[1] * imu.aRes; // - accelBias[1];
    imu.az = (float)imu.accelCount[2] * imu.aRes; // - accelBias[2];

    imu.readGyroData(imu.gyroCount, imu_address);
    imu.getGres();

    imu.gx = (float)imu.gyroCount[0] * imu.gRes;
    imu.gy = (float)imu.gyroCount[1] * imu.gRes;
    imu.gz = (float)imu.gyroCount[2] * imu.gRes;

    imu.readMagData(imu.magCount);  // Read the x/y/z adc values
    imu.getMres();
    imu.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    imu.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    imu.magbias[2] = +125.;

    imu.mx = (float)imu.magCount[0] * imu.mRes * imu.magCalibration[0] -
              imu.magbias[0];
    imu.my = (float)imu.magCount[1] * imu.mRes * imu.magCalibration[1] -
              imu.magbias[1];
    imu.mz = (float)imu.magCount[2] * imu.mRes * imu.magCalibration[2] -
              imu.magbias[2];

    imu.updateTime();
    // Update Quaternions
    MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD,
                           imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my,
                           imu.mx, imu.mz, imu.deltat);
    //Tait Bryan Angle Calculation
    imu.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                               *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
    imu.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                               *(getQ() + 2)));
    imu.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                               *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
    imu.pitch *= RAD_TO_DEG;
    imu.yaw   *= RAD_TO_DEG;
    // Declination of San Luis Obispo (35°17'37"N 120°40'05"W) is
    //   12° 36' E  ± 0° 20' (or 12.5°) on 2018-12-01
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    imu.yaw   -= 12.5;
    imu.roll  *= RAD_TO_DEG;

    printAnglesIMU(imu, imu_address, debugMode);
    imu.count = millis();
    imu.sumCount = 0;
    imu.sum = 0;
    if(debugY){
      printYaw(imu.yaw);
    }
  }
}
*/
void printAnglesIMU(MPU9250 imu, byte imu_address, bool debugMode) {
  if (debugMode)
  {
    Serial.print("IMU Address 0x"); Serial.print(imu_address, HEX); Serial.println(":");
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(imu.yaw, 2);
    Serial.print(", ");
    Serial.print(imu.pitch, 2);
    Serial.print(", ");
    Serial.println(imu.roll, 2);
    Serial.print("rate = ");
    Serial.print((float)imu.sumCount / imu.sum, 2);
    Serial.println(" Hz");
  }
}

void printYaw(float yaw){
  Serial.println(yaw,2);
}

void plotQuats(){
  Serial.print(*(getQ()));Serial.print("\t");
  Serial.print(*(getQ()+1));Serial.print("\t");
  Serial.print(*(getQ()+2));Serial.print("\t");
  Serial.println(*(getQ()+3));
}

