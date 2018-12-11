
/*
 *			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *			%				SatTrackr				%
 *			%  Serial, IMU, RTC read demonstration  %
 *			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 */

//*************************************************************************************
/** @file SatTrackr_combined_sketch
 *    This application demonstrates the sensor readout and serial interface of the SatTrackr board.
 
 *  @b Revisions:
 *    12-10-2018: Demonstration ready-revision
 * 
 *  @b Usage:
 *	This application is designed to read a serial input string of the following form: | num.f , num.f , int, int, int;
 *	- Bitwise "or" character,
 *	- followed by a float from -180.0 to 180.0 for the commanded azimuth angle (0.0 === North), then a comma
 *	- Followed by a float from 0.0 to 90.0 for the commanded elevation angle (0.0 === Horizon), then a comma
 *	- Followed by an integer from 0 to 23 for the commanded local time hour (0 === midnight), then a comma
 *	- Followed by an integer from 0 to 59 for the commanded local time minute, then a comma
 *	- Followed by an integer from 0 to 59 for the commanded local time second, then a semicolon to indicate end-of-line
 *	From this command, the code then runs a loop to check whether the IMUs measured orientation matches to within 5 degrees of the commanded azimuth and elevation, and whether the current time is within five seconds of the commanded time.
 *	Once these logical statements are evaluated and found to be true, a success message is output over serial.
 *	This sketch can be compiled for use on an Arduino Nano, Mega, or Uno, and can be uploaded to the SatTrackr Board rev. A as a hex file via the ICSP header.
 *     
 * 
 *  @b License:
 *    This file is copyright 2018 by Alex Johnson and released under the Lesser GNU 
 *    Public License, version 3. It intended for educational use only. 
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

/**
* \mainpage SatTrackr Sensor_Readout_Test
* See \ref appdoc_main "here" for project documentation
* \copydetails appdoc_preface
*
* \page appdoc_preface Overview
* This application demonstrates the sensor readout and serial interface of the SatTrackr board.
*/

/**
* \page appdoc_main SatTrackr Sensor_Readout_Test
* Overview:
* - \ref appdoc_SatTrackr_sensortest_intro
* - \ref appdoc_SatTrackr_sensortest_usage
* - \ref appdoc_SatTrackr_sensortest_compinfo
* - \ref appdoc_SatTrackr_sensortest_contactinfo
*
* \section appdoc_SatTrackr_sensortest_intro Introduction
* This application demonstrates the sensor readout and serial interface functionality of the SatTrackr board.
* It is intended for testing, debugging, and demonstration of these functions.
*
* \subsection Procedure for Setting Up Objects and Variables
* The following steps are performed in sequence to prepare the board for reading sensors and serial commands:
* - Begin I2C Protocol Interface
* - Begin Serial Interface and Set Baud Rate
* - Set Synchronization with RTC 
* - Identify IMU I2C Address
* - IMU Calibration
* - IMU Accelerometer and Gyroscope Initialization
* - Identify IMU Magnetometer I2C Address
* - Initialize and Calibrate IMU Magnetometer
*
* \section appdoc_SatTrackr_sensortest_usage Usage
* This application is designed to read a serial input string of the following form: | num.f , num.f , int, int, int;
* - Bitwise "or" character, 
* - followed by a float from -180.0 to 180.0 for the commanded azimuth angle (0.0 === North), then a comma 
* - Followed by a float from 0.0 to 90.0 for the commanded elevation angle (0.0 === Horizon), then a comma
* - Followed by an integer from 0 to 23 for the commanded local time hour (0 === midnight), then a comma
* - Followed by an integer from 0 to 59 for the commanded local time minute, then a comma
* - Followed by an integer from 0 to 59 for the commanded local time second, then a semicolon to indicate end-of-line
* From this command, the code then runs a loop to check whether the IMUs measured orientation matches to within 5 degrees of the commanded azimuth and elevation, and whether the current time is within five seconds of the commanded time.
* Once these logical statements are evaluated and found to be true, a success message is output over serial.
* 
* \section appdoc_SatTrackr_sensortest_compinfo Compilation Info
* This software can be compiled for use on an Arduino Nano, Mega, or Uno, and can be uploaded to the SatTrackr Board rev. A as a hex file uploaded via the ICSP header.
*
* \section appdoc_SatTrackr_sensortest_contactinfo Contact Information
* For more information on the SatTrackr board, visit:
* <a href="https://github.com/alex-w-johnson/SatTrackr">https://github.com/alex-w-johnson/SatTrackr</a>.
*/


// Included libraries
/** @brief   Includes RTC Library.
 */
#include <DS3232RTC.h>
/** @brief   Includes MPU9250 Library.
 */
#include <MPU9250.h>
/** @brief   Includes Mahoney filter library.
 */
#include <quaternionFilters.h>

// Defined modes for debugging
/** @brief   Yaw debugging.
 */
#define debugYaw false
/** @brief   Imu debugging
 */
#define imuDebugMode false

//Defined IMU address
/** @brief   IMU I2C address when AD0 pin is pulled down.
 */
#define IMU1_ADDRESS 0x69 // Address when AD0 Pin is pulled down, allowing compatibility with RTC address

// Create IMU Object
/** @brief   Creates IMU object.
 */
MPU9250 IMU1; // Name IMU 

// Assign IMU interrupt pin and led pin
int intPin = 12;
int myLed = 13;

// Initialize serial read variables
String receivedString = "";
const int rowLength = 5;
// Command structure: |Commanded azimuth, commanded elevation, commanded hour, commanded minute, commanded second;
static int receivedValue = 0;
static float receivedArray[rowLength] = {};

// Initialize pointing variables
static float commandElevation = 0;
static float commandAzimuth = 0;
static int commandHour = 0;
static int commandMinute = 0;
static int commandSecond = 0;
static bool isPromptDisplayed = false;
static int isPointing = 0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Initialization
void setup() {
  Wire.begin();
  Serial.begin(9600);
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  // Defining imu id 
  byte imu1_id = IMU1.readByte(IMU1_ADDRESS, WHO_AM_I_MPU9250);

  // Perform IMU Self Test for QA of factory compliance
  IMU1.MPU9250SelfTest(IMU1.SelfTest, IMU1_ADDRESS);

  // Calibrate IMU
  IMU1.calibrateMPU9250(IMU1.gyroBias, IMU1.accelBias, IMU1_ADDRESS);

  //Initialize gyro and accel
  IMU1.initMPU9250(IMU1_ADDRESS);
  
  // Device ID for magnetometer, initialization and calibration
  byte imu_mag_id = IMU1.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  IMU1.initAK8963(IMU1.magCalibration);
  
  // Optional magnetometer debugging information
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

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Continuous sequence
void loop() {
  // Dispaly serial prompt for input command
  if(!isPromptDisplayed){
    Serial.println("Input Command: (|Azimuth,Elevation,Hour,Minute,Second;)");
    isPromptDisplayed = true;
  }
  // While loop for reading input command strings (while pointing maneuver is not being executed)
  while (Serial.available() > 0 && isPointing == 0) {
    int idx = 0;
    while (idx < rowLength) {      
      int CharReceived = Serial.read();      
      if (1) {
        receivedString += (char)CharReceived;
      }
      if (CharReceived == '|') {
        receivedString = "";
        delay(20);
      }
      else if (CharReceived == ',') {        
        receivedValue = receivedString.toInt();
        float receivedFloat = (float)receivedValue;        
        receivedString = "";
        receivedArray[idx] = receivedFloat;
        checksum += receivedValue;
        idx++;
        delay(20);
      }
      else if (CharReceived == ';') {
        receivedValue = receivedString.toInt();
        float receivedFloat = (float)receivedValue;        
        receivedString = "";
        receivedArray[idx] = receivedFloat;       
        idx++;
        delay(20);
      }
      delay(50);
    }
   commandAzimuth = receivedArray[0];
   commandElevation = receivedArray[1];
   commandHour = (int)receivedArray[2];
   commandMinute = (int)receivedArray[3];
   commandSecond = (int)receivedArray[4];   
   isPointing = 1; 
  }
  
  // This code based on example MPU9250BasicAHRS.ino
  // Some changes made to streamline execution process to minimize runtime
  if (IMU1.readByte(IMU1_ADDRESS, INT_STATUS) & 0x01 && isPointing == 1)
  {
    // Read accelerometer, gyro, and magnetometer data
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
	
	// Update measurement time for Mahoney filter integral term
    IMU1.updateTime();
    // Update Quaternions
    MahonyQuaternionUpdate(IMU1.ax, IMU1.ay, IMU1.az, IMU1.gx * DEG_TO_RAD,
                           IMU1.gy * DEG_TO_RAD, IMU1.gz * DEG_TO_RAD, IMU1.my,
                           IMU1.mx, IMU1.mz, IMU1.deltat);
    //Tait Bryan Angle Calculation
    
    //Revised yaw calculation
    IMU1.yaw   = atan2(2.0f * (*(getQ() + 3) * *(getQ()) + *(getQ()+2) *
                               *(getQ() + 1)), 1.0f - 2.0f * ((*(getQ()+3) * *(getQ()+3)) + (*(getQ()+2) * *(getQ()+2))));
    
    IMU1.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                               *(getQ() + 2)));
    IMU1.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                               *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
    IMU1.pitch *= -1*RAD_TO_DEG;
    IMU1.yaw   *= RAD_TO_DEG;
	// Input magnetic declination of San Luis Obispo
    // Declination of San Luis Obispo (35°17'37"N 120°40'05"W) is
    //   12° 36' E  ± 0° 20' (or 12.5°) on 2018-12-01
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    IMU1.yaw   -= 12.5;
    IMU1.roll  *= RAD_TO_DEG;

    // Timing variable resets for IMU upon next readout
    IMU1.count = millis();
    IMU1.sumCount = 0;
    IMU1.sum = 0;
   // Display euler angles to serial
    plotAngles(IMU1.pitch,IMU1.yaw);
    checkPointing(IMU1.pitch,IMU1.yaw,commandAzimuth,commandElevation);
  }
  
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Custom functions for code execution

/** @brief   Prints the results of the IMU self test for debugging information to serial.
 *  @details This function only does anything if debugMode is set to true, so if not used 
 *  for debugging, it can just as easily be deleted to improve program readability.
 *	@param	imu is the IMU object created in the setup function
 *	@param	imu_adddress is the I2C byte address of the IMU
 *	@param	debugMode is a boolean that determines whether anything prints
 */
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
/** @brief   Prints the results of the IMU calibration for debugging information to serial.
 *  @details This function only does anything if debugMode is set to true, so if not used 
 *  for debugging, it can just as easily be deleted to improve program readability.
 *	@param	imu is the IMU object created in the setup function
 *	@param	imu_adddress is the I2C byte address of the IMU
 *	@param	debugMode is a boolean that determines whether anything prints
 */
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

/** @brief   Prints the pitch and yaw angles as "pitch.f \t yaw.f"
 *  @details This function works nicely with the serial line plotter in the Arduino IDE, but also 
 *	is a nice way to see the IMU's orientation in real time via the serial monitor.
 *	@param	yaw is the IMU yaw angle
 *	@param	pitch is the IMU pitch angle
 */
void plotAngles(float yaw, float pitch){
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(yaw);
}

/** @brief   Evaluates whether IMU is pointed in the right place at the right time
 *  @details Checks if measured azimuth and elevation are within 5 degrees of the commanded angles, 
 *	and the measured RTC time is within five seconds of the commanded imaging time.
 *	@param	yaw is the IMU yaw angle
 *	@param	pitch is the IMU pitch angle
 *	@param	azimuth is the commanded azimuth angle
 *	@param	elevation is the commanded elevation angle
 */
void checkPointing(float pitch, float yaw, float azimuth, float elevation){
  if (abs(pitch - elevation) <= 5.0f && abs(yaw - azimuth) <= 5.0f && abs(3600*(commandHour - hour()) + 60*(commandMinute - minute()) + (commandSecond - second())) <= 5 && isPointing == 1){
    Serial.println("Made it to the correct orientation!");
    isPointing = 0;
    commandAzimuth = 0.0f;
    commandElevation = 0.0f;
    commandHour = 0;
    commandMinute = 0;
    commandSecond = 0;
    isPromptDisplayed = false;
  }
}
/** @brief   Prints the Euler angles of the IMU
 *  @details This function only does anything if debugMode is set to true, so if not used
 *  for debugging, it can just as easily be deleted to improve program readability.
 *	@param	imu is the IMU object created in the setup function
 *	@param	imu_adddress is the I2C byte address of the IMU
 *	@param	debugMode is a boolean that determines whether anything prints
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

/** @brief   Prints the yaw angle of the IMU
 *  @details It just prints yaw, rotation about z-axis from North
 *	@param	yaw is the IMU yaw angle
 */
void printYaw(float yaw){
  Serial.println(yaw,2);
}

/** @brief   Prints the quaternion components of the IMU
 *  @details Good for debugging quaternion output using the Arduino serial plotter.
 */
void plotQuats(){
  Serial.print(*(getQ()));Serial.print("\t");
  Serial.print(*(getQ()+1));Serial.print("\t");
  Serial.print(*(getQ()+2));Serial.print("\t");
  Serial.println(*(getQ()+3));
}

