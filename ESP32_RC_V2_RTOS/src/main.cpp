#include <Wire.h>
#include "ICM_20948.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include "sbus.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Function prototypes
float mapStickToAngle(int value, int center, int min_val, int max_val, float max_angle);

// SBUS object and variable declarations
bfs::SbusRx sbus(&Serial2, 17, -1, true);  // GPIO17 for SBUS reception
bfs::SbusData sbusData;

// Remote control channel mapping
#define ROLL_CHANNEL 0     // Roll channel (left/right) - Channel 0
#define PITCH_CHANNEL 1    // Pitch channel (forward/backward) - Channel 1
#define THROTTLE_CHANNEL 2 // Throttle channel - Channel 2
#define YAW_CHANNEL 3      // Yaw channel (self-rotation) - Channel 3
#define ARM_CHANNEL 4      // Arm channel - Channel 4

// Remote control neutral values
#define ROLL_CENTER 1000   // Roll neutral value
#define PITCH_CENTER 852   // Pitch neutral value
#define YAW_CENTER 1002    // Yaw neutral value

// Remote control actual range
#define ROLL_MIN 306      // Roll minimum value
#define ROLL_MAX 1693     // Roll maximum value
#define PITCH_MIN 306     // Pitch minimum value
#define PITCH_MAX 1662    // Pitch maximum value
#define YAW_MIN 306       // Yaw minimum value
#define YAW_MAX 1688      // Yaw maximum value

// Control parameters
#define STICK_DEADZONE 50  // Stick dead zone value
#define ANGLE_MAX 30.0f    // Maximum angle limit
#define CONTROL_SMOOTH_FACTOR 0.05f // Control smoothing factor (smaller values make movement slower)

//******************************************//
//****************Serial Parameter Tuning***************//
// Serial command handling
#define SERIAL_PORT Serial
#define AD0_VAL 0 // AD0 pin is grounded when 0, VCC when 1

// EEPROM configuration
#define EEPROM_SIZE 512
#define PID_ADDR 0  // PID parameter storage start address

struct PIDParams {
  double Kp_roll;
  double Ki_roll;
  double Kd_roll;
  double Kp_pitch;
  double Ki_pitch;
  double Kd_pitch;
  double Kp_yaw;
  double Ki_yaw;
  double Kd_yaw;
};

PIDParams pidParams; // Current PID parameters

ICM_20948_I2C myICM;

// Hardware configuration
#define ESC_PWM_PIN 15
#define PWM_FREQ 50
#define PWM_RESOLUTION 14
#define FULL_DUTY_CYCLE ((1 << PWM_RESOLUTION) - 1)
#define MIN_THROTTLE 1000    // Microseconds
#define MAX_THROTTLE 2000    // Microseconds
#define DISARM_DUTY 0        // Duty cycle when safety locked

// PWM calculation
const float usPerTick = 1000000.0 / PWM_FREQ / FULL_DUTY_CYCLE;
const int minDuty = MIN_THROTTLE / usPerTick;
const int maxDuty = MAX_THROTTLE / usPerTick;
const int pwmChannel = 7;

// Calibration parameters
const float TARGET_ROLL = -177.0;    // Target Roll angle (
const float TARGET_PITCH = -1.0;    // Target Pitch angle
const float ANGLE_TOLERANCE = 5; // Allowed angle error (±2°)
const int STABLE_COUNT_THRESHOLD = 50; // Number of consecutive stable samples
const int CALIBRATION_TIMEOUT = 60000; // Calibration timeout (60 seconds)

// Servo pin definitions (according to your configuration)
const int pinFront = 27;  // Front servo 1 
const int pinRight = 14;  // Right servo 2
const int pinBack  = 13;  // Back servo 3
const int pinLeft  = 12;  // Left servo 4

// Servo offset calibration (according to your configuration)
const int Frontbias = -3;
const int Rightbias = 5;
const int Backbias  = 16;
const int Leftbias  = 4;

// Servo objects
Servo servoFront;
Servo servoRight;
Servo servoBack;
Servo servoLeft;

// System status
bool isArmed = false;  // Default: not armed
bool isPrintingError = true;  // Default: print angle error data
bool isLowVoltage = false;    // Low voltage status flag

// ============== Debug Mode Switches ==============
// Uncomment/comment the following macro definitions to switch debug mode
//#define DEBUG_ROLL
//#define DEBUG_PITCH
//#define DEBUG_YAW
#define FULL_CONTROL

// ============= PID Parameter Configuration ==============
double Kp_roll, Ki_roll, Kd_roll;
double Kp_pitch, Ki_pitch, Kd_pitch;
double Kp_yaw, Ki_yaw, Kd_yaw;

// ============== Control Object Declarations ==============
double rollSetpoint=0, rollInput, rollOutput;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);

double pitchSetpoint=0, pitchInput, pitchOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

double yawSetpoint=0, yawInput, yawOutput;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);

// ============== PID Input Parameters ==============
float gyroX, gyroY, gyroZ, yaw_init, targetRoll, targetPitch, targetYaw;
float gyroYawRate = 0; // Z-axis angular velocity (°/s)

// LED pin definitions
#define LED_CALIBRATION 33    // Purple LED - Gyroscope calibration indicator
#define LED_ERROR 32          // Red LED - Error status
#define LED_PID_ACTIVE 25     // Blue LED - PID control active status
#define LED_ARMED 26          // Green LED - System armed status
#define BATTERY_PIN 35        // Battery voltage sampling pin

// Battery voltage related parameters
#define VOLTAGE_DIVIDER_RATIO 11.0  // Voltage divider ratio (10K + 1K) / 1K
#define ADC_REF_VOLTAGE 3.3         // ESP32 ADC reference voltage
#define ADC_RESOLUTION 4095         // ESP32 ADC resolution
#define LOW_VOLTAGE_THRESHOLD 13.5  // Low voltage warning threshold (4S battery)
#define VOLTAGE_CHECK_INTERVAL 1000 // Voltage check interval (milliseconds)
#define VOLTAGE_CALIBRATION 1.122    // Voltage calibration coefficient (adjust based on actual measurements)

// PWM channel definitions
#define PWM_CHANNEL_CALIBRATION 0
#define PWM_CHANNEL_ERROR 1
#define PWM_CHANNEL_PID 2
#define PWM_CHANNEL_ARMED 3

float YAW_RC = 0.0f; // Remote control yaw smoothing variable
#define YAW_RC_MAX 35.0f

// YAW rate mode flag
bool isYawRateMode = false;

// YAW rate target
float yawTargetRate = 0.0f;

// Mutex and task handles
SemaphoreHandle_t dataMutex;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t actuatorTaskHandle = NULL;
TaskHandle_t sbusTaskHandle = NULL;
TaskHandle_t batteryTaskHandle = NULL;
TaskHandle_t lowPriorityTaskHandle = NULL;

// Function declarations
void Calibration();//Gyroscope calibration
void vectorControl();//Vector control function
void executePID(); //PID calculation
void setupESC();// Modified throttle setting function
void setThrottle(float percentage);//Throttle function
void emergencyStop();//Emergency stop
void calibrateESC();//ESC calibration
void printHelp();//Command
void processCommand(String input);//Serial command handling function
void ServoCalibration();//Servo calibration
void updatePIDControllers();
void initEEPROM();
void loadPIDParams();
void savePIDParams();
void saveDefaultParams();
void checkBatteryVoltage();
float readBatteryVoltage();
void printBatteryStatus(float voltage);

// SBUS related function declarations
void processSbusData();
float mapSbusToAngle(int sbusValue);
void updateTargetAngles();

// Add angle wrapping handling function
float normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Calculate the minimum difference between two angles
float angleDifference(float current, float target) {
    float diff = normalizeAngle(current - target);
    return diff;
}

// Function to map remote control values to angles
float mapStickToAngle(int value, int center, int min_val, int max_val, float max_angle) {
    // Apply dead zone
    if (abs(value - center) < STICK_DEADZONE) {
        return 0.0f;
    }
    
    // Select mapping range based on whether input value is greater or less than center value
    if (value > center) {
        return map(value, center + STICK_DEADZONE, max_val, 0, max_angle);
    } else {
        return map(value, min_val, center - STICK_DEADZONE, -max_angle, 0);
    }
}

// ============= FreeRTOS Task Definitions =============
void SensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    for (;;) {
        icm_20948_DMP_data_t data;
        if (myICM.readDMPdataFromFIFO(&data) == ICM_20948_Stat_Ok) {
            if (data.header & DMP_header_bitmap_Quat6) {
                double q1 = (double)data.Quat6.Data.Q1 / 1073741824.0;
                double q2 = (double)data.Quat6.Data.Q2 / 1073741824.0;
                double q3 = (double)data.Quat6.Data.Q3 / 1073741824.0;
                double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));
                float roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180/M_PI;
                float pitch = asin(2*(q0*q2 - q3*q1)) * 180/M_PI;
                float yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180/M_PI;
                if (!isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
                    xSemaphoreTake(dataMutex, portMAX_DELAY);
                    gyroX = roll;
                    gyroY = pitch;
                    gyroZ = yaw;
                    xSemaphoreGive(dataMutex);
                }
            }
            if (data.header & DMP_header_bitmap_Gyro) {
                float rate = (float)data.Raw_Gyro.Data.Z / 32768.0f * 2000.0f;
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                gyroYawRate = rate;
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    for (;;) {
        float localGyroX, localGyroY, localGyroZ, localGyroYawRate;
        float localTargetRoll, localTargetPitch, localTargetYaw, localYawTargetRate;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        localGyroX = gyroX;
        localGyroY = gyroY;
        localGyroZ = gyroZ;
        localGyroYawRate = gyroYawRate;
        localTargetRoll = targetRoll;
        localTargetPitch = targetPitch;
        localTargetYaw = targetYaw;
        localYawTargetRate = yawTargetRate;
        xSemaphoreGive(dataMutex);
        float rollError = angleDifference(localGyroX, localTargetRoll);
        float pitchError = angleDifference(localGyroY, localTargetPitch);
        rollInput = rollError;
        rollSetpoint = 0;
        rollPID.Compute();
        pitchInput = pitchError;
        pitchSetpoint = 0;
        pitchPID.Compute();
        yawInput = localGyroYawRate - localYawTargetRate;
        yawSetpoint = 0;
        yawPID.Compute();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ActuatorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
    for (;;) {
        vectorControl();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void SBUSTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    for (;;) {
        processSbusData();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void BatteryTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1Hz
    for (;;) {
        checkBatteryVoltage();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void LowPriorityTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    for (;;) {
        // Serial command handling
        if (Serial.available() > 0) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            processCommand(command);
        }
        // Other auxiliary functions can be added here
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT); // Wait for serial connection
  
  // Initialize SBUS
  sbus.Begin();
  Serial.println("SBUS initialized");
  
  // Initialize LED pins to output mode
  pinMode(LED_CALIBRATION, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_PID_ACTIVE, OUTPUT);
  pinMode(LED_ARMED, OUTPUT);
  
  // Initialize battery voltage sampling pin
  pinMode(BATTERY_PIN, INPUT);
  
  // Initial state: all LEDs off
  digitalWrite(LED_CALIBRATION, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_PID_ACTIVE, LOW);
  digitalWrite(LED_ARMED, LOW);
  
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize EEPROM and load PID parameters
  initEEPROM();
  saveDefaultParams(); // Force recovery to code default parameters
  loadPIDParams();     // Then load to variables
  
  // Update PID parameters to variables
  Kp_roll = pidParams.Kp_roll;
  Ki_roll = pidParams.Ki_roll;
  Kd_roll = pidParams.Kd_roll;
  Kp_pitch = pidParams.Kp_pitch;
  Ki_pitch = pidParams.Ki_pitch;
  Kd_pitch = pidParams.Kd_pitch;
  Kp_yaw = pidParams.Kp_yaw;
  Ki_yaw = pidParams.Ki_yaw;
  Kd_yaw = pidParams.Kd_yaw;
  

   float initialVoltage = readBatteryVoltage();
  printBatteryStatus(initialVoltage);
  checkBatteryVoltage();
  ServoCalibration();//ServoCalibration
    // Check battery voltage once during initialization
 
  // If automatic calibration is not needed at startup, comment out this line
  //calibrateESC();
  //delay(300);

  SERIAL_PORT.println("ESC calibrated!");
  // Initialize PID
  rollPID.SetSampleTime(10);
  pitchPID.SetSampleTime(10);
  yawPID.SetSampleTime(10);
  
  // Set PID mode
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  
  // Set PID output limits
  rollPID.SetOutputLimits(-40, 40);   // Limit Roll axis output range
  pitchPID.SetOutputLimits(-40, 40); // Limit Pitch axis output range
  yawPID.SetOutputLimits(-25, 25);   // Limit Yaw axis output range
  
  // Set PID direction
  rollPID.SetControllerDirection(DIRECT);
  pitchPID.SetControllerDirection(DIRECT);
  yawPID.SetControllerDirection(DIRECT);

  // Set PID parameters
  rollPID.SetTunings(Kp_roll, Ki_roll, Kd_roll);
  pitchPID.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);
  yawPID.SetTunings(Kp_yaw, Ki_yaw, Kd_yaw);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("ICM-20948 Connected");
      
      // Initialize DMP
      if (myICM.initializeDMP() == ICM_20948_Stat_Ok) {
        // Enable game rotation vector (quaternion)
        myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
        myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
        myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
        myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0);
        myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0); // Maximum output rate (approx 110Hz)
        myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0);
        myICM.enableDMP();
        myICM.enableFIFO();
        myICM.resetFIFO();
        initialized = true;
        SERIAL_PORT.println("DMP Initialized!");
      }
    }
    delay(500);
  }
  delay(500);
  //calibrateESC();
  // Re-enable calibration
  Calibration();
  
  // Update target values after calibration
  targetRoll = TARGET_ROLL;
  targetPitch = TARGET_PITCH;
  targetYaw = yaw_init; // Yaw target set to initial value
  rollSetpoint = targetRoll;
  pitchSetpoint = targetPitch; 
  yawSetpoint = targetYaw;

  printHelp();

  // Print current PID parameters
  Serial.println("\n[INIT] Current PID parameters:");
  Serial.print("Roll  - P:"); Serial.print(Kp_roll); Serial.print(" I:"); Serial.print(Ki_roll); Serial.print(" D:"); Serial.println(Kd_roll);
  Serial.print("Pitch - P:"); Serial.print(Kp_pitch); Serial.print(" I:"); Serial.print(Ki_pitch); Serial.print(" D:"); Serial.println(Kd_pitch);
  Serial.print("Yaw   - P:"); Serial.print(Kp_yaw); Serial.print(" I:"); Serial.print(Ki_yaw); Serial.print(" D:"); Serial.println(Kd_yaw);

  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  // Create tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 3, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL, 2, &controlTaskHandle, 1);
  xTaskCreatePinnedToCore(ActuatorTask, "ActuatorTask", 4096, NULL, 1, &actuatorTaskHandle, 1);
  xTaskCreatePinnedToCore(SBUSTask, "SBUSTask", 4096, NULL, 2, &sbusTaskHandle, 1);
  xTaskCreatePinnedToCore(BatteryTask, "BatteryTask", 2048, NULL, 1, &batteryTaskHandle, 1);
  xTaskCreatePinnedToCore(LowPriorityTask, "LowPriorityTask", 3072, NULL, 1, &lowPriorityTaskHandle, 1);
}

void loop() {
    // Empty, all functions are handled by FreeRTOS tasks
}

void Calibration()
{
    SERIAL_PORT.println("Starting calibration...");
    SERIAL_PORT.println("Place the sensor in the target attitude [ Roll=90°, Pitch=1° ]");
  
    // Calibration phase: wait for data to stabilize
    bool isStable = false;
    unsigned long startTime = millis();
    int stableCount = 0;
    unsigned long previousMillis = 0;
    const long blinkInterval = 500; // Blink interval 500ms
    bool ledState = false;
  
    while (!isStable && (millis() - startTime < CALIBRATION_TIMEOUT)) {
      // LED blinking effect
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_CALIBRATION, ledState); // Use digitalWrite instead of PWM
      }

      icm_20948_DMP_data_t data;
      if (myICM.readDMPdataFromFIFO(&data) == ICM_20948_Stat_Ok) {
        if (data.header & DMP_header_bitmap_Quat6) {
          double q1 = (double)data.Quat6.Data.Q1 / 1073741824.0;
          double q2 = (double)data.Quat6.Data.Q2 / 1073741824.0;
          double q3 = (double)data.Quat6.Data.Q3 / 1073741824.0;
          double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));
  
          float roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180/M_PI;
          float pitch = asin(2*(q0*q2 - q3*q1)) * 180/M_PI;
          float yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180/M_PI;  //YAW

          yaw_init = yaw;
          // Check if within target range
          bool isRollStable = (abs(roll - TARGET_ROLL) < ANGLE_TOLERANCE);
          bool isPitchStable = (abs(pitch - TARGET_PITCH) < ANGLE_TOLERANCE);
  
          if (isRollStable && isPitchStable) {
            stableCount++;
          } else {
            stableCount = 0; // Data fluctuation, reset counter
          }
  
          // Number of consecutive stable samples reaches threshold
          if (stableCount >= STABLE_COUNT_THRESHOLD) {
            isStable = true;
            SERIAL_PORT.println("Calibration complete! System remains locked");
            digitalWrite(LED_CALIBRATION, HIGH); // LED remains on after calibration
          }
        }
      }
    }
  
    if (!isStable) {
      SERIAL_PORT.println("Calibration Overtime! Please check the sensor placement !");
      digitalWrite(LED_CALIBRATION, LOW); // Turn off LED
      digitalWrite(LED_ERROR, HIGH);     // Error LED turns on
      while(1); // Stop running
    }
}

void vectorControl() {
  // If in low voltage state, stop vector control
  if (isLowVoltage) {
    // Return all servos to neutral position
    const int baseAngle = 90;
    servoFront.write(baseAngle + Frontbias);
    servoRight.write(baseAngle + Rightbias);
    servoBack.write(baseAngle + Backbias);
    servoLeft.write(baseAngle + Leftbias);
    return;
  }

  const int baseAngle = 90; // Neutral angle changed back to 90 degrees
  float rollAdj = 0, pitchAdj = 0, yawAdj = 0;

  // Get adjustment amounts for each axis
  #ifdef DEBUG_ROLL
  rollAdj = constrain(rollOutput, -45, 45);
  #elif defined(DEBUG_PITCH)
  pitchAdj = constrain(pitchOutput, -30, 30);
  #elif defined(DEBUG_YAW)
  yawAdj = constrain(yawOutput, -15, 15);
  #else
  rollAdj = constrain(rollOutput, -40, 40);
  pitchAdj = constrain(pitchOutput, -40, 40);
  yawAdj = constrain(yawOutput, -35, 35);
  //yawAdj += (rollOutput + pitchOutput) * 0.3;
  #endif

  // Apply servo offsets and write angles
  //servoFront.write(baseAngle + pitchAdj - yawAdj + Frontbias);
  //servoRight.write(baseAngle - rollAdj - yawAdj + Rightbias);
  //servoBack.write(baseAngle - pitchAdj + yawAdj + Backbias);
  //servoLeft.write(baseAngle + rollAdj + yawAdj + Leftbias);

  //roll
  //servoRight.write(baseAngle + rollAdj + yawAdj + Leftbias);
  //servoLeft.write(baseAngle - rollAdj - yawAdj + Rightbias);
  
  //roll 2 4 
  servoRight.write(constrain(baseAngle + rollAdj + Rightbias- yawAdj ,61+Rightbias,120+Rightbias));
  servoLeft.write(constrain(baseAngle - rollAdj + Leftbias-yawAdj , 61+Leftbias, 120+ Leftbias));
  // servoRight.write(baseAngle + rollAdj + Leftbias);
  // servoLeft.write(baseAngle - rollAdj  + Rightbias);

  
  //pitch 1 3
  servoFront.write(constrain(baseAngle - pitchAdj + Frontbias- yawAdj ,61 + Frontbias,120 + Frontbias));
  servoBack.write(constrain(baseAngle + pitchAdj + Backbias- yawAdj, 61 + Backbias,120+ Backbias));
  // SERIAL_PORT.print(", Pitch:");

}

// ============== Core Control Functions ==============
void executePID() {
  // Check if input values are valid
  if (isnan(gyroX) || isnan(gyroY) || isnan(gyroZ)) {
    SERIAL_PORT.println("Invalid PID input values");
    return;
  }

  // Calculate error (considering angle wrapping)
  float rollError = angleDifference(gyroX, targetRoll);
  float pitchError = angleDifference(gyroY, targetPitch);

  // Roll axis PID (X-axis)
  #if defined(DEBUG_ROLL) || defined(FULL_CONTROL)
  rollInput = rollError;  // Use angle error directly as PID input
  rollSetpoint = 0;      // Target error is 0
  rollPID.Compute();     // Execute PID calculation
  #endif
  
  // Pitch axis PID (Y-axis)
  #if defined(DEBUG_PITCH) || defined(FULL_CONTROL)
  pitchInput = pitchError;  // Use angle error directly as PID input
  pitchSetpoint = 0;       // Target error is 0
  pitchPID.Compute();      // Execute PID calculation
  #endif
  
  // YAW axis rate error closed-loop
  yawInput = gyroYawRate - yawTargetRate; // Input is current angular velocity minus target rate
  yawSetpoint = 0; // Target is 0
  yawPID.Compute();

  // Serial output for rate closed-loop information
  Serial.print("[YAW RATE LOOP] gyroYawRate: ");
  Serial.print(gyroYawRate);
  Serial.print(", targetRate: ");
  Serial.print(yawTargetRate);
  Serial.print(", input(error): ");
  Serial.print(yawInput);
  Serial.print(", output: ");
  Serial.println(yawOutput);

  // Check if PID outputs are valid
  if (isnan(rollOutput) || isnan(pitchOutput) || isnan(yawOutput)) {
    SERIAL_PORT.println("Invalid PID outputs");
    rollOutput = 0;
    pitchOutput = 0;
    yawOutput = 0;
    return;
  }

  // LED indication when PID is active
  if (abs(rollOutput) > 0.1 || abs(pitchOutput) > 0.1 || abs(yawOutput) > 0.1) {
    digitalWrite(LED_PID_ACTIVE, HIGH); // Blue LED turns on
  } else {
    digitalWrite(LED_PID_ACTIVE, LOW);   // Blue LED turns off
  }
}



void setupESC() {
  ledcSetup(pwmChannel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ESC_PWM_PIN, pwmChannel);
  
  // Initial safe state: zero signal
  ledcWrite(pwmChannel, DISARM_DUTY);
  Serial.println("System in SAFE MODE (DISARMED)");
}

// Modified throttle setting function
void setThrottle(float percentage) {
  if(!isArmed) {
    Serial.println("Operation denied: Please send ARM command to unlock system first");
    return;
  }
  
  percentage = constrain(percentage, 0.0, 100.0);
  // Calculate with floating point to ensure precision
  int duty = minDuty + (maxDuty - minDuty) * (percentage / 100.0);
  ledcWrite(pwmChannel, duty);
  Serial.print("Throttle set to: ");
  Serial.print(percentage);
  Serial.println("%");
}



void emergencyStop() {
  ledcWrite(pwmChannel, DISARM_DUTY);
  isArmed = false;
  digitalWrite(LED_ARMED, LOW);    // Turn off green LED
  digitalWrite(LED_ERROR, HIGH);  // Red LED turns on
  Serial.println("Emergency stop activated! System locked");
}

void calibrateESC() {
  if(isArmed) {
    Serial.println("System must be locked before calibration!");
    digitalWrite(LED_ERROR, HIGH); // Red LED turns on
    return;
  }
  
  Serial.println("Starting calibration...");
  digitalWrite(LED_CALIBRATION, HIGH); // Purple LED turns on
  
  // Enter calibration mode
  ledcWrite(pwmChannel, maxDuty);
  delay(100);
  ledcWrite(pwmChannel, minDuty);
  delay(4000);
  
  digitalWrite(LED_CALIBRATION, LOW); // Turn off purple LED
  Serial.println("Calibration complete! System remains locked");
}

void printHelp() {
  Serial.println("\nAvailable commands:");
  Serial.println("THR [0-100]         - Set throttle percentage");
  Serial.println("STOP                - Emergency stop and lock");
  Serial.println("CALIBRATE           - ESC calibration");
  Serial.println("ARM                 - Unlock system");
  Serial.println("DISARM              - Lock system");
  Serial.println("STATUS              - View system status");
  Serial.println("HELP                - Show help information");
  Serial.println("PID [axis] [p/i/d] [value] - Set PID parameters, e.g.: PID roll p 2.0");
  Serial.println("TARGET [r/p/y] [value] - Set target angle, e.g.: TARGET r 90.0");
  Serial.println("PRINT               - Print current PID parameters and target angles");
  Serial.println("ERROR ON/OFF        - Enable/disable angle error data transmission");
}

void processCommand(String input) { 
  input.trim();
  input.toUpperCase();
  
  // Split command by space
  int spaceIndex = input.indexOf(' ');
  String command = (spaceIndex == -1) ? input : input.substring(0, spaceIndex);
  String param = (spaceIndex != -1) ? input.substring(spaceIndex + 1) : "";

  if(command == "THR") {
    if(param.length() > 0) {
      float throttle = param.toFloat();
      setThrottle(throttle);
    } else {
      Serial.println("Parameter error! Usage: THR 50");
    }
  }
  else if(command == "STOP") {
    emergencyStop();
  }
  else if(command == "CALIBRATE") {
    calibrateESC();
  }
  else if(command == "ARM") {
    if(!isArmed) {
      ledcWrite(pwmChannel, minDuty);  // Send minimum throttle signal to unlock
      delay(500);                      // Wait for ESC to recognize
      isArmed = true;
      digitalWrite(LED_ARMED, HIGH); // Green LED turns on
      digitalWrite(LED_ERROR, LOW);   // Turn off error LED
      Serial.println("System unlocked, ready for operation");
    } else {
      Serial.println("System is already unlocked");
    }
  }
  else if(command == "DISARM") {
    emergencyStop();
  }
  else if(command == "STATUS") {
    Serial.print("Current status: ");
    Serial.println(isArmed ? "Unlocked" : "Locked");
    Serial.print("Throttle signal: ");
    Serial.println(ledcRead(pwmChannel) * usPerTick);
  }
  else if(command == "HELP") {
    printHelp();
  }
  else if(command == "PID") {
    // Parse PID parameter setting command
    // Format: PID [axis] [p/i/d] [value]
    // Example: PID roll p 2.0
    String params[3];
    int paramIndex = 0;
    int lastSpace = -1;
    
    for(int i = 0; i < param.length() && paramIndex < 3; i++) {
      if(param[i] == ' ') {
        params[paramIndex++] = param.substring(lastSpace + 1, i);
        lastSpace = i;
      }
    }
    if(paramIndex == 2) {
      params[2] = param.substring(lastSpace + 1);
      
      float value = params[2].toFloat();
      String axis = params[0];
      String param = params[1];
      
      if(axis == "ROLL") {
        if(param == "P") {
          Kp_roll = value;
          pidParams.Kp_roll = value;
        }
        else if(param == "I") {
          Ki_roll = value;
          pidParams.Ki_roll = value;
        }
        else if(param == "D") {
          Kd_roll = value;
          pidParams.Kd_roll = value;
        }
      }
      else if(axis == "PITCH") {
        if(param == "P") {
          Kp_pitch = value;
          pidParams.Kp_pitch = value;
        }
        else if(param == "I") {
          Ki_pitch = value;
          pidParams.Ki_pitch = value;
        }
        else if(param == "D") {
          Kd_pitch = value;
          pidParams.Kd_pitch = value;
        }
      }
      else if(axis == "YAW") {
        if(param == "P") {
          Kp_yaw = value;
          pidParams.Kp_yaw = value;
        }
        else if(param == "I") {
          Ki_yaw = value;
          pidParams.Ki_yaw = value;
        }
        else if(param == "D") {
          Kd_yaw = value;
          pidParams.Kd_yaw = value;
        }
      }
      
      savePIDParams(); // Save to EEPROM
      updatePIDControllers();
      Serial.println("PID parameters updated and saved");
    } else {
      Serial.println("Parameter error! Usage: PID [roll/pitch/yaw] [p/i/d] [value]");
    }
  }
  else if(command == "TARGET") {
    // Parse target angle setting command
    // Format: TARGET [r/p/y] [value]
    // Example: TARGET r 90.0
    String params[2];
    int paramIndex = 0;
    int lastSpace = -1;
    
    for(int i = 0; i < param.length() && paramIndex < 2; i++) {
      if(param[i] == ' ') {
        params[paramIndex++] = param.substring(lastSpace + 1, i);
        lastSpace = i;
      }
    }
    if(paramIndex == 1) {
      params[1] = param.substring(lastSpace + 1);
      
      float value = params[1].toFloat();
      String axis = params[0];
      
      if(axis == "R") {
        targetRoll = value;
        rollSetpoint = value;
      }
      else if(axis == "P") {
        targetPitch = value;
        pitchSetpoint = value;
      }
      else if(axis == "Y") {
        targetYaw = value;
        yawSetpoint = value;
      }
      
      Serial.println("Target angle updated");
    } else {
      Serial.println("Parameter error! Usage: TARGET [r/p/y] [value]");
    }
  }
  else if(command == "PRINT") {
    Serial.println("\nCurrent PID parameters:");
    Serial.println("Roll  - P:" + String(Kp_roll) + " I:" + String(Ki_roll) + " D:" + String(Kd_roll));
    Serial.println("Pitch - P:" + String(Kp_pitch) + " I:" + String(Ki_pitch) + " D:" + String(Kd_pitch));
    Serial.println("Yaw   - P:" + String(Kp_yaw) + " I:" + String(Ki_yaw) + " D:" + String(Kd_yaw));
    
    Serial.println("\nCurrent target angles:");
    Serial.println("Roll: " + String(targetRoll) + "°");
    Serial.println("Pitch: " + String(targetPitch) + "°");
    Serial.println("Yaw: " + String(targetYaw) + "°");
  }
  else if(command == "ERROR") {
    if(param == "ON") {
      isPrintingError = true;
      Serial.println("Angle error data transmission enabled");
    }
    else if(param == "OFF") {
      isPrintingError = false;
      Serial.println("Angle error data transmission disabled");
    }
    else {
      Serial.println("Parameter error! Usage: ERROR ON/OFF");
    }
  }
  else {
    Serial.println("Unknown command, type HELP for available commands");
  }
}

void updatePIDControllers() {
    rollPID.SetTunings(Kp_roll, Ki_roll, Kd_roll);
    pitchPID.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);
    yawPID.SetTunings(Kp_yaw, Ki_yaw, Kd_yaw);
}

void ServoCalibration()
{
   if (isLowVoltage) {
    // Return all servos to neutral position
    const int baseAngle = 90;
    servoFront.write(baseAngle + Frontbias);
    servoRight.write(baseAngle + Rightbias);
    servoBack.write(baseAngle + Backbias);
    servoLeft.write(baseAngle + Leftbias);
    return;
  }
  // Mount servos
  servoFront.attach(pinFront);
  servoRight.attach(pinRight);
  servoBack.attach(pinBack);
  servoLeft.attach(pinLeft);
  
  Serial.println("Starting servo test...");
  
  // Test all servos from 90 degrees to 135 degrees
  for(int angle = 90; angle <= 135; angle += 1) {
    // Serial.print("Testing angle: ");
    // Serial.println(angle);
    
    // All servos move simultaneously
    servoFront.write(angle + Frontbias);
    servoRight.write(angle + Rightbias);
    servoBack.write(angle + Backbias);
    servoLeft.write(angle + Leftbias);
    
    delay(10); // Wait for servos to move
  }
  
  // Return from 135 degrees to 45 degrees
  for(int angle = 135; angle >= 45; angle -= 1) {
    // Serial.print("Testing angle: ");
    // Serial.println(angle);
    
    // All servos move simultaneously
    servoFront.write(angle + Frontbias);
    servoRight.write(angle + Rightbias);
    servoBack.write(angle + Backbias);
    servoLeft.write(angle + Leftbias);
    
    delay(10); // Wait for servos to move
  }
    for(int angle = 45; angle <= 90; angle += 1) {
    // Serial.print("Testing angle: ");
    // Serial.println(angle);
    
    // All servos move simultaneously
    servoFront.write(angle + Frontbias);
    servoRight.write(angle + Rightbias);
    servoBack.write(angle + Backbias);
    servoLeft.write(angle + Leftbias);
    
    delay(10); // Wait for servos to move
  }
  // Finally return to neutral position 90 degrees
  Serial.println("Returning to neutral position 90 degrees");

  delay(100);
  Serial.println("Servo test complete");
  
  setupESC();
  printHelp();
}

void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if(EEPROM.read(PID_ADDR) == 255) { // First use
      Serial.println("No saved PID params, using defaults");
      saveDefaultParams();
  } else {
      loadPIDParams();
      Serial.println("Loaded PID params from EEPROM");
  }
}

void loadPIDParams() {
  EEPROM.get(PID_ADDR, pidParams);
}

void savePIDParams() {
  EEPROM.put(PID_ADDR, pidParams);
  EEPROM.commit();
  Serial.println("PID params saved to EEPROM");
}

void saveDefaultParams() {
  pidParams = {
      .Kp_roll = 2.0,    // Increase proportional coefficient
      .Ki_roll = 0.0,    // Add integral term
      .Kd_roll = 0.3,    // Increase derivative coefficient
      .Kp_pitch = 2.0,   // Increase proportional coefficient
      .Ki_pitch = 0.0,   // Add integral term
      .Kd_pitch = 0.3,   // Increase derivative coefficient
      .Kp_yaw = 0.3,   // YAW proportional coefficient decreased
      .Ki_yaw = 0.001,     // YAW integral coefficient
      .Kd_yaw = 0.00      // YAW derivative coefficient
  };
  savePIDParams();
}

// Read battery voltage
float readBatteryVoltage() {
  const int samples = 30;  // Number of samples
  long sum = 0;
  
  // Take multiple samples and average
  for(int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(1);  // Short delay to ensure stable sampling
  }
  
  float averageADC = sum / (float)samples;
  // Apply calibration coefficient
  float voltage = (averageADC * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO * VOLTAGE_CALIBRATION) / ADC_RESOLUTION;
  return voltage;
}

// Check battery voltage status
void checkBatteryVoltage() {
  float voltage = readBatteryVoltage();
  
  if (voltage < LOW_VOLTAGE_THRESHOLD) {
    // Set low voltage status
    isLowVoltage = true;
    
    // Low voltage warning: blink blue LED
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_ERROR, ledState);
    
    // Print warning message every 5 checks
    static int warningCount = 0;
    if (warningCount++ % 5 == 0) {
      Serial.print("Warning: Low battery voltage! Current voltage: ");
      Serial.print(voltage);
      Serial.println("V");
    }
  } else {
    // Voltage恢复正常
    if (isLowVoltage) {
      isLowVoltage = false;
      Serial.println("Battery voltage returned to normal, vector control re-enabled");
    }
    // Ensure LED is off
    digitalWrite(LED_ERROR, LOW);
  }
}

// Print battery status
void printBatteryStatus(float voltage) {
  Serial.print("Battery voltage: ");
  Serial.print(voltage);
  Serial.print("V (");
  
  if (voltage < LOW_VOLTAGE_THRESHOLD) {
    Serial.println("Low voltage warning!)");
  } else {
    Serial.println("Normal)");
  }
}

// SBUS data processing function
void processSbusData() {
  if (sbus.Read()) {
    sbusData = sbus.data();
    
    // Disable SBUS channel serial output
    /*
    SERIAL_PORT.print("SBUS: Roll(CH0):");
    SERIAL_PORT.print(sbusData.ch[ROLL_CHANNEL]);
    SERIAL_PORT.print(" Pitch(CH1):");
    SERIAL_PORT.print(sbusData.ch[PITCH_CHANNEL]);
    SERIAL_PORT.print(" Throttle(CH2):");
    SERIAL_PORT.print(sbusData.ch[THROTTLE_CHANNEL]);
    SERIAL_PORT.print(" Yaw(CH3):");
    SERIAL_PORT.print(sbusData.ch[YAW_CHANNEL]);
    SERIAL_PORT.print(" Arm(CH4):");
    SERIAL_PORT.println(sbusData.ch[ARM_CHANNEL]);
    */
    
    // Check arm channel
    if (sbusData.ch[ARM_CHANNEL] < 700) {
      if (!isArmed) {
        isArmed = true;
        digitalWrite(LED_ARMED, HIGH);
        digitalWrite(LED_ERROR, LOW);
        Serial.println("System armed by remote control");
      }
    } else {
      if (isArmed) {
        isArmed = false;
        digitalWrite(LED_ARMED, LOW);
        Serial.println("System disarmed by remote control");
      }
    }

    // Only process control inputs if system is armed
    if (isArmed) {
      // Update target angles
      updateTargetAngles();
      
      // // Output calculated target angles and current angles
      // SERIAL_PORT.print("Current/Target - Roll:");
      // SERIAL_PORT.print(gyroX);
      // SERIAL_PORT.print("/");
      // SERIAL_PORT.print(targetRoll);
      // SERIAL_PORT.print(" Pitch:");
      // SERIAL_PORT.print(gyroY);
      // SERIAL_PORT.print("/");
      // SERIAL_PORT.print(targetPitch);
      // SERIAL_PORT.print(" Yaw:");
      // SERIAL_PORT.print(gyroZ);
      // SERIAL_PORT.print("/");
      // SERIAL_PORT.println(targetYaw);
    }
  }
}

// Smooth control mapping for roll, pitch, yaw (no throttle SBUS control)
void updateTargetAngles() {
    int rollValue = sbusData.ch[ROLL_CHANNEL];
    int pitchValue = sbusData.ch[PITCH_CHANNEL];
    int yawValue = sbusData.ch[YAW_CHANNEL];
    
    // roll
    float newRollAngle = 0.0f;
    if (rollValue > ROLL_CENTER + STICK_DEADZONE) {
        newRollAngle = (float)(rollValue - (ROLL_CENTER + STICK_DEADZONE)) / (ROLL_MAX - (ROLL_CENTER + STICK_DEADZONE)) * ANGLE_MAX;
    } else if (rollValue < ROLL_CENTER - STICK_DEADZONE) {
        newRollAngle = (float)(rollValue - (ROLL_CENTER - STICK_DEADZONE)) / ((ROLL_CENTER - STICK_DEADZONE) - ROLL_MIN) * ANGLE_MAX;
    }
    // pitch
    float newPitchAngle = 0.0f;
    if (pitchValue > PITCH_CENTER + STICK_DEADZONE) {
        newPitchAngle = (float)(pitchValue - (PITCH_CENTER + STICK_DEADZONE)) / (PITCH_MAX - (PITCH_CENTER + STICK_DEADZONE)) * ANGLE_MAX;
    } else if (pitchValue < PITCH_CENTER - STICK_DEADZONE) {
        newPitchAngle = (float)(pitchValue - (PITCH_CENTER - STICK_DEADZONE)) / ((PITCH_CENTER - STICK_DEADZONE) - PITCH_MIN) * ANGLE_MAX;
    }
    // yaw
    float newYawDelta = 0.0f;
    if (yawValue > YAW_CENTER + STICK_DEADZONE) {
        newYawDelta = (float)(yawValue - (YAW_CENTER + STICK_DEADZONE)) / (YAW_MAX - (YAW_CENTER + STICK_DEADZONE)) * 70.0f; // -10~10
    } else if (yawValue < YAW_CENTER - STICK_DEADZONE) {
        newYawDelta = (float)(yawValue - (YAW_CENTER - STICK_DEADZONE)) / ((YAW_CENTER - STICK_DEADZONE) - YAW_MIN) * 70.0f; // -10~10
    }
    // Determine if YAW remote control is off-center
    if (abs(yawValue - YAW_CENTER) > STICK_DEADZONE) {
        isYawRateMode = true;
        YAW_RC = newYawDelta; // Rate direct ( -10~10)
        yawTargetRate = YAW_RC;
    } else {
        isYawRateMode = false;
        YAW_RC = 0;
        yawTargetRate = 0;
    }
    // Smooth roll/pitch target angles
    targetRoll = TARGET_ROLL + newRollAngle * CONTROL_SMOOTH_FACTOR + (targetRoll - TARGET_ROLL) * (1.0f - CONTROL_SMOOTH_FACTOR);
    targetPitch = TARGET_PITCH + newPitchAngle * CONTROL_SMOOTH_FACTOR + (targetPitch - TARGET_PITCH) * (1.0f - CONTROL_SMOOTH_FACTOR);
    // Yaw target angle remains constant, always the initial target
    rollSetpoint = targetRoll;
    pitchSetpoint = targetPitch;
    yawSetpoint = targetYaw;
}