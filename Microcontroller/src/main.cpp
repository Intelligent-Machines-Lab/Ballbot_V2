#include "wit_c_sdk.h"
#include "reg.h"
#include <BasicLinearAlgebra.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AccelStepper.h>
#include <PID_v1_bc.h>
#include <WiFi.h>
   
#define LED_OB        2
#define RX            16
#define TX            17
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
#define g             9.81

#define MS1   33
#define MS2   14
#define MS3   32

#define MOTOR_1_STEP 12
#define MOTOR_1_DIR 13

#define MOTOR_2_STEP 19
#define MOTOR_2_DIR 18

#define MOTOR_3_STEP 25
#define MOTOR_3_DIR 26

using namespace BLA;

Matrix<3> gravity_world = {0, 0, g};
Matrix<4> pos_world, pos_robot;
Matrix<3> vel_world_out, vel_robot_out;   // Velocity data comming from sensors other than the IMU (not directly connected to the ESP32)
Matrix<3> vel_world_odom, vel_robot_odom; // Velocity data from wheels velocity
Matrix<4> vel_world, vel_world_imu, vel_robot, vel_robot_imu;
Matrix<4, 4> R_yaw;

struct FilterState {
  float filteredValue;
  bool firstRun;
}; 
FilterState gyroXFilter, gyroYFilter, gyroZFilter, 
            accXFilter, accYFilter, accZFilter,
            txFilter, tyFilter;

float lowPassFilter(float rawValue, float alpha, FilterState* state);
float alphaAccel = 1.0, alphaGyro = 0.75, alphaTorque = 1.0;

bool debug = false;
int calibDuration = 0;
int loopCount = 0, outerLoopCount = 0, integralXCount = 0, integralYCount = 0;

unsigned long timeStart;
int status = 0;

float controlRate = 200;                    // Loop Frequency in Hz
float sampleTime = 1/controlRate;           // Loop Period in s
float sat, satAngle, W1 = 0.0, W2 = 0.0, W3 = 0.0, wx = 0.0, wy = 0.0, wz = 0.0;
double satXY = 5*2*PI;
int controller = 0;

int stepRes = 32;
int stepsPerRev = 200*stepRes;

float accX_world = 0.0, accY_world = 0.0, accZ_world = 0.0, accX_robot = 0.0, accY_robot = 0.0, accZ_robot = 0.0;
float posX_world = 0.0, posY_world = 0.0, posX_world_out = 0.0, posY_world_out = 0.0, posX_world_imu = 0.0, posY_world_imu = 0.0;
float velX_world = 0.0, velY_world = 0.0;
float velX_world_imu = 0.0, velY_world_imu = 0.0;
float velX_robot_out = 0.0, velY_robot_out = 0.0, velX_world_out = 0.0, velY_world_out = 0.0;
float posXRefWorld = 0.0, posYRefWorld = 0.0, velXRefWorld = 0.0, velYRefWorld = 0.0;
float integralErroPosX = 0.0, integralErroPosY = 0.0, erroPosXAnt = 0.0, erroPosYAnt = 0.0;
float satErroPos = 0.30, satErroVel = 1.0;
float pitchMax = 1.0, rollMax = 1.0;

bool posXUpdate, posYUpdate;

struct Quaternion {
  float w, x, y, z;
};

Quaternion conjugate(Quaternion q);
Quaternion multiply(Quaternion q_1, Quaternion q_2);
Quaternion yawToQuaternion(float yaw);

float quaternionToYaw(Quaternion q);

// ------------------------------ Define the State and Controller parameters ------------------------------
struct State {
  double posX = 0.0;      // x position   [m]
  double velX = 0.0;      // x velocity   [m/s]
  double theta = 0.0;     // pitch angle  [rad]
  double q = 0.0;         // pitch rate   [rad/s]
  double posY = 0.0;      // y position   [m]
  double velY = 0.0;      // y velocity   [m/s]
  double phi = 0.0;       // roll angle   [rad]
  double p = 0.0;         // roll rate    [rad/s]
  double psi = 0.0;       // yaw angle    [rad]
  double r = 0.0;         // yaw rate     [rad/s]
};
State x, xRef;

double yawError = 0, yawErrorRef = 0;
double refTheta = 0.0, refPhi = 0.0;
double rollOffset = 0.0, pitchOffset = 0.0;
double tx = 0.0, ty = 0.0, tz = 0.0, rateTx = 0.0, rateTy = 0.0, rateTz = 0.0;
double k1 = -35.0/100.0, k2 = -110.0/100.0, k3 = 280.0/100.0, k4 = 40.0/100.0;      // Tune these values as necessary
double kp = 165.0/100.0, ki = 1.0/100.0, kd = 13.0/100.0;                         
double arKp = 0.0, arKi = 0.0, arKd = 0.0;
double posKp = 5.0/100.0, posKi = 0.0, posKd = 0.05/100.0;                      
double velKp = 10.0/100.0, velKi = 0.0, velKd = 0.5/100.0;                      
double kpYaw = 4.0/100.0, kiYaw = 0.0, kdYaw = 1.25/100;
// ---------------------------------------------------------------------------------------

// ------------------- IMU Functions and Variables -------------------
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
float fAcc[3], fGyro[3], fAngle[3];
// -------------------------------------------------------------------

//  ------------ Define PID controllers for roll and pitch ------------
PID rollPID(&x.phi, &tx, &xRef.phi, kp, ki, kd, DIRECT);
PID pitchPID(&x.theta, &ty, &xRef.theta, kp, ki, kd, DIRECT);
// PID yawPID(&x.psi, &tz, &xRef.psi, kpYaw, kiYaw, kdYaw, DIRECT);
PID yawPID(&yawError, &tz, &yawErrorRef, kpYaw, kiYaw, kdYaw, REVERSE);

PID rollRatePID(&x.p, &rateTx, &xRef.p, arKp, arKi, arKd, DIRECT);
PID pitchRatePID(&x.q, &rateTy, &xRef.q, arKp, arKi, arKd, DIRECT);
// PID yawRatePID();

PID posXPID(&x.posX, &xRef.velX, &xRef.posX, posKp, posKi, posKd, DIRECT);    // ty acts on X position -> Visualize 2D Model 
PID posYPID(&x.posY, &xRef.velY, &xRef.posY, posKp, posKi, posKd, DIRECT);    // tx acts on Y position 

PID velXPID(&x.velX, &refTheta, &xRef.velX, velKp, velKi, velKd, DIRECT);
PID velYPID(&x.velY, &refPhi, &xRef.velY, velKp, velKi, velKd, REVERSE);
// --------------------------------------------------------------------

// ------------------- Step Motor Objects -------------------
AccelStepper stepper1(1, MOTOR_1_STEP, MOTOR_1_DIR);
AccelStepper stepper2(1, MOTOR_2_STEP, MOTOR_2_DIR);
AccelStepper stepper3(1, MOTOR_3_STEP, MOTOR_3_DIR);
// ----------------------------------------------------------

// -------------- ESP32 to Raspberry Coms --------------
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE = 0xFF;

struct DataPacket {
    float fAngle[3];    // Angles of Attitude (Roll Pitch Yaw)
    float fGyro[3];     // Angular Velocity -> Robot Frame
    float fAcc[3];      // Linear Acceleration -> Robot "Leveled" Frame
    float vel[2];       // Linear Velocity -> Robot "Leveled" Frame
    float pos[2];       // Position -> World Frame
    float W[3];         // Angular Velocities of the planar wheels (2D Model Wheel)
};

DataPacket data;
void sendData(DataPacket data);
// -----------------------------------------------------

TaskHandle_t C0;
void loop2(void *pvParameters);
double saturate(double V, double sat);

const char* ssid = "$$$$"; // Replace with your SSID for the wifi network for OTA (Over the Air) code upload, if needed
const char* password = "$$$$"; // Replace with the password for the wifi network
bool wifiEnabled;
unsigned long wifiStartTime = 0;
const unsigned long wifiTimeout = 300000; // Wifi Timeout in milliseconds (5 minutes)

void disableWiFi();
void enableWiFiAndOTA();
void blinkLED(int times);
void velocityConversion();
Matrix<3, 3> rotationMatrix(const int& axis, const float& angle);
Matrix<4, 4> transformMatrix(const float& angle, Matrix<4> position);
Matrix<3, 3> robotToWorldRotation(const Matrix<3>& angles);

// ------------------ LQR Controller ------------------
class LQR {
  private:
    double k1, k2, k3, k4;

  public:
    // Constructor to initialize LQR gains
    LQR(double k1, double k2, double k3, double k4) {
      this->k1 = k1;
      this->k2 = k2;
      this->k3 = k3;
      this->k4 = k4;
    }

    void setGains(double k1, double k2, double k3, double k4) {
      this->k1 = k1;
      this->k2 = k2;
      this->k3 = k3;
      this->k4 = k4;
    }

    // Update function to calculate control inputs
    // State Vector x -> [x, x_dot, theta, theta_dot, y, y_dot, phi, phi_dot]
    void update(State xRef, State x, double &tx, double &ty) {
      // Calculate the error for each variable
      double erroPosX = xRef.posX - x.posX;
      double erroVelX = xRef.velX - x.velX;
      double erroTheta = xRef.theta - x.theta;
      double erroq = xRef.q - x.q;

      double erroPosY = xRef.posY - x.posY;
      double erroVelY = xRef.velY - x.velY;
      double erroPhi = xRef.phi - x.phi;
      double errop = xRef.p - x.p;

      // Anti-windup for X and Y position controllers
      // When the counter is active (greater than 0), the corresponding integrator value 
      // is gradually reduced at each iteration to prevent saturation.
      if ((erroPosXAnt < 0 && erroPosX > 0) || (erroPosXAnt > 0 && erroPosX < 0))
      {
          integralXCount = 200;
      }

      if ((erroPosYAnt < 0 && erroPosY > 0) || (erroPosYAnt > 0 && erroPosY < 0))
      {
          integralYCount = 200;
      }
      
      if (integralXCount > 0)
      {
          integralXCount--;
          integralErroPosX *= 0.98;
      }

      if (integralYCount > 0)
      {
          integralYCount--;
          integralErroPosY *= 0.98;
      }

      erroPosXAnt = erroPosX;
      erroPosYAnt = erroPosY;

      // This tries to compensate for the mass unbalance on the x-z plane (we have only one motor at the front of the robot and two at the rear)
      if (erroPosX > satErroPos*1.2) erroPosX = satErroPos*1.2;
      if (erroPosX < -satErroPos) erroPosX = -satErroPos;
      // erroPosX = saturate(erroPosX, satErroPos);
      erroPosY = saturate(erroPosY, satErroPos*1.25);

      integralErroPosX += erroPosX*sampleTime;
      integralErroPosY += erroPosY*sampleTime;

      integralErroPosX = saturate(integralErroPosX, 1.25*satErroPos);
      integralErroPosY = saturate(integralErroPosY, 1.25*satErroPos);
      
      float absErroX = fabs(erroPosX);
      float absErroY = fabs(erroPosY);

      tx = -(integralErroPosY*ki) + (erroPosY * k1) + (erroVelY * k2) + (erroPhi * k3) + (errop * k4);    
      ty = (integralErroPosX*ki) - (erroPosX * k1) - (erroVelX * k2) + (erroTheta * k3) + (erroq * k4);
    }
};

LQR lqrController(k1, k2, k3, k4);
// ----------------------------------------------------

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t start_byte;
    uint8_t cmd_type;
    float cmd_value;
    uint8_t end_byte;
};
#pragma pack(pop)

enum CommandType {
    CMD_CONTROLLER_SELECT = 0,
    CMD_KP,
    CMD_KI,
    CMD_KD,
    CMD_KP_ANGLE_RATE,
    CMD_KI_ANGLE_RATE,
    CMD_KD_ANGLE_RATE,
    CMD_KP_POS,
    CMD_KI_POS,
    CMD_KD_POS,
    CMD_KP_VEL,
    CMD_KI_VEL,
    CMD_KD_VEL,
    CMD_K1,
    CMD_K2,
    CMD_K3,
    CMD_K4,
    CMD_KP_YAW,
    CMD_KI_YAW,
    CMD_KD_YAW,
    CMD_SATURATION,
    CMD_POS_X,
    CMD_POS_Y,
    CMD_VEL_X,
    CMD_VEL_Y,
    CMD_YAW,
    CMD_POS_X_SETPOINT,
    CMD_POS_Y_SETPOINT,
    CMD_VEL_X_SETPOINT,
    CMD_VEL_Y_SETPOINT,
    CMD_ROLL_SETPOINT,
    CMD_PITCH_SETPOINT,
    CMD_YAW_SETPOINT,
    CMD_ACCEL_FILTER,
    CMD_GYRO_FILTER,
    CMD_TORQUE_FILTER,
    CMD_SAT_ERRO_POS,
    CMD_SAT_ERRO_VEL,
    CMD_ROLL_MAX,
    CMD_PITCH_MAX,
    CMD_WIFI_MODE,
    CMD_ACCEL_CALIB,
    CMD_DEBUG,
    CMD_MICRO_STEP,
    NUM_COMMANDS
};

class CommandHandler {
  public:
  void processSerial() {
    static uint8_t buffer[sizeof(CommandPacket)]; // Buffer to store incoming packet
    static size_t bufferIndex = 0;                // Track how many bytes have been read
    static bool packetStarted = false;            // Flag to indicate packet detection
  
    while (Serial.available() > 0) {
      uint8_t byte = Serial.read();
  
      if (!packetStarted) {
        // Discard bytes until START_BYTE is found
        if (byte == START_BYTE) {
          buffer[0] = byte;           // Store START_BYTE
          bufferIndex = 1;            // Reset buffer index
          packetStarted = true;       // Start reading the packet
        }
      } 
      
      else {
        // Continue reading the packet after START_BYTE
        buffer[bufferIndex++] = byte;
  
        // Check if the full packet is received
        if (bufferIndex == sizeof(CommandPacket)) {
          // Reset state for next packet
          packetStarted = false;
  
          // Cast buffer to CommandPacket for validation
          CommandPacket* packet = reinterpret_cast<CommandPacket*>(buffer);
  
          // Validate start/end bytes
          if (packet->start_byte == START_BYTE && packet->end_byte == END_BYTE) {
            handleCommand(packet->cmd_type, packet->cmd_value);
          } else {
            // Send back malformed packet for debugging
            Serial.write(buffer, sizeof(CommandPacket));
            blinkLED(10); // Indicate error
          }
        }
      }
    }
  }
  
  private:
    void handleCommand(uint8_t cmd_type, float cmd_value) {
      switch (cmd_type) {
        case CMD_CONTROLLER_SELECT:
          controller = (int) cmd_value;
          if (controller == 0 || controller == 1){
            xRef.phi = 0; xRef.theta = 0;
            tx = 0; ty = 0; tz = 0;
            wx = 0; wy = 0; wz = 0;
            outerLoopCount = 0;

            if (debug) {
              if (controller == 0) Serial.printf("Controller set to PID");
              else Serial.printf("Controller set to Position PID");
              blinkLED(1);
            }

          } else if (controller == 2 || controller == 3) {
            xRef.phi = 0; xRef.theta = 0;
            tx = 0; ty = 0; tz = 0;
            wx = 0; wy = 0; wz = 0;
            outerLoopCount = 0;

            if (debug) {
              Serial.printf("Controller set to LQR");
              blinkLED(1);
            }  
          }

          else controller = -1;       
          
          break;
            
        case CMD_KP:
          kp = cmd_value/100;
          rollPID.SetTunings(kp, ki, kd);
          pitchPID.SetTunings(kp, ki, kd);

          if (debug) {
            Serial.printf("Updated Kp to: %.4f\n", kp);
            blinkLED(1);
          }
          break;
            
        case CMD_KI:
          ki = cmd_value/100;
          rollPID.SetTunings(kp, ki, kd);
          pitchPID.SetTunings(kp, ki, kd);
          
          if (debug) {
            Serial.printf("Updated Ki to: %.4f\n", ki);
            blinkLED(1);
          }
          break;
  
        case CMD_KD:
          kd = cmd_value/100;
          rollPID.SetTunings(kp, ki, kd);
          pitchPID.SetTunings(kp, ki, kd);

          if (debug) {
            Serial.printf("Updated Kd to: %.4f\n", kd);
            blinkLED(1);
          }
          break;

        case CMD_KP_ANGLE_RATE:
          arKp = cmd_value/100;
          rollRatePID.SetTunings(arKp, arKi, arKd);
          pitchRatePID.SetTunings(arKp, arKi, arKd);

          if (debug) {
            Serial.printf("Updated Angle Rate Kp to: %.4f\n", arKp);
            blinkLED(1);
          }
          break;
            
        case CMD_KI_ANGLE_RATE:
          arKi = cmd_value/100;
          rollRatePID.SetTunings(arKp, arKi, arKd);
          pitchRatePID.SetTunings(arKp, arKi, arKd);
          
          if (debug) {
            Serial.printf("Updated Angle Rate Ki to: %.4f\n", arKi);
            blinkLED(1);
          }
          break;
  
        case CMD_KD_ANGLE_RATE:
          arKd = cmd_value/100;
          rollRatePID.SetTunings(arKp, arKi, arKd);
          pitchRatePID.SetTunings(arKp, arKi, arKd);

          if (debug) {
            Serial.printf("Updated Angle Rate Kd to: %.4f\n", arKd);
            blinkLED(1);
          }
          break;
  
        case CMD_KP_POS:
          posKp = cmd_value/100;
          posXPID.SetTunings(posKp, posKi, posKd);
          posYPID.SetTunings(posKp, posKi, posKd);

          if (debug) {
            Serial.printf("Updated Pos Kp to: %.4f\n", posKp);
            blinkLED(1);
          }
          break;
  
        case CMD_KI_POS:
          posKi = cmd_value/100;
          posXPID.SetTunings(posKp, posKi, posKd);
          posYPID.SetTunings(posKp, posKi, posKd);

          if (debug) {
            Serial.printf("Updated Pos Ki to: %.4f\n", posKi);
            blinkLED(1);
          }
          break;
  
        case CMD_KD_POS:
          posKd = cmd_value/100;
          posXPID.SetTunings(posKp, posKi, posKd);
          posYPID.SetTunings(posKp, posKi, posKd);
          
          if (debug) {
            Serial.printf("Updated Pos Kd to: %.4f\n", posKd);
            blinkLED(1);
          }
          break;

        case CMD_KP_VEL:
          velKp = cmd_value/100;
          velXPID.SetTunings(velKp, velKi, velKd);
          velYPID.SetTunings(velKp, velKi, velKd);

          if (debug) {
            Serial.printf("Updated Vel Kp to: %.4f\n", velKp);
            blinkLED(1);
          }
          break;
  
        case CMD_KI_VEL:
          velKi = cmd_value/100;
          velXPID.SetTunings(velKp, velKi, velKd);
          velYPID.SetTunings(velKp, velKi, velKd);

          if (debug) {
            Serial.printf("Updated Vel Ki to: %.4f\n", velKi);
            blinkLED(1);
          }
          break;
  
        case CMD_KD_VEL:
          velKd = cmd_value/100;
          velXPID.SetTunings(velKp, velKi, velKd);
          velYPID.SetTunings(velKp, velKi, velKd);
          
          if (debug) {
            Serial.printf("Updated Vel Kd to: %.4f\n", velKd);
            blinkLED(1);
          }
          break;
  
        case CMD_K1:
          k1 = cmd_value/100;
          lqrController.setGains(k1, k2, k3, k4);

          if (debug) {
            Serial.printf("Updated K1 to: %.4f\n", k1);
            blinkLED(1);
          }
          break;
  
        case CMD_K2:
          k2 = cmd_value/100;
          lqrController.setGains(k1, k2, k3, k4);

          if (debug) {
            Serial.printf("Updated K2 to: %.4f\n", k2);
            blinkLED(1);
          }
          break;
  
        case CMD_K3:
          k3 = cmd_value/100;
          lqrController.setGains(k1, k2, k3, k4);

          if (debug) {
            Serial.printf("Updated K3 to: %.4f\n", k3);
            blinkLED(1);
          }
          break;
  
        case CMD_K4:
          k4 = cmd_value/100;
          lqrController.setGains(k1, k2, k3, k4);
          
          if (debug) {
            Serial.printf("Updated K4 to: %.4f\n", k4);
            blinkLED(1);
          }
          break;
  
        case CMD_KP_YAW:
          if (cmd_value == 0) {
            tz = 0; wz = 0;
          }

          kpYaw = cmd_value/100;
          yawPID.SetTunings(kpYaw, kiYaw, kdYaw);

          if (debug) {
            Serial.printf("Updated Yaw Kp to: %.4f\n", kpYaw);
            blinkLED(1);
          }
          break;

        case CMD_KI_YAW:
          kiYaw = cmd_value/100;
          yawPID.SetTunings(kpYaw, kiYaw, kdYaw);

          if (debug) {
            Serial.printf("Updated Yaw Ki to: %.4f\n", kiYaw);
            blinkLED(1);
          }
          break;

        case CMD_KD_YAW:
          kdYaw = cmd_value/100;
          yawPID.SetTunings(kpYaw, kiYaw, kdYaw);

          if (debug) {
            Serial.printf("Updated Yaw Kd to: %.4f\n", kdYaw);
            blinkLED(1);
          }
          break;
  
        case CMD_SATURATION:
          sat = cmd_value*2*PI/100;
          if (debug) {
            Serial.printf("Updated Saturation to: %.4f\n", sat);
            blinkLED(1);
          }
          break;
  
        case CMD_POS_X:
          posX_world_out = cmd_value;
          posXUpdate = true;

          if (debug) {
            Serial.printf("Updated X Position to: %.4f\n", posX_world_out);
            blinkLED(1);
          }
          break;
  
        case CMD_POS_Y:
          posY_world_out = cmd_value;
          posYUpdate = true;
          
          if (debug) {
            Serial.printf("Updated Y Position to: %.4f\n", posY_world_out);
            blinkLED(1);
          }
          break;

        case CMD_VEL_X:
          velX_robot_out = cmd_value;

          if (debug) {
            Serial.printf("Updated X Velocity to: %.4f\n", velX_robot_out);
            blinkLED(1);
          }
          break;
  
        case CMD_VEL_Y:
          velY_robot_out = cmd_value;

          if (debug) {
            Serial.printf("Updated Y Velocity to: %.4f\n", velY_robot_out);
            blinkLED(1);
          }
          break;
  
        case CMD_YAW:
          x.psi = cmd_value;

          if (debug) {
            Serial.printf("Updated Yaw to: %.4f\n", x.psi);
            Serial.printf("Rotation Matrix: \n");
            Serial.println(R_yaw);
            blinkLED(1);
          }
          break;
  
        case CMD_POS_X_SETPOINT:
          posXRefWorld = cmd_value;
          if (debug) {
            Serial.printf("Updated X Position Set Point to: %.4f\n", posXRefWorld);
            blinkLED(1);
          }
          break;
  
        case CMD_POS_Y_SETPOINT:
          posYRefWorld = cmd_value;
          if (debug) {
            Serial.printf("Updated Y Position Set Point to: %.4f\n", posYRefWorld);
            blinkLED(1);
          }
          break;

        case CMD_VEL_X_SETPOINT:
          velXRefWorld = cmd_value;
          if (debug) {
            Serial.printf("Updated X Velocity Set Point to: %.4f\n", velXRefWorld);
            blinkLED(1);
          }
          break;
  
        case CMD_VEL_Y_SETPOINT:
          velYRefWorld = cmd_value;
          if (debug) {
            Serial.printf("Updated Y Velocity Set Point to: %.4f\n", velYRefWorld);
            blinkLED(1);
          }
        break;
  
        case CMD_ROLL_SETPOINT:
          rollOffset = cmd_value*DEG_TO_RAD;
          if (debug) {
            Serial.printf("Updated Roll Set Point to: %.4f degs\n", cmd_value);
            blinkLED(1);
          }
          break;
  
        case CMD_PITCH_SETPOINT:
          pitchOffset = cmd_value*DEG_TO_RAD;
          if (debug) {
            Serial.printf("Updated Pitch Set Point to: %.4f degs\n", cmd_value);
            blinkLED(1);
          }
          break;
  
        case CMD_YAW_SETPOINT:
          xRef.psi = cmd_value*DEG_TO_RAD;
          if (debug) {
            Serial.printf("Updated Yaw Set Point to: %.4f degs\n", cmd_value);
            blinkLED(1);
          }
          break;

        case CMD_ACCEL_FILTER:
          if (0 <= cmd_value && cmd_value <= 1) { 
            alphaAccel = cmd_value;
            if (debug) Serial.printf("Alpha value for acceleration filter updated to: %.4f\n", alphaAccel);
          }
          else {
            if (debug) Serial.printf("Invalid alpha value: %.4f\n", cmd_value);
          }
          break;

        case CMD_GYRO_FILTER:
          if (0 <= cmd_value && cmd_value <= 1) {
            alphaGyro = cmd_value;
            if (debug) Serial.printf("Alpha value for gyro filter updated to: %.4f\n", alphaGyro);
          }
          else {
            if (debug) Serial.printf("Invalid alpha value: %.4f\n", cmd_value);
          }
          break;

        case CMD_TORQUE_FILTER:
          if (0 <= cmd_value && cmd_value <= 1) {
            alphaTorque = cmd_value;
            if (debug) Serial.printf("Alpha value for torque filter updated to: %.4f\n", alphaTorque);
          }
          else {
            if (debug) Serial.printf("Invalid alpha value: %.4f\n", cmd_value);
          }
          break;

        case CMD_SAT_ERRO_POS:
          if (cmd_value >= 0) {
            satErroPos = cmd_value;

            if (debug) Serial.printf("Position error saturation updated to: %.4f\n", satErroPos);
          }
          else {
            if (debug) Serial.printf("Invalid saturation value: %.4f\n", cmd_value);
          }
          break;

        case CMD_SAT_ERRO_VEL:
          if (cmd_value >= 0) {
            satErroVel = cmd_value;
            if (debug) Serial.printf("Velocity error saturation updated to: %.4f\n", satErroVel);
          }
          else {
            if (debug) Serial.printf("Invalid saturation value: %.4f\n", cmd_value);
          }
          break;

        case CMD_ROLL_MAX:
          if (cmd_value >= 0) {
            rollMax = cmd_value;

            if (debug) Serial.printf("Maximum Roll Angle updated to: %.4f\n", rollMax);
          }
          else {
            if (debug) Serial.printf("Invalid value: %.4f\n", cmd_value);
          }
          break;

        case CMD_PITCH_MAX:
          if (cmd_value >= 0) {
            pitchMax = cmd_value;

            if (debug) Serial.printf("Maximum Pitch Angle updated to: %.4f\n", pitchMax);
          }
          else {
            if (debug) Serial.printf("Invalid value: %.4f\n", cmd_value);
          }
          break;
          
        case CMD_WIFI_MODE:
          status = (int) cmd_value;
          if (status == 0){
            disableWiFi();
            if (debug) blinkLED(1);
          } else {
            enableWiFiAndOTA();
            if (debug) blinkLED(1);
          }
          break;

        case CMD_ACCEL_CALIB:
          calibDuration = (int) cmd_value;
          if (calibDuration > 0) {
            calibDuration *= 1000;
            digitalWrite(LED_OB, LOW);
            WitStartAccCali();
            delay(calibDuration);
            WitStopAccCali();
            digitalWrite(LED_OB, HIGH);
          }
          break;

        case CMD_DEBUG:
          if ((int) cmd_value == 0) {
            debug = false;
            loopCount = 0;
          } else {
            debug = true;
            loopCount = 0;
          }
          break;

        case CMD_MICRO_STEP:
          /* Commented out as the current electronic board does not allow to change the micro step

          if ((int) cmd_value == 0) {
            if (debug) Serial.print("Micro Step set to 1");

            digitalWrite(MS1, LOW);
            digitalWrite(MS2, LOW);
            digitalWrite(MS3, LOW);

            stepRes = 1;
            stepsPerRev = 200*stepRes;

            stepper1.setMaxSpeed(stepsPerRev*5.0);
            stepper2.setMaxSpeed(stepsPerRev*5.0);
            stepper3.setMaxSpeed(stepsPerRev*5.0);
          }

          else if ((int) cmd_value == 1) {
            if (debug) Serial.print("Micro Step set to 1/2");

            digitalWrite(MS1, HIGH);
            digitalWrite(MS2, LOW);
            digitalWrite(MS3, LOW);

            stepRes = 2;
            stepsPerRev = 200*stepRes;

            stepper1.setMaxSpeed(stepsPerRev*5.0);
            stepper2.setMaxSpeed(stepsPerRev*5.0);
            stepper3.setMaxSpeed(stepsPerRev*5.0);
          }

          else if ((int) cmd_value == 2) {
            if (debug) Serial.print("Micro Step set to 1/4");

            digitalWrite(MS1, LOW);
            digitalWrite(MS2, HIGH);
            digitalWrite(MS3, LOW);

            stepRes = 4;
            stepsPerRev = 200*stepRes;

            stepper1.setMaxSpeed(stepsPerRev*5.0);
            stepper2.setMaxSpeed(stepsPerRev*5.0);
            stepper3.setMaxSpeed(stepsPerRev*5.0);
          }

          else if ((int) cmd_value == 3) {
            if (debug) Serial.print("Micro Step set to 1/8");

            digitalWrite(MS1, HIGH);
            digitalWrite(MS2, HIGH);
            digitalWrite(MS3, LOW);

            stepRes = 8;
            stepsPerRev = 200*stepRes;

            stepper1.setMaxSpeed(stepsPerRev*5.0);
            stepper2.setMaxSpeed(stepsPerRev*5.0);
            stepper3.setMaxSpeed(stepsPerRev*5.0);
          }

          else if ((int) cmd_value == 4) {
            if (debug) Serial.print("Micro Step set to 1/16");

            digitalWrite(MS1, HIGH);
            digitalWrite(MS2, HIGH);
            digitalWrite(MS3, HIGH);

            stepRes = 16;
            stepsPerRev = 200*stepRes;

            stepper1.setMaxSpeed(stepsPerRev*5.0);
            stepper2.setMaxSpeed(stepsPerRev*5.0);
            stepper3.setMaxSpeed(stepsPerRev*5.0);
          } 
          */
          break;          

        default:
          if (debug) {
            Serial.printf("Unknown Command");
            blinkLED(5);
          }
          break;
      }
    }
  }; CommandHandler cmdHandler;

void setup() {
  pinMode(LED_OB, OUTPUT);
  digitalWrite(LED_OB, LOW);
  delay(5000);
  Serial.begin(115200);
  Serial.println("Booting");

  // ------------------ WIFI Setup ------------------
  // Disable Wi-Fi on startup to prevent interference with loop timing.
  // Wi-Fi will only be enabled when explicitly requested, for example, via an external command.
  WiFi.mode(WIFI_OFF);
  // ------------------------------------------------

  // ------------------------------------ IMU Setup ------------------------------------
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  Serial1.flush();
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);

  Serial.println("Accelerometer Calibration Starting: Keep the robot stationary for 30 seconds...");
  WitStartAccCali();
  delay(5000);
  WitStopAccCali();
  Serial.println("Accelerometer Calibration Completed");
  // -----------------------------------------------------------------------------------

  // ------------------ PID Setup ------------------
  // Roll Pitch Yaw
  rollPID.SetOutputLimits(-satXY, satXY);
  pitchPID.SetOutputLimits(-satXY, satXY);
  yawPID.SetOutputLimits(-satXY, satXY);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  // Angular Rates
  rollRatePID.SetOutputLimits(-satXY, satXY);
  pitchRatePID.SetOutputLimits(-satXY, satXY);
  rollRatePID.SetMode(AUTOMATIC);
  pitchRatePID.SetMode(AUTOMATIC);

  // Position
  posXPID.SetOutputLimits(-1.0, 1.0);
  posYPID.SetOutputLimits(-1.0, 1.0);
  posXPID.SetMode(AUTOMATIC);
  posYPID.SetMode(AUTOMATIC);

  // Velocity
  velXPID.SetOutputLimits(-15*DEG_TO_RAD, 15*DEG_TO_RAD);
  velYPID.SetOutputLimits(-15*DEG_TO_RAD, 15*DEG_TO_RAD);
  velXPID.SetMode(AUTOMATIC);
  velYPID.SetMode(AUTOMATIC);
  // -----------------------------------------------

  // ------------------ Motor Setup ------------------
  // pinMode(MS1, OUTPUT);
  // pinMode(MS2, OUTPUT);
  // pinMode(MS3, OUTPUT);

  // digitalWrite(MS1, HIGH);
  // digitalWrite(MS2, HIGH);
  // digitalWrite(MS3, HIGH);

  stepper1.setMaxSpeed(stepsPerRev*5.0);
  stepper2.setMaxSpeed(stepsPerRev*5.0);
  stepper3.setMaxSpeed(stepsPerRev*5.0);
  // -------------------------------------------------

  disableCore0WDT();

  xTaskCreatePinnedToCore(
    loop2,                    
    "Core_0",                 
    10000,                   
    NULL,                     
    1,                       
    &C0,                      
    0);                       

  pos_world = {0, 0, 0, 1};
  R_yaw = transformMatrix(x.psi, pos_world);

  digitalWrite(LED_OB, HIGH);
}

void loop() {
  for(;;) {
    /*
      Motor angular velocity unit is -> Wn [rad/s]
      setSpeed input is [steps/s]
      We must convert from [rad/s] to [steps/s]
      As we are using micro-stepping, one revolution (2pi) has 6400 steps. (e.g. microstepping set to 32, then 200*32)
      We have to multiply V by 6400/2pi [steps/rad].
      1/(2pi) = 0.15915
    */   
    if (controller == -1) {
      stepper1.setSpeed(stepsPerRev * 0.15915 * k1);
      stepper2.setSpeed(stepsPerRev * 0.15915 * k2);
      stepper3.setSpeed(stepsPerRev * 0.15915 * k3);
    }

    else {
      stepper1.setSpeed(stepsPerRev * 0.15915 * W1);
      stepper2.setSpeed(stepsPerRev * 0.15915 * W2);
      stepper3.setSpeed(stepsPerRev * 0.15915 * W3);
    }

    // Run motors
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
  }
}

void loop2( void *pvParameters ) {
  for(;;) {     
    timeStart = micros(); 

    cmdHandler.processSerial();

    // Handle OTA updates if Wi-Fi is enabled
    if (wifiEnabled) {
      ArduinoOTA.handle();

      // Automatically disable Wi-Fi after a timeout
      if (millis() - wifiStartTime > wifiTimeout) {
        disableWiFi();
      }
    }

    while (Serial1.available())
    {
      WitSerialDataIn(Serial1.read());
    }

    if(s_cDataUpdate)
    {
      for(int i = 0; i < 3; i++)
      {
        fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
        fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
        fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
      }
      
      if(s_cDataUpdate & ACC_UPDATE); s_cDataUpdate &= ~ACC_UPDATE;		
      if(s_cDataUpdate & GYRO_UPDATE); s_cDataUpdate &= ~GYRO_UPDATE;
      if(s_cDataUpdate & ANGLE_UPDATE); s_cDataUpdate &= ~ANGLE_UPDATE;			
      if(s_cDataUpdate & MAG_UPDATE); s_cDataUpdate &= ~MAG_UPDATE;
      
      s_cDataUpdate = 0;
    }

    x.phi = fAngle[0]*DEG_TO_RAD;                             // [rad]
    x.theta = fAngle[1]*DEG_TO_RAD;                           // [rad]
    // x.psi = fAngle[2]*DEG_TO_RAD;                          // [rad] Yaw info is comming from Outside 
    
    x.p = fGyro[0]*DEG_TO_RAD;                                // [rad/s]
    x.q = fGyro[1]*DEG_TO_RAD;                                // [rad/s]
    x.r = fGyro[2]*DEG_TO_RAD;                                // [rad/s]
   
    x.p = lowPassFilter(x.p, alphaGyro, &gyroXFilter);
    x.q = lowPassFilter(x.q, alphaGyro, &gyroYFilter);
    x.r = lowPassFilter(x.r, alphaGyro, &gyroZFilter);

    Matrix<3> angles = {(float)x.phi, (float)x.theta, (float)x.psi};

    // We are transforming the Robot's acceleration from body frame to inertial frame
    Matrix<3> acc_robot = {(float)(fAcc[0]*g), (float)(fAcc[1]*g), (float)(fAcc[2]*g)};  // [m/s^2]
    Matrix<3, 3> R_robot_to_world = robotToWorldRotation(angles);
    Matrix<3> acc_world = (R_robot_to_world*acc_robot) - gravity_world;

    accX_world = lowPassFilter(acc_world(0), alphaAccel, &accXFilter);
    accY_world = lowPassFilter(acc_world(1), alphaAccel, &accYFilter);
    accZ_world = lowPassFilter(acc_world(2), alphaAccel, &accZFilter);
    
    vel_robot_odom = {float (-wy*0.07/2), float (wx*0.07/2), 0};
    vel_world_odom = robotToWorldRotation(Matrix<3> {0, 0, (float) x.psi})*vel_robot_odom;

    velX_world_imu = velX_world + accX_world*sampleTime;
    velY_world_imu = velY_world + accY_world*sampleTime;
    vel_world_imu = {velX_world_imu, velY_world_imu, 0, 0};

    velX_world = 0.004*vel_world_odom(0) + 0.996*velX_world_imu;
    velY_world = 0.004*vel_world_odom(1) + 0.996*velY_world_imu;

    vel_world = {velX_world, velY_world, 0, 0};
    vel_robot = R_yaw*vel_world;
    
    x.velX = vel_robot(0); x.velY = vel_robot(1);

    posX_world_imu = posX_world + velX_world_imu*sampleTime;
    posY_world_imu = posY_world + velY_world_imu*sampleTime;

    posX_world = 0.99*posX_world_out + 0.01*posX_world_imu; 
    posY_world = 0.99*posY_world_out + 0.01*posY_world_imu;
    
    pos_world = {posX_world, posY_world, 0, 1};

    // Transforming Acceleration, Position and Velocity to the "Leveled" robot's frame, 
    // which is a frame fixed to the Robot's body and that has its X-Y plane aligned to the Inertial frame X-Y plane.
    // Note that for Position, we're also translating it to the robot's frame, which means the position should be 0 at any time, 
    // as we're measuring the robot's position related to it's own frame of reference that has it's origint at the robot's C.G
    R_yaw = transformMatrix(x.psi, pos_world);    

    pos_robot = R_yaw*pos_world;
    x.posX = pos_robot(0); x.posY = pos_robot(1);

    Matrix<4> acc_robot_leveled = R_yaw * Matrix<4> {acc_world(0), acc_world(1), acc_world(2), 0};
    accX_robot = acc_robot_leveled(0); accY_robot = acc_robot_leveled(1); accZ_robot = acc_robot_leveled(2); 

    Matrix<4> pos_ref_world = {posXRefWorld, posYRefWorld, 0, 1};
    Matrix<4> pos_ref_robot = R_yaw * pos_ref_world;
    xRef.posX = pos_ref_robot(0); xRef.posY = pos_ref_robot(1);    

    Quaternion qSP = yawToQuaternion(xRef.psi);
    Quaternion qCur = yawToQuaternion(x.psi);
    Quaternion qDelta = multiply(qSP, conjugate(qCur));

    yawError = quaternionToYaw(qDelta);
    if ((yawError*RAD_TO_DEG) > 15){
      yawError = 15*DEG_TO_RAD;
    }

    if ((yawError*RAD_TO_DEG) < -15){
      yawError = -15*DEG_TO_RAD;
    }
    
    if (controller == 0)
    {
      // Roll Pitch Yaw
      rollPID.SetSampleTime((int)(sampleTime*1000));      // Input is [ms], converting from [s] to [ms]
      pitchPID.SetSampleTime((int)(sampleTime*1000));     
      yawPID.SetSampleTime((int)(sampleTime*1000));

      rollRatePID.SetSampleTime((int)(sampleTime*100));
      pitchRatePID.SetSampleTime((int)(sampleTime*100));

      xRef.phi = rollOffset;
      xRef.theta = pitchOffset;

      rollPID.Compute();
      pitchPID.Compute();
      yawPID.Compute();

      rollRatePID.Compute();
      pitchRatePID.Compute();

      tx = lowPassFilter(tx, alphaTorque, &txFilter);
      ty = lowPassFilter(ty, alphaTorque, &tyFilter);

      wx += tx + rateTx; //*47619.047*(1/controlRate);         // This is 2D Wheel x Angular Velocity
      wy += ty + rateTy; //*47619.047*(1/controlRate);         // This is 2D Wheel Y Angular Velocity
      wz += tz; //*47619.047*(1/controlRate);                  // This is 2D Wheel Z Angular Velocity

      velocityConversion();
    }

    else if (controller == 1) {
      if (outerLoopCount == 0)
      {
        // The Outer Control Loop should run at a lower frequency since position updates are received at 30 Hz
        posXPID.SetSampleTime((int)(sampleTime*1000/3));
        posYPID.SetSampleTime((int)(sampleTime*1000/3));
        
        posXPID.Compute();
        posYPID.Compute();

        xRef.velX = saturate(xRef.velX, 0.1);
        xRef.velY = saturate(xRef.velY, 0.1);

        velXPID.SetSampleTime((int)(sampleTime*1000/3));
        velYPID.SetSampleTime((int)(sampleTime*1000/3));

        velXPID.Compute();
        velYPID.Compute();
        
        refPhi = saturate(refPhi, rollMax*DEG_TO_RAD);
        refTheta = saturate(refTheta, pitchMax*DEG_TO_RAD);        
        
        xRef.phi = refPhi + rollOffset;
        xRef.theta = refTheta + pitchOffset;

        outerLoopCount++;
      }

      else {
        outerLoopCount++;
        if (outerLoopCount == 3) outerLoopCount = 0;        
      }       

      rollPID.SetSampleTime((int)(sampleTime*1000));
      pitchPID.SetSampleTime((int)(sampleTime*1000));     
      yawPID.SetSampleTime((int)(sampleTime*1000));

      rollPID.Compute();
      pitchPID.Compute();
      yawPID.Compute();

      rollRatePID.Compute();
      pitchRatePID.Compute();

      tx = lowPassFilter(tx, alphaTorque, &txFilter);
      ty = lowPassFilter(ty, alphaTorque, &tyFilter);

      wx += tx + rateTx; //*47619.047*(1/controlRate);         // This is 2D Wheel x Angular Velocity
      wy += ty + rateTy; //*47619.047*(1/controlRate);         // This is 2D Wheel Y Angular Velocity
      wz += tz; //*47619.047*(1/controlRate);                  // This is 2D Wheel Z Angular Velocity

      velocityConversion();
    }

    else if (controller == 2)
    {
      Matrix<4> vel_ref_world = {velXRefWorld, velYRefWorld, 0, 0};
      Matrix<4> vel_ref_robot = R_yaw * vel_ref_world;
      xRef.velX = vel_ref_robot(0); xRef.velY = vel_ref_robot(1);

      xRef.phi = rollOffset;
      xRef.theta = pitchOffset;

      lqrController.setGains(k1, k2, k3, k4);
      lqrController.update(xRef, x, tx, ty);

      tx = lowPassFilter(tx, alphaTorque, &txFilter);
      ty = lowPassFilter(ty, alphaTorque, &tyFilter);

      // xRef.psi = atan2(xRef.posY, xRef.posX);

      yawPID.SetSampleTime((int)(sampleTime*1000));
      yawPID.Compute();

      wx += (float) tx; //*47619.047*(1/controlRate); // This is 2D Wheel x Angular Velocity
      wy += (float) ty; //*47619.047*(1/controlRate); // This is 2D Wheel Y Angular Velocity
      wz += (float) tz; //*47619.047*(1/controlRate); // This is 2D Wheel Z Angular Velocity

      velocityConversion();
    }

    else if (controller == 3) {
      if (outerLoopCount == 0)
      {
        // The Outer Control Loop should run at a lower frequency since position updates are received at 30 Hz
        posXPID.SetSampleTime((int)(sampleTime*1000/3));
        posYPID.SetSampleTime((int)(sampleTime*1000/3));
        
        posXPID.Compute();
        posYPID.Compute();

        xRef.velX = saturate(xRef.velX, 0.1);
        xRef.velY = saturate(xRef.velY, 0.1);

        velXPID.SetSampleTime((int)(sampleTime*1000/3));
        velYPID.SetSampleTime((int)(sampleTime*1000/3));

        velXPID.Compute();
        velYPID.Compute();
        
        refPhi = saturate(refPhi, rollMax*DEG_TO_RAD);
        refTheta = saturate(refTheta, pitchMax*DEG_TO_RAD);        
        
        xRef.phi = refPhi + rollOffset;
        xRef.theta = refTheta + pitchOffset;

        outerLoopCount++;
      }

      else {
        outerLoopCount++;
        if (outerLoopCount == 3) outerLoopCount = 0;        
      }

      lqrController.setGains(k1, k2, k3, k4);
      lqrController.update(xRef, x, tx, ty);

      tx = lowPassFilter(tx, alphaTorque, &txFilter);
      ty = lowPassFilter(ty, alphaTorque, &tyFilter);

      yawPID.SetSampleTime((int)(sampleTime*1000));
      yawPID.Compute();

      wx += (float) tx; //*47619.047*(1/controlRate); // This is 2D Wheel x Angular Velocity
      wy += (float) ty; //*47619.047*(1/controlRate); // This is 2D Wheel Y Angular Velocity
      wz += (float) tz; //*47619.047*(1/controlRate); // This is 2D Wheel Z Angular Velocity

      velocityConversion();
    }
    
    // this ensures loop frequency is steady and close to 200Hz
    while((micros() - timeStart) < 4700) 
    {
      delayMicroseconds(100);
    }

    if (debug)
    {
      loopCount += 1;

      if (loopCount == 100) // Printing data every 100th loop run
      {
        char buffer[1024];
        int len = snprintf(buffer, sizeof(buffer),
            "\nRoll: %.2f | Pitch: %.2f | Yaw: %.2f \n"
            "Ref Roll: %.2f | Ref Pitch: %.2f \n"
            "GyroX: %.2f | GyroY: %.2f | GyroZ: %.2f \n"
            "AccX: %.2f | AccY: %.2f | AccZ: %.2f \n"
            "PosX: %.2f | PosY: %.2f \n"
            "RefPosX: %.2f | RefPosY: %.2f \n"
            "VelX: %.2f | VelY: %.2f\n"
            "RefVelX: %.2f | RefVelY: %.2f \n"
            "Wx: %.2f | Wy: %.2f | Wz: %.2f\n",
            (float) (x.phi*RAD_TO_DEG), (float) (x.theta*RAD_TO_DEG), (float) (x.psi*RAD_TO_DEG),
            xRef.phi*RAD_TO_DEG, xRef.theta*RAD_TO_DEG,
            x.p*RAD_TO_DEG, x.q*RAD_TO_DEG, x.r*RAD_TO_DEG,
            accX_world, accY_world, accZ_world,
            x.posX, x.posY,
            xRef.posX, xRef.posY,
            x.velX, x.velY,
            xRef.velX, xRef.velY,  
            wx, wy, wz);
        // Timing after formatting
        long dt = micros() - timeStart;
        if (dt != 0) {
          controlRate = 1e6 / dt;
          sampleTime = 1 / controlRate;
        } else {
          controlRate = 0;
          sampleTime = 1;
        }

        // Append control rate
        len += snprintf(buffer + len, sizeof(buffer) - len,
                        "Control Rate: %.2f \n", controlRate);
        Serial.println(buffer);

        Serial.printf("Vel X Ref: %.2f | Vel Y Ref: %.2f\n", xRef.velX, xRef.velY);
        Serial.printf("Phi Ref: %.2f | Theta Ref: %.2f\n", xRef.phi*RAD_TO_DEG, xRef.theta*RAD_TO_DEG);
        Serial.print("Rotation Matrix: \n");
        Serial.println(R_yaw);

        loopCount = 0;
      }
    }
    
    else
    {
      data = {{(float) (x.phi), (float) (x.theta), (float) (x.psi)},  //  [rad]
              {(float) (x.p), (float) (x.q), (float) (x.r)},          //  [rad/s] Robot's Frame
              // {(float) fAcc[0]*g, (float) fAcc[1]*g, (float) fAcc[2]*g},
              {accX_robot, accY_robot, accZ_robot},                   //  [m/s^2] Robot's Frame
              // {(float) (-wy*0.07/2), (float) (wx*0.07/2)},
              {(float) x.velX, (float) x.velY},   //  [m/s]   Robot's Frame
              {(float) posX_world, (float) posY_world},               //  [m]     World Frame
              {(float) wx, (float) wy, (float) wz},                   //  [rad/s]
            };
      
      loopCount += 1;

      // if (loopCount == 4) // Sending data every 4th loop run, this saves processing on the RPi Side
      if (loopCount == 1)
      {
        sendData(data);
        loopCount = 0;
      }
    }
  }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}

static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
  for(i = 0; i < uiRegNum; i++)
  {
    switch(uiReg)
    {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

void velocityConversion(){
  wx = saturate(wx, sat);
  wy = saturate(wy, sat);
  wz = saturate(wz, sat);

  W1 = (0.333)*(2.8284*wx - wz);
  W2 = (0.333)*(-1.4142*wx + 2.4495*wy - wz);
  W3 = -(0.333)*(1.4142*wx + 2.4495*wy + wz);
}

double saturate(double V, double sat){
    double Vsat;
    if (sat <= 0){
      rollPID.outputSum = 0; pitchPID.outputSum = 0;
      posXPID.outputSum = 0; posYPID.outputSum = 0;

      tx = 0; ty = 0; tz = 0;
      wx = 0; wy = 0; wz = 0; 
      
      return 0;
    } 
    
    else{
      Vsat = min(max(V, -sat), sat);

      return Vsat;
    }
}

void enableWiFiAndOTA() {
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();

  // Wait for connection (timeout after 10 seconds)
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected!");
    ArduinoOTA.begin();
    wifiEnabled = true;
    wifiStartTime = millis();
  } 
  
  else {
    Serial.println("Wi-Fi connection failed.");
  }
}

// Function to disable Wi-Fi
void disableWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  Serial.println("Wi-Fi disabled.");
}

void sendData(DataPacket data) {
  Serial.write(START_BYTE);                                   // Start marker
  Serial.write((uint8_t*)&data, sizeof(data));                // Data
  controlRate = 1e6/(micros() - timeStart);
  sampleTime = 1/controlRate;
  Serial.write((uint8_t*)&controlRate, sizeof(controlRate));  // Control Rate
  Serial.write(END_BYTE);                                     // End marker
  Serial.flush();
}

void blinkLED(int times) {
  for(int i = 0; i < times; i++) {
      digitalWrite(LED_OB, LOW);
      delay(200);
      digitalWrite(LED_OB, HIGH);
      delay(200);
  }
}

float lowPassFilter(float rawValue, float alpha, FilterState* state) {
  if (state->firstRun) {
    state->filteredValue = rawValue;
    state->firstRun = false;
  } else {
    state->filteredValue = (alpha * rawValue) + ((1 - alpha) * state->filteredValue);
  }
  return state->filteredValue;
}

Matrix<3, 3> rotationMatrix(const int& axis, const float& angle) {
  if (axis == 1) {
    Matrix<3, 3> R = {1,           0,           0,
                      0,  cos(angle),  -sin(angle),
                      0,  sin(angle),   cos(angle)};
    return R;
  }

  else if (axis == 2) {
    Matrix<3, 3> R = { cos(angle), 0,  sin(angle),
                                0, 1,           0,
                      -sin(angle), 0,  cos(angle)};
    return R;
  }

  else {
    Matrix<3, 3> R = {cos(angle),  -sin(angle),    0,
                      sin(angle),   cos(angle),    0,
                                0,           0,    1};
    return R;
  }
}

// Function to generate the transformation matrix from robot frame to world frame
Matrix<3, 3> robotToWorldRotation(const Matrix<3>& angles) {
  float roll = angles(0);
  float pitch = angles(1);
  float yaw = angles(2);

  // Define rotation matrices for roll, pitch, and yaw
  Matrix<3, 3> R_roll = rotationMatrix(1, roll);
  Matrix<3, 3> R_pitch = rotationMatrix(2, pitch);
  Matrix<3, 3> R_yaw = rotationMatrix(3, yaw);

  // Combine rotation matrices (R_yaw * R_pitch * R_roll)
  Matrix<3, 3> R_combined = R_yaw * R_pitch * R_roll;   // Rotates from World Frame to Robot Frame

  return R_combined;
}

Matrix<4, 4> transformMatrix(const float& angle, Matrix<4> position) {
    Matrix<4, 4> R = {cos(angle), -sin(angle),    0, position(0),
                      sin(angle),  cos(angle),    0, position(1),
                                0,          0,    1, position(2),
                                0,          0,    0,          1};

    bool is_nonsingular = Invert(R);
    
    if (is_nonsingular) {
      return R;
    }

    else {
      R = 0;
      return R;
    }      
}

// Compute the conjugate of a quaternion
Quaternion conjugate(Quaternion q) {
  q.x *= -1;
  q.y *= -1;
  q.z *= -1;
  return q;
}

// Extract yaw from quaternion (assuming Z-only rotation)
float quaternionToYaw(Quaternion q) {
  return atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
}

// Convert a yaw angle (in radians) to a quaternion (Z-axis rotation only)
Quaternion yawToQuaternion(float yaw) {
  Quaternion q;
  float halfYaw = yaw * 0.5;
  q.w = cos(halfYaw);
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(halfYaw);
  return q;
}

// Multiply two quaternions: q1 * q2
Quaternion multiply(Quaternion q_1, Quaternion q_2) {
  Quaternion result;
  result.w = q_1.w*q_2.w - q_1.x*q_2.x - q_1.y*q_2.y - q_1.z*q_2.z;
  result.x = q_1.w*q_2.x + q_1.x*q_2.w + q_1.y*q_2.z - q_1.z*q_2.y;
  result.y = q_1.w*q_2.y - q_1.x*q_2.z + q_1.y*q_2.w + q_1.z*q_2.x;
  result.z = q_1.w*q_2.z + q_1.x*q_2.y - q_1.y*q_2.x + q_1.z*q_2.w;
  return result;
}