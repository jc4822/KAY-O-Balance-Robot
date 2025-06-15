#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <step.h> 

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;

const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

const int TOGGLE_PIN        = 32;


const int PRINT_INTERVAL_MS = 500;
const int LOOP_INTERVAL_MS  = 10;
const float LOOP_INTERVAL_S = LOOP_INTERVAL_MS / 1000.0f;
const int STEPPER_INTERVAL_US = 20;

const float VREF = 4.096;
const float GRAVITY_ACCELERATION = 9.81f;


float MPU_GYRO_Y_OFFSET = -0.068354f;


float Kp = 1500.0f;  
float Ki = 0.0f; 
float Kd = 1.0f;
float OBSERVED_UPRIGHT_OFFSET_DEG = 0.0f;
float ANGLE_CALIBRATION_OFFSET_RAD = 0.0f;

float current_angle = 0.0f;
float angle_setpoint = 0.0f;

float pid_error = 0.0f;
float pid_previous_error = 0.0f;
float pid_integral = 0.0f;
float pid_derivative = 0.0f;
float pid_output = 0.0f;

const float MAX_PID_INTEGRAL = 150.0f;

float alpha = 0.98f;


float rotation_speed = 0.0f;

const float Robot_Backward_Speed = 0.9f * DEG_TO_RAD;
const float Robot_Forward_Speed = -0.9f * DEG_TO_RAD;
const float Rotation_Rate = 1.0f;

const float TILT_LIMIT_DEG = 15.0f;



unsigned long loopTimer = 0;
unsigned long printTimer = 0;
const int TELEMETERY_INTERVAL_MS = 100;
unsigned long telemetryTimer = 0;

float pos_x = 0.0f;             // X轴位置（米）
float pos_y = 0.0f;             // Y轴位置（米）
float yaw_angle = 0.0f;         // 偏航角（度）
float prev_yaw_angle = 0.0f;    // 上一周期的偏航角
unsigned long last_position_update = 0; // 上次位置更新时间

int32_t  last_steps1 = 0;
int32_t  last_steps2 = 0;


ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;


step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);



bool TimerHandler(void * timerNo);
uint16_t readADC(uint8_t channel);
void calculateTiltAngle();
void computePID();
void handleSerialInput();
void applyMotorSpeeds();
void performGyroCalibration();


bool TimerHandler(void * timerNo) {
    static bool toggle = false;
    step1.runStepper();
    step2.runStepper();
    digitalWrite(TOGGLE_PIN, toggle);
    toggle = !toggle;
    return true;
}


uint16_t readADC(uint8_t channel) {
    uint8_t TX0 = 0x06 | (channel >> 2);
    uint8_t TX1_B = (channel & 0x03) << 6;
    digitalWrite(ADC_CS_PIN, LOW);
    SPI.transfer(TX0);
    uint8_t RX0 = SPI.transfer(TX1_B);
    uint8_t RX1_B = SPI.transfer(0x00);
    digitalWrite(ADC_CS_PIN, HIGH);
    uint16_t result = ((RX0 & 0x0F) << 8) | RX1_B;
    return result;
}


void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } 

    pinMode(TOGGLE_PIN, OUTPUT);

    Wire.begin(); 

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip. Halting.");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    Serial.println("\nAUTO-CALIBRATING ANGLE OFFSET:");
    Serial.println("Ensure robot is PERFECTLY STILL and UPRIGHT for the next few seconds...");
    delay(2000);

    const int numReadingsForCalibration = 350;
    const int targetReadingIndex = 350;

    //Calibration:
    for (int i = 1; i <= numReadingsForCalibration; ++i) {
        calculateTiltAngle();
        delay(10);

        if (i == targetReadingIndex) {
            
            
            ANGLE_CALIBRATION_OFFSET_RAD = current_angle; 
            OBSERVED_UPRIGHT_OFFSET_DEG = current_angle * RAD_TO_DEG;

            Serial.print("Reading #"); Serial.print(i);
            Serial.print(": Using this angle for offset. Offset (rad): "); Serial.print(ANGLE_CALIBRATION_OFFSET_RAD, 4);
            Serial.print(", Offset (deg): "); Serial.println(OBSERVED_UPRIGHT_OFFSET_DEG, 2);
            break;
        } else if (i > targetReadingIndex) {
        
        }
        if (i >= targetReadingIndex && OBSERVED_UPRIGHT_OFFSET_DEG != 0.0f) {
            
        }
    }
    
    
    if (ANGLE_CALIBRATION_OFFSET_RAD == 0.0f && OBSERVED_UPRIGHT_OFFSET_DEG == 0.0f && current_angle != 0.0f) {
        Serial.println("WARNING: Auto-calibration might not have captured a reading. Using last known current_angle.");
        ANGLE_CALIBRATION_OFFSET_RAD = current_angle;
        OBSERVED_UPRIGHT_OFFSET_DEG = current_angle * RAD_TO_DEG;
    }

    Serial.println("Angle offset auto-calibration complete.");
    Serial.println("Calibrated Offset (DEG): " + String(OBSERVED_UPRIGHT_OFFSET_DEG, 2));
    Serial.println("Calibrated Offset (RAD): " + String(ANGLE_CALIBRATION_OFFSET_RAD, 4));
    Serial.println("Gyro Y (MPU) Offset remains: " + String(MPU_GYRO_Y_OFFSET, 6));
    delay(1000);
    
   //performGyroCalibration();
    Serial.println("Gyro Y offset is currently: " + String(MPU_GYRO_Y_OFFSET, 4));
    Serial.println("Ensure MPU_GYRO_Y_OFFSET is accurately calibrated!");
    delay(2000); 

    Serial.println("Initializing Stepper Motor Interrupt...");
    if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
        Serial.println("Failed to start stepper interrupt. Halting.");
        while (1) { delay(10); }
    }
    Serial.println("Stepper Interrupt Initialized.");
    Serial.println("Telemetry data format: {\"a\": angle, \"x\": pos_x, \"y\": pos_y");

   
    step1.setAccelerationRad(40.0); 
    step2.setAccelerationRad(40.0); 

    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW); 

   
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

    Serial.println("\nSetup Complete. Robot starting in Balance-Only mode.");
    Serial.println("Commands (ensure Serial Monitor is set to 115200 baud and sends newline/carriage return):");
    Serial.println("W: Move Forward | S: Move Backward");
    Serial.println("A: Yaw Left    | D: Yaw Right");
    Serial.println("X: Balance-Only Mode");

    angle_setpoint = 0.0f;
 rotation_speed = 0.0f;
}

// 机器人物理参数（根据实际机器人调整）
const float WHEEL_DIAMETER = 0.065;      // 轮子直径（米）
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // 轮子周长
const float WHEEL_BASE = 0.11;           // 轮间距（米）
const int STEPS_PER_REVOLUTION = 3200;    // 200步/转 * 16微步

float lastprinttime = 0;

void updatePosition() {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_position_update) / 1000.0f; // 转换为秒
    
    if (delta_time > 0) {
        // 获取当前电机位置（步数）
        int32_t current_steps1 = step1.getPosition();
        int32_t current_steps2 = step2.getPosition();

        // 计算步数变化量（考虑电机方向）
        float delta_steps1 = static_cast<float>(current_steps1 - last_steps1);
        float delta_steps2 = -static_cast<float>(current_steps2 - last_steps2); // 取负因为电机反向安装
        
        // 保存当前步数用于下次计算
        last_steps1 = current_steps1;
        last_steps2 = current_steps2;
        
        // 将步数转换为距离（米）
        float distance1 = (delta_steps1 / STEPS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        float distance2 = (delta_steps2 / STEPS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        
        // 计算线速度和角速度
        float linear_velocity = (distance1 + distance2) / (2 * delta_time);
        float angular_velocity = (distance2 - distance1) / (WHEEL_BASE * delta_time);
        
        // 更新偏航角（度）
        yaw_angle += angular_velocity * delta_time * RAD_TO_DEG;
        
        // 规范化偏航角到0-360度范围
        if (yaw_angle >= 360.0f) yaw_angle -= 360.0f;
        if (yaw_angle < 0.0f) yaw_angle += 360.0f;
        
        // 更新位置（使用平均偏航角）
        float avg_yaw = (prev_yaw_angle + yaw_angle) / 2.0f;
        pos_x += linear_velocity * delta_time * cos(avg_yaw * DEG_TO_RAD);
        pos_y += linear_velocity * delta_time * sin(avg_yaw * DEG_TO_RAD);
        
        if (millis() - lastprinttime >= 1000) {
            lastprinttime = millis();
            Serial.print("current_steps1: "); Serial.print(current_steps1);
            Serial.print(", current_steps2: "); Serial.print(current_steps2);
            Serial.print(", pos_x: "); Serial.print(pos_x, 3);
            Serial.print(", pos_y: "); Serial.print(pos_y, 3);
            Serial.print(", yaw_angle: "); Serial.print(yaw_angle, 3);
            Serial.println();
        }

        // 保存当前偏航角用于下次计算
        prev_yaw_angle = yaw_angle;
        last_position_update = current_time;
    }
}


void loop() {
    
    if (millis() >= loopTimer) {
        loopTimer += LOOP_INTERVAL_MS;

        calculateTiltAngle();
        computePID();
        handleSerialInput();

        const float TILT_LIMIT_RAD = TILT_LIMIT_DEG * DEG_TO_RAD;
        float calibrated_angle = current_angle - ANGLE_CALIBRATION_OFFSET_RAD;

         
        if (abs(calibrated_angle) > TILT_LIMIT_RAD) {
            
            digitalWrite(STEPPER_EN_PIN, HIGH); 

        } else {
            
            digitalWrite(STEPPER_EN_PIN, LOW); 
        }
        applyMotorSpeeds();

        updatePosition();
    }

    if (millis() >= telemetryTimer) {
        telemetryTimer += TELEMETERY_INTERVAL_MS;

        String telemetry = "{\"a\":";
        telemetry += yaw_angle;
        telemetry += ",\"x\":";
        telemetry += pos_x;
        telemetry += ",\"y\":";
        telemetry += pos_y;
        telemetry += "}";
        
        Serial.println(telemetry);
    }
   
    if (millis() >= printTimer) {
        printTimer += PRINT_INTERVAL_MS;
        Serial.print("Pitch Angle(deg): "); Serial.print(current_angle * RAD_TO_DEG - OBSERVED_UPRIGHT_OFFSET_DEG, 2);
        Serial.println();
    }
}



void calculateTiltAngle() {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    
    float accel_z_m_s2 = accel_event.acceleration.z;
    float gyro_y_rad_s = gyro_event.gyro.y - MPU_GYRO_Y_OFFSET;

    
    float angle_from_accel = 0.0f;
    float asin_arg = accel_z_m_s2 / GRAVITY_ACCELERATION;
    if (asin_arg > 1.0f) asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;
    angle_from_accel = asin(asin_arg);

    
    current_angle = alpha * (current_angle + gyro_y_rad_s * LOOP_INTERVAL_S) + 
                    (1.0f - alpha) * angle_from_accel;
}


void computePID() {
   
    float calibrated_angle = current_angle - ANGLE_CALIBRATION_OFFSET_RAD;

    pid_error = angle_setpoint - calibrated_angle;

    
    pid_integral += pid_error * LOOP_INTERVAL_S;
    if (pid_integral > MAX_PID_INTEGRAL) pid_integral = MAX_PID_INTEGRAL;
    else if (pid_integral < -MAX_PID_INTEGRAL) pid_integral = -MAX_PID_INTEGRAL;

    
    pid_derivative = (pid_error - pid_previous_error) / LOOP_INTERVAL_S;

    
    pid_output = Kp * pid_error + Ki * pid_integral + Kd * pid_derivative;

    pid_previous_error = pid_error;
}


void handleSerialInput() {
    if (Serial.available() > 0) {
        char command = toupper(Serial.read());
        String cmdStr = "";
        cmdStr += command;

        while(Serial.available() > 0) {
            cmdStr += (char)Serial.read();
        }
        cmdStr.trim();

        if (cmdStr == "S") {
            angle_setpoint = Robot_Backward_Speed;
         rotation_speed = 0.0f;
            Serial.println("CMD: Backward");
        } else if (cmdStr == "W") {
            angle_setpoint = Robot_Forward_Speed;
         rotation_speed = 0.0f;
            Serial.println("CMD: Forward");
        } else if (cmdStr == "A") {
         rotation_speed = Rotation_Rate;
            angle_setpoint = 0.0f;
            Serial.println("CMD: Yaw Left");
        } else if (cmdStr == "D") {
         rotation_speed = -Rotation_Rate;
            angle_setpoint = 0.0f;
            Serial.println("CMD: Yaw Right");
        } else if (cmdStr == "X") {
            angle_setpoint = 0.0f;
         rotation_speed = 0.0f;
            Serial.println("CMD: Balance Only");
        } else {
            Serial.print("Unknown command: "); Serial.println(cmdStr);
        }
    }
}


void applyMotorSpeeds() {
    
    float motor1_target_speed_rad_s = pid_output - rotation_speed;
    float motor2_target_speed_rad_s = pid_output + rotation_speed;

    
    step1.setTargetSpeedRad(motor1_target_speed_rad_s);
    step2.setTargetSpeedRad(-motor2_target_speed_rad_s);
}


void performGyroCalibration() {
    Serial.println("Starting Gyro Y-axis calibration. Keep robot STILL and UPRIGHT for 10 seconds...");
    long gyro_y_sum = 0;
    int num_samples = 2000; 

    
    if (!mpu.begin()) {
         Serial.println("MPU6050 not found for calibration!"); return;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    for (int i = 0; i < num_samples; ++i) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyro_y_sum += g.gyro.y * 10000; 
        delay(5);
    }
    MPU_GYRO_Y_OFFSET = (static_cast<float>(gyro_y_sum) / 10000.0f) / num_samples;
    Serial.print("Calibration complete. Calculated MPU_GYRO_Y_OFFSET: ");
    Serial.println(MPU_GYRO_Y_OFFSET, 6);
    Serial.println("Update this value in the code near the top.");
    delay(5000);
}
