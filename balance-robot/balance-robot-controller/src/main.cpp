
#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <math.h>

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;

const int ADC_CS_PIN  = 5;
const int ADC_SCK_PIN = 18;
const int ADC_MISO_PIN= 19;
const int ADC_MOSI_PIN= 23;

const int TOGGLE_PIN  = 32;


const int  PRINT_INTERVAL      = 500;
const int  LOOP_INTERVAL       = 5;
const int  STEPPER_INTERVAL_US = 15;

const float VREF = 4.096f;


const float LOOP_DT  = LOOP_INTERVAL / 1000.0f;
const float DEG2RAD  = PI / 180.0f;

const float SET_FWD_RAD  = -1.0f * DEG2RAD;   // W key
const float SET_BACK_RAD =  1.0f * DEG2RAD;   // S key
const float TURN_SPEED_RAD = 1.1f;            // for A/D spins

const float FILTER_ALPHA = 0.98f;


const float KP = 805.0f;
const float KI = 0.0;                  
const float KD = 2.1f;


const float MIN_SPEED_RAD = 0.5f;             


const uint16_t CALIB_SAMPLES = 1000;
 

float pitch       = 0.0f;
float pitchOffset = 0.0f;
float gyroBiasY   = 0.0f;
float pidIntegral = 0.0f;
float prevError   = 0.0f;

char  currentCmd  = 'X';   
char  lastCmd     = ' ';

bool  motorsEnabled = false; 


ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);


bool IRAM_ATTR TimerHandler(void *)
{
  static bool t = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, t);
  t = !t;
  return true;
}


uint16_t readADC(uint8_t ch)
{
  uint8_t TX0   = 0x06 | (ch >> 2);
  uint8_t TX1_B = (ch & 0x03) << 6;
  digitalWrite(ADC_CS_PIN, LOW);
  SPI.transfer(TX0);
  uint8_t RX0   = SPI.transfer(TX1_B);
  uint8_t RX1_B = SPI.transfer(0x00);
  digitalWrite(ADC_CS_PIN, HIGH);
  return ((RX0 & 0x0F) << 8) | RX1_B;
}


void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  if (!mpu.begin()) { Serial.println("MPU6050 not found"); while (true) delay(10);}  

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  
  float sumA=0,sumG=0; Serial.println("Calibrating … keep upright");
  for(uint16_t i=0;i<CALIB_SAMPLES;++i){sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t); sumA+=atan2(a.acceleration.z,a.acceleration.x); sumG+=g.gyro.y; delay(5);}  
  pitchOffset=sumA/CALIB_SAMPLES - 0.003f; gyroBiasY=sumG/CALIB_SAMPLES;

  ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler);
  step1.setAccelerationRad(35.0f); step2.setAccelerationRad(35.0f);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, HIGH);  

  pinMode(ADC_CS_PIN, OUTPUT); digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}


void loop()
{
  static unsigned long printT=0, loopT=0;

  while (Serial.available()){
    char c = toupper(Serial.read());
    if(strchr("WSADX",c)) currentCmd=c;
  }

  
  

  if(millis()>loopT){
    loopT+=LOOP_INTERVAL;
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    float gyroY=g.gyro.y-gyroBiasY;
    float accelAng=atan2(a.acceleration.z,a.acceleration.x)-pitchOffset;
    pitch=FILTER_ALPHA*(pitch+gyroY*LOOP_DT)+(1-FILTER_ALPHA)*accelAng;

    
    if ((lastCmd == 'A' || lastCmd == 'D') && currentCmd == 'X') {
        pidIntegral = 0.0f; 
        
        
        float errorForNewXState = 0.0f - pitch;
        prevError = errorForNewXState;
    }
    

    
    if(!motorsEnabled && fabs(pitch) < 0.05f){
      digitalWrite(STEPPER_EN_PIN, LOW); 
      pidIntegral = 0;                  
      motorsEnabled = true;
    }

    float setpt=0; if(currentCmd=='W')setpt=SET_FWD_RAD; else if(currentCmd=='S')setpt=SET_BACK_RAD; else if(currentCmd == 'X')setpt=0;
    float err=setpt-pitch; bool okI=fabs(pitch)<0.07f&&fabs(step1.getSpeedRad())<10;
    
    
    if (motorsEnabled && okI) {
        pidIntegral+=err*LOOP_DT;
    } else if (!motorsEnabled) { 
        pidIntegral = 0.0f;
    }
    
    pidIntegral*=0.999f; 
    pidIntegral=constrain(pidIntegral,-0.1f,0.1f); 

    float deriv=(err-prevError)/LOOP_DT; float pid=KP*err+KI*pidIntegral+KD*deriv; prevError=err;

    float ls=pid, rs=-pid;
    if(currentCmd=='A'){ls-=TURN_SPEED_RAD; rs-=TURN_SPEED_RAD;} else if(currentCmd=='D'){ls+=TURN_SPEED_RAD; rs+=TURN_SPEED_RAD;} 
    
    
    if (motorsEnabled && fabs(ls) < MIN_SPEED_RAD && fabs(rs) < MIN_SPEED_RAD) {
      ls = 0;
      rs = 0;
    } else if (!motorsEnabled) { 
        ls = 0;
        rs = 0;
    }

    step1.setTargetSpeedRad(ls); step2.setTargetSpeedRad(rs);
  }

if (millis() > printT) {
    printT += PRINT_INTERVAL;
    Serial.print(pitch * 180000.0f / PI);  
    Serial.print(' ');
    Serial.print(step1.getSpeedRad());
    Serial.print(' ');
    Serial.print((readADC(0)*VREF)/4095.0f);
    Serial.println();
  }



  if (currentCmd != lastCmd) {
    Serial.print("CMD:");
    Serial.print(currentCmd);
    Serial.print("  tilt_deg:");
    Serial.println(pitch * 180.0f / PI);
    lastCmd = currentCmd;
  }
  
}
