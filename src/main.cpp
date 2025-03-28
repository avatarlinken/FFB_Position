/*
 * Force Feedback Implementation for Arduino Leonardo
 * Based on HID-Project library with motor protection
 */
#include"Arduino.h"
#include "HID-Project.h"

// Pin Definitions
const int PIN_MOTOR_PWM1 = 5;  // Motor control PWM pin 1
const int PIN_MOTOR_PWM2 = 6;  // Motor control PWM pin 2
const int PIN_TRIGGER_POS = A1; // Trigger position analog input
const int PIN_MOTOR_POS = A0;   // Motor position feedback analog input

// Constants
const int WINDOW_SIZE = 8;      // Size of moving average window
const uint32_t SERIAL_BAUD = 115200;
const int SAMPLES_PER_READ = 8; // Number of samples to read for each position reading
const int SAMPLE_DELAY = 1;     // Delay between samples in microseconds

// PWM related constants
const int MIN_EFFECTIVE_PWM = 50;  // Minimum effective PWM value
const int MAX_PWM = 255;           // Maximum PWM value

// PWM smoothing related variables
const int PWM_STEP = 5;        // Maximum PWM change step size
int currentPWM1 = 0;           // Current PWM1 value
int currentPWM2 = 0;           // Current PWM2 value

// Position limits and protection
const int TRIGGER_MAX = 490;    // Maximum trigger position (released)
const int TRIGGER_MIN = 85;     // Minimum trigger position (fully pressed)
const int MOTOR_MAX = 770;      // Maximum motor position
const int MOTOR_MIN = 299;      // Minimum motor position
const int POSITION_DEADZONE = 8; // Deadzone for position control
const int MAX_CONTINUOUS_PWM_TIME = 5000; // Maximum time (ms) for continuous PWM
const int PWM_COOLDOWN_TIME = 500;      // Cooldown time (ms) after max continuous PWM
const int LIMIT_DEADZONE = 10;          // Deadzone near mechanical limits
const int MIN_PWM_FOR_PROTECTION = 50;  // Minimum PWM value that counts as "active" for protection

// Constants for force feedback profiles
const int FORCE_PROFILE_LINEAR = 0;
const int FORCE_PROFILE_PROGRESSIVE = 1;
const int FORCE_PROFILE_TRIGGER = 2;

// Constants for position change detection
const int POSITION_PRINT_THRESHOLD = 5;  // Only print when position change exceeds this value

// Current force profile
int currentForceProfile = FORCE_PROFILE_LINEAR;
// Effect types
const uint8_t EFFECT_CONSTANT = 0;
const uint8_t EFFECT_RAMP = 1;
const uint8_t EFFECT_SQUARE = 2;
const uint8_t EFFECT_SINE = 3;
const uint8_t EFFECT_TRIANGLE = 4;
const uint8_t EFFECT_SAWTOOTHDOWN = 5;
const uint8_t EFFECT_SAWTOOTHUP = 6;

// Effect parameters
struct EffectParams {
    uint8_t type;
    uint16_t duration;
    int16_t startLevel;
    int16_t endLevel;
    uint16_t period;
    uint16_t magnitude;
    bool active;
    unsigned long startTime;
} currentEffect;

// Motor protection state
struct MotorState {
    unsigned long lastPwmStartTime;
    unsigned long lastCooldownTime;
    bool needsCooldown;
    int lastSpeed;
    bool enabled;  // Control whether motor is enabled
} motorState;

// HID communication buffer
uint8_t rawhidData[64];

// Sliding window buffer
struct SlidingWindow {
    int buffer[WINDOW_SIZE];
    int index;
    bool isFull;
    
    
    SlidingWindow() : index(0), isFull(false) {
        memset(buffer, 0, sizeof(buffer));
    }
    
    // Add new value and return window average
    int addValue(int newValue) {
        // Save new value
        buffer[index] = newValue;
        
        // Update index
        index = (index + 1) % WINDOW_SIZE;
        
        // Check if window is full
        if (index == 0) {
            isFull = true;
        }
        
        // Calculate average
        long sum = 0;
        int count = isFull ? WINDOW_SIZE : index;
        
        for (int i = 0; i < count; i++) {
            sum += buffer[i];
        }
        
        return sum / count;
    }
};

// Create sliding window for each sensor
SlidingWindow triggerWindow;
SlidingWindow motorWindow;

int getFilteredPosition(int pin, SlidingWindow& window) {
    // Read new value
    int newValue = analogRead(pin);
    
    // Process through sliding window
    return window.addValue(newValue);
}

// Function declarations
void processHIDReport();
void updateEffect();
void setMotorPWM(int speed);
bool isPositionSafe(int position);
void printEffectInfo();
void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_BAUD);

    Serial.println(F("Force Feedback Controller Initializing..."));

    // Initialize pins
    pinMode(PIN_MOTOR_PWM1, OUTPUT);
    pinMode(PIN_MOTOR_PWM2, OUTPUT);
    pinMode(PIN_TRIGGER_POS, INPUT);
    pinMode(PIN_MOTOR_POS, INPUT);

    // Initialize HID with RawHID
    RawHID.begin(rawhidData, sizeof(rawhidData));

    // Initialize effect parameters
    currentEffect.active = false;
    
    // Initialize motor state
    motorState.lastPwmStartTime = 0;
    motorState.lastCooldownTime = 0;
    motorState.needsCooldown = false;
    motorState.lastSpeed = 0;
    motorState.enabled = false;  // Initial state is disabled
    
    // Ensure motor is stopped
    setMotorPWM(0);
    
    Serial.println(F("Initialization complete. Waiting for commands..."));
}

void loop() {
    // Read and filter positions
    int triggerPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
    int motorPos = getFilteredPosition(PIN_MOTOR_POS, motorWindow);

    // Process any incoming HID reports
    processHIDReport();
    
    // Update motor state based on current mode
    if (motorState.enabled) {
        // Update effect if active
        if (currentEffect.active) {
            updateEffect();
        } 
    } else {
        // If motor is disabled, ensure it stops
        setMotorPWM(0);
    }

    // Print position information (when position change exceeds threshold)
    static int lastPrintedPos = 0;
    if (abs(motorPos - lastPrintedPos) > POSITION_PRINT_THRESHOLD) {
        Serial.print(F("Motor Pos: ")); Serial.println(motorPos);
        lastPrintedPos = motorPos;
    }
}

void setMotorPWM(int pwm)
{
  if(pwm>0)
  {
    analogWrite(PIN_MOTOR_PWM1, 255);
    analogWrite(PIN_MOTOR_PWM2, 255-pwm);
  }
  else
  {
    analogWrite(PIN_MOTOR_PWM1, 255);
    analogWrite(PIN_MOTOR_PWM2, 255+pwm);
  }
}

void processHIDReport() {
    while (RawHID.available()) {
        uint8_t report[64];
        RawHID.readBytes(report, sizeof(report));
        
        uint8_t reportId = report[0];
        switch (reportId) {
            case 1: // Set Effect
                currentEffect.type = report[2];
                currentEffect.duration = (report[3] << 8) | report[4];
                currentEffect.startLevel = (report[5] << 8) | report[6];
                currentEffect.endLevel = (report[7] << 8) | report[8];
                currentEffect.period = (report[9] << 8) | report[10];
                currentEffect.magnitude = (report[11] << 8) | report[12];
                motorState.enabled = true;
                printEffectInfo();
                break;
                
            case 2: // Start Effect
                if (motorState.enabled) {  // 只有在电机启用时才开始效果
                    currentEffect.active = true;
                    currentEffect.startTime = millis();
                    Serial.println(F("Effect Started"));
                }
                break;
                
            case 3: // Stop Effect
                currentEffect.active = false;
                motorState.enabled = false;  // 停止效果时禁用电机
                setMotorPWM(0);
                Serial.println(F("Effect Stopped - Motor Disabled"));
                break;
        }
    }
}

void updateEffect() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - currentEffect.startTime;
    
    if (elapsedTime >= currentEffect.duration) {
        currentEffect.active = false;
        setMotorPWM(0);
        Serial.println(F("Effect Complete"));
        return;
    }

    int force = 0;
    
    switch (currentEffect.type) {
        case EFFECT_CONSTANT:
            // 确保magnitude大于最小有效值
            force = (currentEffect.magnitude < MIN_EFFECTIVE_PWM && currentEffect.magnitude > 0) ? 
                    MIN_EFFECTIVE_PWM : currentEffect.magnitude;
            break;
            
        case EFFECT_RAMP:
            force = map(elapsedTime, 0, currentEffect.duration, 
                       currentEffect.startLevel, currentEffect.endLevel);
            // 保持力的方向，但确保绝对值不小于最小有效值
            if (abs(force) < MIN_EFFECTIVE_PWM && force != 0) {
                force = (force > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
            }
            break;
            
        case EFFECT_SQUARE:
            if ((elapsedTime % currentEffect.period) < (currentEffect.period / 2))
                force = max(MIN_EFFECTIVE_PWM, currentEffect.magnitude);
            else
                force = min(-MIN_EFFECTIVE_PWM, -currentEffect.magnitude);
            break;
            
        case EFFECT_SINE:
            {
                float rawForce = sin(2 * PI * elapsedTime / currentEffect.period) * currentEffect.magnitude;
                // 对正弦波进行调整，保持平滑但确保达到最小有效值
                if (abs(rawForce) < MIN_EFFECTIVE_PWM && rawForce != 0) {
                    force = (rawForce > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
                } else {
                    force = rawForce;
                }
            }
            break;

        case EFFECT_TRIANGLE:
            {
                unsigned long periodPos = elapsedTime % currentEffect.period;
                if (periodPos < (currentEffect.period / 2)) {
                    force = map(periodPos, 0, currentEffect.period / 2, 
                              -currentEffect.magnitude, currentEffect.magnitude);
                } else {
                    force = map(periodPos, currentEffect.period / 2, currentEffect.period,
                              currentEffect.magnitude, -currentEffect.magnitude);
                }
                // 调整三角波，确保达到最小有效值
                if (abs(force) < MIN_EFFECTIVE_PWM && force != 0) {
                    force = (force > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
                }
            }
            break;

        case EFFECT_SAWTOOTHDOWN:
        case EFFECT_SAWTOOTHUP:
            {
                unsigned long periodPos = elapsedTime % currentEffect.period;
                int rawForce;
                if (currentEffect.type == EFFECT_SAWTOOTHDOWN) {
                    rawForce = map(periodPos, 0, currentEffect.period,
                                 currentEffect.magnitude, -currentEffect.magnitude);
                } else {
                    rawForce = map(periodPos, 0, currentEffect.period,
                                 -currentEffect.magnitude, currentEffect.magnitude);
                }
                // 调整锯齿波，确保达到最小有效值
                if (abs(rawForce) < MIN_EFFECTIVE_PWM && rawForce != 0) {
                    force = (rawForce > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
                } else {
                    force = rawForce;
                }
            }
            break;
    }
    
    if (motorState.enabled) {
        // 读取当前电机位置
        int currentPos = getFilteredPosition(PIN_MOTOR_POS, motorWindow);
        
        // 应用安全限制
        if (currentPos <= MOTOR_MIN + LIMIT_DEADZONE || 
            currentPos >= MOTOR_MAX - LIMIT_DEADZONE) {
            // 在限位附近，根据力的方向决定是否允许运动
            if ((currentPos <= MOTOR_MIN + LIMIT_DEADZONE && force < 0) ||
                (currentPos >= MOTOR_MAX - LIMIT_DEADZONE && force > 0)) {
                force = 0;
            }
        }
        
        setMotorPWM(force);
    }
}

void printEffectInfo() {
    Serial.println(F("\nNew Effect Set:"));
    Serial.print(F("Type: ")); Serial.println(currentEffect.type);
    Serial.print(F("Duration: ")); Serial.println(currentEffect.duration);
    Serial.print(F("Start Level: ")); Serial.println(currentEffect.startLevel);
    Serial.print(F("End Level: ")); Serial.println(currentEffect.endLevel);
    Serial.print(F("Period: ")); Serial.println(currentEffect.period);
    Serial.print(F("Magnitude: ")); Serial.println(currentEffect.magnitude);
}