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
const int WINDOW_SIZE = 5;      // Size of moving average window
const uint32_t SERIAL_BAUD = 115200;
const unsigned long USB_DEBUG_INTERVAL = 5000; // Interval for USB debug messages (ms)

// PWM related constants
const int MIN_EFFECTIVE_PWM = 50;  // Minimum effective PWM value
const int MAX_PWM = 255;           // Maximum PWM value

// Position limits and protection
const int TRIGGER_MAX = 490;    // Maximum trigger position (released)
const int TRIGGER_MIN = 85;     // Minimum trigger position (fully pressed)
const int MOTOR_POS_MAX = 770;      // Maximum motor position
const int MOTOR_POS_MIN = 299;      // Minimum motor position
const int POSITION_DEADZONE = 8; // Deadzone for position control
const int MAX_CONTINUOUS_PWM_TIME = 5000; // Maximum time (ms) for continuous PWM
const int PWM_COOLDOWN_TIME = 500;      // Cooldown time (ms) after max continuous PWM
const int LIMIT_DEADZONE = 10;          // Deadzone near mechanical limits
const int MIN_PWM_FOR_PROTECTION = 50;  // Minimum PWM value that counts as "active" for protection

// Constants for position change detection
const int POSITION_PRINT_THRESHOLD = 5;  // Only print when position change exceeds this value

// Effect types
const uint8_t EFFECT_GENERAL = 0x10;  // General mode
const uint8_t EFFECT_RACING = 0x11;   // Racing mode with damping
const uint8_t EFFECT_RECOIL = 0x12;   // Recoil mode with vibration
const uint8_t EFFECT_SNIPER = 0x13;   // Sniper mode
const uint8_t EFFECT_TRIGGERLOCK = 0x14;  // Trigger lock mode

// Command constants
const uint8_t CMD_HEADER = 0xAA;
const uint8_t CMD_FOOTER = 0x55;
const uint8_t CMD_MODE_SET = 0x01;
const uint8_t CMD_PARAM_SET = 0x02;

// Parameter IDs
// Racing mode parameters (0x21-0x2F)
const uint8_t PARAM_DAMPING_START_POS = 0x21;
const uint8_t PARAM_DAMPING_STRENGTH = 0x22;

// Recoil mode parameters (0x31-0x3F)
const uint8_t PARAM_VIB_START_POS = 0x31;
const uint8_t PARAM_VIB_START_STRENGTH = 0x32;
const uint8_t PARAM_VIB_INTENSITY = 0x33;
const uint8_t PARAM_VIB_FREQUENCY = 0x34;
const uint8_t PARAM_VIB_START_DATA = 0x35;

// Sniper mode parameters (0x41-0x4F)
const uint8_t PARAM_START_POS = 0x41;
const uint8_t PARAM_TRIGGER_STROKE = 0x42;
const uint8_t PARAM_RESISTANCE = 0x43;
const uint8_t PARAM_BREAK_START_DATA = 0x44;

// Lock mode parameters (0x51-0x5F)
const uint8_t PARAM_LOCK_DAMPING_START = 0x51;

// Effect parameters
struct EffectParams {
    uint8_t type;                    // Effect type
    uint16_t startPosition;          // Position where effect starts
    uint16_t dampingStrength;        // PWM strength for damping (Racing & TriggerLock)
    uint16_t vibrationStrength;      // Initial vibration PWM (Recoil)
    uint16_t vibrationIntensity;     // Vibration amplitude (Recoil)
    uint16_t vibrationFrequency;     // Vibration cycle speed (Recoil)
    uint16_t vibrationStartData;     // Vibration start data (Recoil)
    uint16_t triggerStroke;          // Trigger stroke length (Sniper)
    uint16_t resistance;             // Resistance for Sniper mode
    bool active;                     // Whether effect is currently active
    unsigned long startTime;         // Time when effect started
    unsigned long lastUpdateTime;    // Last time effect was updated
    unsigned long duration;          // Effect duration
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
uint8_t rxBuffer[64];
uint8_t rxIndex = 0;
bool messageStarted = false;
unsigned long lastUsbDebugTime = 0; // Last time USB debug was printed

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

// Maps motor position (299-770) to PWM value (1-255)
// When position goes from 770->299 (pressing), PWM goes from 1->255
// When position goes from 299->770 (releasing), PWM goes from 255->1
int mapPositionToPWM(int position) {
    // Ensure position is within the valid range
    position = constrain(position, TRIGGER_MIN, TRIGGER_MAX);
    
    // Map the position to PWM value (inverted mapping)
    // As position decreases (770->299), PWM increases (1->255)
    return map(position, TRIGGER_MAX, TRIGGER_MIN, 1, 255);
}

// Function declarations
void processHIDReport();
void setupHighFrequencyPWM();
void updateEffect();
void setMotorPWM(int speed);
bool isPositionSafe(int position);
void printEffectInfo();
void processCommand();
void setMode(uint8_t modeId);
void setParameter(uint8_t paramId, uint16_t value);
void impactEffect(int startStrength, int pwmStrength, int frequency);

void setup() {
    // Initialize serial for debugging
    Serial.begin(SERIAL_BAUD);

    Serial.println(F("Force Feedback Controller Initializing..."));

    // Initialize pins
    pinMode(PIN_MOTOR_PWM1, OUTPUT);
    pinMode(PIN_MOTOR_PWM2, OUTPUT);
    pinMode(PIN_TRIGGER_POS, INPUT);
    pinMode(PIN_MOTOR_POS, INPUT);
    
    // Configure PWM frequency to above 30kHz to eliminate motor buzzing noise
    setupHighFrequencyPWM();

    // Initialize HID with RawHID
    RawHID.begin(rawhidData, sizeof(rawhidData));

    // Initialize effect parameters
    currentEffect.active = true;  // 设置为激活状态
    currentEffect.type = EFFECT_GENERAL;  // 默认使用赛车模式
    currentEffect.startPosition = 100;   // 触发位置
    currentEffect.dampingStrength = 150; // 阻尼强度 (0-255)
    currentEffect.startTime = 0;
    currentEffect.lastUpdateTime = 0;
    currentEffect.duration = 10000; // 默认10秒
    
    // Initialize motor state
    motorState.lastPwmStartTime = 0;
    motorState.lastCooldownTime = 0;
    motorState.needsCooldown = false;
    motorState.lastSpeed = 0;
    motorState.enabled = true;  // 启用电机
    
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
        Serial.print(F("Trigger Pos: ")); Serial.println(mapPositionToPWM(triggerPos));
        lastPrintedPos = motorPos;
    }
}

/**
 * Sets up high frequency PWM for motor control to eliminate audible buzzing
 * Configures Timer3 and Timer4 for pins 5 and 6 to operate at 62.5kHz
 * This is well above the human hearing range (>30kHz)
 */
void setupHighFrequencyPWM() {
    // On ATmega32u4 Leonardo:
    // Pin 5 is connected to Timer3 (OC3A)
    // Pin 6 is connected to Timer4 (OC4D)
    
    // Configure Timer3 for pin 5 (OC3A)
    TCCR3A = 0;
    TCCR3B = 0;
    // Set Fast PWM mode (mode 5: WGM3[3:0] = 0101)
    TCCR3A |= (1 << WGM30);
    TCCR3B |= (1 << WGM32);
    // Set non-inverting mode
    TCCR3A |= (1 << COM3A1);
    // Set prescaler to 1 (no prescaling) for highest frequency
    // 16MHz / 1 / 256 = 62.5kHz
    TCCR3B |= (1 << CS30);
    
    // Configure Timer4 for pin 6 (OC4D)
    TCCR4A = 0;
    TCCR4B = 0;
    TCCR4C = 0;
    TCCR4D = 0;
    // Set Fast PWM mode
    TCCR4C |= (1 << PWM4D);
    TCCR4D |= (1 << WGM40);
    // Set prescaler to 1 (no prescaling) for highest frequency
    // 16MHz / 1 / 256 = 62.5kHz
    TCCR4B |= (1 << CS40);
    // Set TOP value to 255 for 8-bit resolution
    TC4H = 0;
    OCR4C = 255;
}

void setMotorPWM(int pwm)
{
    // 应用计算后的PWM值
    if(pwm > 0) {
        analogWrite(PIN_MOTOR_PWM1, 255);
        analogWrite(PIN_MOTOR_PWM2, 255-pwm);
    } else {
        analogWrite(PIN_MOTOR_PWM1, 255);
        analogWrite(PIN_MOTOR_PWM2, 255+pwm);
    }
}

void processHIDReport() {
    // Check if any data is available
    // int available = RawHID.available();
    // if (available > 0) {

    //     Serial.print(F("HID Data Available: ")); Serial.println(available);
    // }
    
    while (RawHID.available()) {
        uint8_t report[64];
        RawHID.readBytes(report, sizeof(report));
        if (report[0] == CMD_HEADER) {
            uint8_t cmdType = report[1];
            uint8_t dataLen = report[2];
            int footerPos = -1;
            for (int i = 3 + dataLen; i < 10; i++) { // Search within a reasonable range
                if (report[i] == CMD_FOOTER) {
                    footerPos = i;
                    break;
                }
            }
            
            if (footerPos != -1) {
                // Serial.print(F("Footer found at position: ")); Serial.println(footerPos);
                
                // Process based on command type
                switch (cmdType) {
                    case CMD_MODE_SET:
                        if (dataLen >= 1) {
                            uint8_t modeId = report[3];
                            setMode(modeId);
                        }
                        break;
                        
                    case CMD_PARAM_SET:
                        if (dataLen >= 3) {
                            uint8_t paramId = report[3];
                            uint16_t value = (report[4] << 8) | report[5];
                            setParameter(paramId, value);
                        }
                        break;
                        
                    default:
                        Serial.print(F("Unknown command type: 0x"));
                        Serial.println(cmdType, HEX);
                        break;
                }
            } 
        }
    }
}

void processCommand() {
    // Check if command format is correct
    if (rxBuffer[0] != CMD_HEADER || rxBuffer[rxIndex-1] != CMD_FOOTER) {
        Serial.println(F("Invalid command format"));
        return;
    }
    
    uint8_t cmdType = rxBuffer[2];
    uint8_t dataLen = rxBuffer[3];
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 2; i < rxIndex-2; i++) {
        checksum += rxBuffer[i];
    }
    checksum &= 0xFF;
    
    // Verify checksum
    if (checksum != rxBuffer[rxIndex-2]) {
        Serial.println(F("Checksum error"));
        return;
    }
    
    // Process based on command type
    switch (cmdType) {
        case CMD_MODE_SET:
            if (dataLen >= 1) {
                setMode(rxBuffer[4]);
                Serial.print(F("Mode set: "));
                Serial.print(rxBuffer[4], HEX);
                Serial.print(F(" = "));
                Serial.println(rxBuffer[4]);
            }
            break;
            
        case CMD_PARAM_SET:
            if (dataLen >= 3) {
                uint8_t paramId = rxBuffer[4];
                uint16_t value = (rxBuffer[5] << 8) | rxBuffer[6];
                setParameter(paramId, value);
                Serial.print(F("Parameter set: "));
                Serial.print(paramId, HEX);
                Serial.print(F(" = "));
                Serial.println(value);
            }
            break;
            
        default:
            Serial.print(F("Unknown command type: "));
            Serial.println(cmdType, HEX);
            break;
    }
}

void setMode(uint8_t modeId) {
    // Set the effect type based on mode ID
    switch (modeId) {
        case EFFECT_GENERAL:
            currentEffect.type = modeId;
            currentEffect.active = true;
            motorState.enabled = false;
            Serial.print(F("Mode set to: "));
            Serial.println(modeId, HEX);    
            break;
        case EFFECT_RACING:
            currentEffect.type = modeId;
            currentEffect.active = true;
            currentEffect.startTime = millis();
            motorState.enabled = true;
            Serial.print(F("Mode set to: "));
            Serial.println(modeId, HEX);
            break;
        case EFFECT_RECOIL:
            currentEffect.type = modeId;
            currentEffect.active = true;
            currentEffect.startTime = millis();
            motorState.enabled = true;
            Serial.print(F("Mode set to: "));
            Serial.println(modeId, HEX);
            break;
        case EFFECT_SNIPER:
            currentEffect.type = modeId;
            currentEffect.active = true;
            currentEffect.startTime = millis();
            motorState.enabled = true;
            Serial.print(F("Mode set to: "));
            Serial.println(modeId, HEX);
            break;
        case EFFECT_TRIGGERLOCK:
            currentEffect.type = modeId;
            currentEffect.active = true;
            currentEffect.startTime = millis();
            motorState.enabled = true;
            Serial.print(F("Mode set to: "));
            Serial.println(modeId, HEX);
            break;
        default:
            Serial.print(F("Unknown mode ID: "));
            Serial.println(modeId, HEX);
            break;
    }
    
    // printEffectInfo();
}

void setParameter(uint8_t paramId, uint16_t value) {
    // Set the appropriate parameter based on parameter ID
    switch (paramId) {
        // Racing mode parameters
        case PARAM_DAMPING_START_POS:
            currentEffect.startPosition = value;
            Serial.print(F("Damping start position: "));
            Serial.println(value);
            break;
        case PARAM_DAMPING_STRENGTH:
            currentEffect.dampingStrength = value;
            Serial.print(F("Damping strength: "));
            Serial.println(value);
            break;

        // Recoil mode parameters
        case PARAM_VIB_START_POS:
            currentEffect.startPosition = value;
            Serial.print(F("Vibration start position: "));
            Serial.println(value);
            break;
        case PARAM_VIB_START_STRENGTH:
            currentEffect.vibrationStrength = value;
            Serial.print(F("Vibration start strength: "));
            Serial.println(value);
            break;
        case PARAM_VIB_INTENSITY:
            currentEffect.vibrationIntensity = value;
            Serial.print(F("Vibration intensity: "));
            Serial.println(value);
            break;
        case PARAM_VIB_FREQUENCY:
            currentEffect.vibrationFrequency = value;
            Serial.print(F("Vibration frequency: "));
            Serial.println(value);
            break;
        case PARAM_VIB_START_DATA:
            currentEffect.vibrationStartData = value;
            Serial.print(F("Vibration start data: "));
            Serial.println(value);
            break;
            
        // Sniper mode parameters
        case PARAM_START_POS:
            currentEffect.startPosition = value;
            Serial.print(F("Start position: "));
            Serial.println(value);
            break;
        case PARAM_TRIGGER_STROKE:
            currentEffect.triggerStroke = value;
            Serial.print(F("Trigger stroke: "));
            Serial.println(value);
            break;
        case PARAM_RESISTANCE:
            currentEffect.resistance = value;
            Serial.print(F("Resistance: "));
            Serial.println(value);
            break;
            
        // Lock mode parameters
        case PARAM_LOCK_DAMPING_START:
            currentEffect.startPosition = value;    
            Serial.print(F("Lock damping start position: "));
            Serial.println(value);
            break;
            
        default:
            Serial.print(F("Unknown parameter ID: "));
            Serial.println(paramId, HEX);
            break;
    }
    
    // Update effect info
    // printEffectInfo();
}

void updateEffect() {
    int force = 0;  
    
    // 根据当前效果类型计算基础力反馈值
    switch (currentEffect.type) {
        case EFFECT_GENERAL:
            // General mode - no force
            force = 0;
            motorState.enabled = false;
            break;
            
        case EFFECT_RACING:
            {
                // 读取当前位置
                int currentPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
                int mappedPos = mapPositionToPWM(currentPos);
                static int prevPos = mappedPos;
                static bool triggerReleased = false;
                static unsigned long releaseStartTime = 0;
                static bool isPressed = false;
                
                // 检测按键是否被按下（位置超过起始位置）
                if (mappedPos > currentEffect.startPosition + 10) {
                    isPressed = true;
                }
                
                // 检测扳机释放 - 改进的检测逻辑
                if (isPressed && mappedPos <= currentEffect.startPosition + 10) {
                    // 从按下状态变为接近起始位置，认为是释放
                    triggerReleased = true;
                    releaseStartTime = millis();
                    isPressed = false;
                }
                
                if (triggerReleased) {
                    // 计算回弹力度 - 使用平滑的回弹曲线
                    unsigned long elapsedTime = millis() - releaseStartTime;
                    // 回弹时间窗口 (ms)
                    const unsigned long REBOUND_WINDOW = 300;
                    
                    if (elapsedTime < REBOUND_WINDOW) {
                        // 使用余弦函数创建平滑的回弹曲线
                        float progress = (float)elapsedTime / REBOUND_WINDOW;
                        float reboundFactor = cos(progress * PI) * 0.5 + 0.5; // 从1平滑降到0
                        
                        // 基础回弹力 - 确保即使在低阻尼时也有足够的回弹力
                        int baseReboundForce = -100; // 负值表示反向力
                        
                        // 应用回弹因子，使回弹力随时间平滑减小
                        force = (int)(baseReboundForce * reboundFactor);
                    } else {
                        // 回弹时间结束后，应用较小的恒定力以确保完全回到初始位置
                        force = -30;
                    }
                    
                    // 如果回到起始位置附近，重置释放标志
                    if (mappedPos <= currentEffect.startPosition + 5) {
                        triggerReleased = false;
                        // 应用原始阻尼力
                        force = (currentEffect.dampingStrength < MIN_EFFECTIVE_PWM && currentEffect.dampingStrength > 0) ? 
                                MIN_EFFECTIVE_PWM : currentEffect.dampingStrength;
                    }
                } else {
                    // 未释放状态下使用正常阻尼力
                    force = (currentEffect.dampingStrength < MIN_EFFECTIVE_PWM && currentEffect.dampingStrength > 0) ? 
                            MIN_EFFECTIVE_PWM : currentEffect.dampingStrength;
                }
                
                // 更新前一位置
                prevPos = mappedPos;
            }
            break;
            
        case EFFECT_RECOIL:
            impactEffect(currentEffect.vibrationStrength, currentEffect.vibrationIntensity, currentEffect.vibrationFrequency);
            break;
            
        case EFFECT_SNIPER:
            {
                int currentPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
                int mappedPos = mapPositionToPWM(currentPos);
                
                // 定义三个阶段的位置阈值
                const int STAGE1_THRESHOLD = currentEffect.startPosition + (currentEffect.triggerStroke / 3);
                const int STAGE2_THRESHOLD = currentEffect.startPosition + (currentEffect.triggerStroke * 2 / 3);

                // 第一阶段：较大阻力
                if (mappedPos < STAGE1_THRESHOLD) {
                    force = currentEffect.resistance;  // 使用完整阻力
                }
                // 第二阶段：恒定较小阻力
                else if (mappedPos < STAGE2_THRESHOLD) {
                    force = 50;  // 阻力降为1/5
                }
                
                // 第三阶段：快速回弹
                else {
                    static bool triggerReleased = false;
                    int prevPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
                    Serial.println(prevPos);
                    // 检测扳机释放
                    if (prevPos > currentPos) {  // 位置值减小表示扳机释放
                        triggerReleased = true;
                    }
                    
                    if (triggerReleased) {
                        force = -50 ;  // 反向回弹
                        setMotorPWM(force);
                        delay(100);
                    } else {
                        force = currentEffect.resistance / 5;  // 维持小阻力直到释放
                    }
                    
                    // 如果回到起始位置，重置释放标志
                    if (mappedPos <= currentEffect.startPosition) {
                        triggerReleased = false;
                    }
                }
            }
            break;

        case EFFECT_TRIGGERLOCK:
            {
                // 全力输出锁死
                force =255; 
            }
            break;
    }
    
    // 应用力反馈
    if (motorState.enabled) {
        // 读取当前位置
        int currentPos = getFilteredPosition(PIN_MOTOR_POS, motorWindow);
        int triggerPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
        
        // 应用安全限制
        if (currentPos <= MOTOR_POS_MIN + LIMIT_DEADZONE || 
            currentPos >= MOTOR_POS_MAX - LIMIT_DEADZONE) {
            // 在限位附近，根据力的方向决定是否允许运动
            if ((currentPos <= MOTOR_POS_MIN + LIMIT_DEADZONE && force < 0) ||
                (currentPos >= MOTOR_POS_MAX - LIMIT_DEADZONE && force > 0)) {
                force = 0;
            }
        }

        // 应用起始位置限制 - 实现渐进式力反馈，避免突然的力反馈变化
        // 创建一个过渡区域，在该区域内力反馈逐渐增加
        int mappedTriggerPos = mapPositionToPWM(triggerPos);
        int transitionZone = 10; // 过渡区域大小，可以根据实际体验调整
        
        if (mappedTriggerPos < (int)currentEffect.startPosition - transitionZone) {
            // 完全在起始位置之前，不应用力反馈
            force = 0;
        } else if (mappedTriggerPos < (int)currentEffect.startPosition) {
            // 在过渡区域内，线性增加力反馈
            float ratio = (float)(mappedTriggerPos - (currentEffect.startPosition - transitionZone)) / transitionZone;
            // 确保比例在0到1之间
            ratio = constrain(ratio, 0.0, 1.0);
            // 应用平滑过渡
            force = (int)(force * ratio);
        }
        // Serial.print("Force:");
        // Serial.println(force);
        setMotorPWM(force);
    }
}

void impactEffect(int startStrength, int pwmStrength, int frequency) {
    static unsigned long lastStateChangeTime = 0;
    static int currentState = 0; // 0: 等待开始, 1: 正转撞击, 2: 反转返回
    
    unsigned long currentTime = millis();
    unsigned long impactInterval = (1000 / frequency / 4); // 计算撞击间隔时间(ms)
    int triggerPos = getFilteredPosition(PIN_TRIGGER_POS, triggerWindow);
    int motorPos = getFilteredPosition(PIN_MOTOR_POS, motorWindow);
    
    switch(currentState) {
        case 0: // 等待开始
            if (mapPositionToPWM(triggerPos) > currentEffect.startPosition+1) {
                if(currentTime - lastStateChangeTime >= impactInterval){
                    lastStateChangeTime = currentTime;
                    setMotorPWM(pwmStrength);
                    currentState = 1;
                    // Serial.print(F("startStrength: "));
                    // Serial.println(pwmStrength);
                }
            }
            break;
            
        case 1: // 正转撞击
            if (currentTime - lastStateChangeTime >= impactInterval) {
                setMotorPWM(-pwmStrength);
                lastStateChangeTime = currentTime;
                currentState = 2;
                // Serial.print(F("impactInterval: "));
                // Serial.println(impactInterval);
            }
            break;
            
        case 2: // 反转返回
            if (motorPos <= MOTOR_POS_MIN + LIMIT_DEADZONE || 
                currentTime - lastStateChangeTime >= impactInterval) {
                setMotorPWM(pwmStrength);
                delay(impactInterval);
                lastStateChangeTime = currentTime;
                currentState = 0;
            }
            break;
    }
}

void printEffectInfo() {
    Serial.println(F("\nEffect Configuration:"));
    Serial.print(F("Type: 0x")); Serial.println(currentEffect.type, HEX);
    
    // Print only parameters relevant to the current effect type
    switch(currentEffect.type) {
        case EFFECT_RACING:
            Serial.print(F("Start Position: ")); Serial.println(currentEffect.startPosition);
            Serial.print(F("Damping Strength: ")); Serial.println(currentEffect.dampingStrength);
            break;
            
        case EFFECT_RECOIL:
            Serial.print(F("Start Position: ")); Serial.println(currentEffect.startPosition);
            Serial.print(F("Vibration Strength: ")); Serial.println(currentEffect.vibrationStrength);
            Serial.print(F("Vibration Intensity: ")); Serial.println(currentEffect.vibrationIntensity);
            Serial.print(F("Vibration Frequency: ")); Serial.println(currentEffect.vibrationFrequency);
            break;
            
        case EFFECT_SNIPER:
            Serial.print(F("Start Position: ")); Serial.println(currentEffect.startPosition);
            Serial.print(F("Trigger Stroke: ")); Serial.println(currentEffect.triggerStroke);
            Serial.print(F("Resistance: ")); Serial.println(currentEffect.resistance);
            break;
            
        case EFFECT_TRIGGERLOCK:
            Serial.print(F("Start Position: ")); Serial.println(currentEffect.startPosition);
            Serial.print(F("Damping Strength: ")); Serial.println(currentEffect.dampingStrength);
            break;
            
        default:
            // For general effect or unknown types, don't print any parameters
            break;
    }
}