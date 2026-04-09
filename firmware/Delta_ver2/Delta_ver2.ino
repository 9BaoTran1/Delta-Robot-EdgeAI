/*******************************************************************************
 * @file    DeltaRobotController_v8.3_ALL_SENSORS_OK.ino
 * @brief   Delta Robot - CẢ 3 SENSOR HOẠT ĐỘNG
 * @version 8.3 - Auto Homing A, B, C
 * @date    2026-01-28
 * 
 * ============================================================================
 * HARDWARE VERIFIED:
 * ============================================================================
 *  Sensor A (Pin 19) - WORKING
 *  Sensor B (Pin 21) - WORKING
 *  Sensor C (Pin 24) - WORKING
 * 
 * → AUTO HOMING CHO CẢ 3 MOTOR!
 ******************************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include "DeltaKinematics.h"

/*******************************************************************************
 * HARDWARE PIN CONFIGURATION
 ******************************************************************************/
#define PIN_MOTOR_A_PULSE           53
#define PIN_MOTOR_A_DIRECTION       52
#define PIN_MOTOR_B_PULSE           5
#define PIN_MOTOR_B_DIRECTION       6
#define PIN_MOTOR_C_PULSE           3
#define PIN_MOTOR_C_DIRECTION       4

#define PIN_LIMIT_SWITCH_A          19  // WORKING
#define PIN_LIMIT_SWITCH_B          21  // WORKING
#define PIN_LIMIT_SWITCH_C          24  // WORKING

#define PIN_VACUUM_PUMP             22

/*******************************************************************************
 * ROBOT MECHANICAL PARAMETERS
 ******************************************************************************/
#define ROBOT_ARM_LENGTH            350.0f
#define ROBOT_ROD_LENGTH            500.0f
#define ROBOT_BASE_TRIANGLE         477.0f
#define ROBOT_PLATFORM_TRIANGLE     85.0f

/*******************************************************************************
 * STEPPER MOTOR CONFIGURATION
 ******************************************************************************/
#define MOTOR_MICROSTEPS            3200.0f
#define MOTOR_GEAR_RATIO            5.0f
#define MOTOR_STEPS_PER_DEGREE      ((MOTOR_MICROSTEPS * MOTOR_GEAR_RATIO) / 360.0f)

/*******************************************************************************
 * MOTION PROFILES
 ******************************************************************************/
#define HOMING_MAX_SPEED            70.0f
#define HOMING_MAX_ACCEL            30.0f

#define FAST_MAX_SPEED              3000.0f
#define FAST_MAX_ACCEL              1800.0f

#define PRECISION_MAX_SPEED         1200.0f
#define PRECISION_MAX_ACCEL         600.0f

#define SMOOTH_MAX_SPEED            2400.0f
#define SMOOTH_MAX_ACCEL            1000.0f

/*******************************************************************************
 * WORKSPACE CONFIGURATION
 ******************************************************************************/
#define DEFAULT_WORKSPACE_RADIUS    200.0f
#define DEFAULT_WORKSPACE_Z_MIN     -400.0f
#define DEFAULT_WORKSPACE_Z_MAX     -50.0f
#define DEFAULT_SAFE_HEIGHT         -100.0f

const float WORKSPACE_RADIUS_TABLE[][2] = {
    {-50.0f,  150.0f},
    {-100.0f, 180.0f},
    {-150.0f, 200.0f},
    {-200.0f, 200.0f},
    {-250.0f, 190.0f},
    {-300.0f, 170.0f},
    {-350.0f, 140.0f},
    {-400.0f, 100.0f}
};
#define WORKSPACE_TABLE_SIZE (sizeof(WORKSPACE_RADIUS_TABLE) / sizeof(WORKSPACE_RADIUS_TABLE[0]))

/*******************************************************************************
 * PICKUP HEIGHTS
 ******************************************************************************/
#define DEFAULT_PICKUP_HEIGHT_PAPER     -230.0f
#define DEFAULT_PICKUP_HEIGHT_PLASTIC   -210.0f
#define DEFAULT_PICKUP_HEIGHT_METAL     -220.0f

/*******************************************************************************
 * TIMING PARAMETERS
 ******************************************************************************/
#define TIMING_VACUUM_ON_MS         120
#define TIMING_VACUUM_OFF_MS        100
#define TIMING_MOVE_SETTLE_MS       30
#define TIMING_HOMING_TIMEOUT_MS    30000
#define TIMING_WATCHDOG_MS          10000

/*******************************************************************************
 * COMMUNICATION
 ******************************************************************************/
#define SERIAL_BAUD_RATE            115200
#define CMD_BUFFER_SIZE             256

/*******************************************************************************
 * EEPROM MEMORY MAP
 ******************************************************************************/
#define EEPROM_MAGIC_NUMBER         0xDEADBEEF
#define EEPROM_ADDR_MAGIC           0
#define EEPROM_ADDR_BIN_PAPER       8
#define EEPROM_ADDR_BIN_PLASTIC     20
#define EEPROM_ADDR_BIN_METAL       32
#define EEPROM_ADDR_PICKUP_PAPER    44
#define EEPROM_ADDR_PICKUP_PLASTIC  48
#define EEPROM_ADDR_PICKUP_METAL    52
#define EEPROM_ADDR_SAFE_HEIGHT     56
#define EEPROM_ADDR_WORKSPACE       60
#define EEPROM_ADDR_TOTAL_CYCLES    84
#define EEPROM_ADDR_ERROR_COUNT     88

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

typedef enum {
    STATE_UNINITIALIZED = 0,
    STATE_IDLE,
    STATE_HOMING,
    STATE_TEACH,
    STATE_AUTO,
    STATE_MOVING,
    STATE_SORTING,
    STATE_ERROR,
    STATE_MAX
} RobotState_t;

typedef enum {
    PROFILE_HOMING = 0,
    PROFILE_FAST,
    PROFILE_PRECISION,
    PROFILE_SMOOTH,
    PROFILE_MAX
} MotionProfile_t;

typedef enum {
    WASTE_TYPE_UNKNOWN = 0,
    WASTE_TYPE_PAPER,
    WASTE_TYPE_PLASTIC,
    WASTE_TYPE_METAL,
    WASTE_TYPE_MAX
} WasteType_t;

typedef enum {
    ERROR_NONE = 0,
    ERROR_NOT_HOMED = 1,
    ERROR_IK_FAILED = 3,
    ERROR_WATCHDOG_TIMEOUT = 5,
    ERROR_WORKSPACE_VIOLATION = 6,
    ERROR_HOMING_TIMEOUT = 9,
    ERROR_INVALID_MODE = 10,
    ERROR_SINGULARITY = 11,
    ERROR_MAX
} ErrorCode_t;

typedef struct {
    float x, y, z;
} Position_t;

typedef struct {
    float theta1, theta2, theta3;
} MotorAngles_t;

typedef struct {
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;
    float max_radius;
} WorkspaceLimits_t;

typedef struct {
    RobotState_t        state;
    Position_t          currentPosition;
    MotorAngles_t       motorAngles;
    MotionProfile_t     activeProfile;
    
    bool                isHomed;
    bool                motorsEnabled;
    bool                vacuumActive;
    
    WasteType_t         currentWasteType;
    ErrorCode_t         lastError;
    uint16_t            errorCount;
    
    uint32_t            totalCycles;
    uint16_t            lastCycleTime_ms;
    uint16_t            averageCycleTime_ms;
    uint32_t            cycleStartTime_ms;
    
    uint32_t            lastCommandTime_ms;
    uint32_t            uptimeSeconds;
    
    float               safeHeight;
    float               pickupHeights[WASTE_TYPE_MAX];
    WorkspaceLimits_t   workspace;
    
} SystemState_t;

typedef struct {
    Position_t  position;
    bool        isEnabled;
} WasteBin_t;

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

static DeltaKinematics g_kinematics(
    ROBOT_ARM_LENGTH,
    ROBOT_ROD_LENGTH,
    ROBOT_BASE_TRIANGLE,
    ROBOT_PLATFORM_TRIANGLE
);

static AccelStepper g_motorA(AccelStepper::DRIVER, PIN_MOTOR_A_PULSE, PIN_MOTOR_A_DIRECTION);
static AccelStepper g_motorB(AccelStepper::DRIVER, PIN_MOTOR_B_PULSE, PIN_MOTOR_B_DIRECTION);
static AccelStepper g_motorC(AccelStepper::DRIVER, PIN_MOTOR_C_PULSE, PIN_MOTOR_C_DIRECTION);

static SystemState_t g_state;

static WasteBin_t g_bins[WASTE_TYPE_MAX] = {
    {{0.0f, 0.0f, 0.0f}, false},
    {{0.0f, 0.0f, DEFAULT_PICKUP_HEIGHT_PAPER}, false},
    {{0.0f, 0.0f, DEFAULT_PICKUP_HEIGHT_PLASTIC}, false},
    {{0.0f, 0.0f, DEFAULT_PICKUP_HEIGHT_METAL}, false}
};

static char g_cmdBuffer[CMD_BUFFER_SIZE];
static uint16_t g_cmdBufferIndex = 0;

/*******************************************************************************
 * UTILITY FUNCTIONS
 ******************************************************************************/

float degreesToSteps(float degrees) {
    return degrees * MOTOR_STEPS_PER_DEGREE;
}

float stepsToDegrees(long steps) {
    return (float)steps / MOTOR_STEPS_PER_DEGREE;
}

float getMaxRadiusAtZ(float z) {
    if (z >= WORKSPACE_RADIUS_TABLE[0][0]) {
        return WORKSPACE_RADIUS_TABLE[0][1];
    }
    if (z <= WORKSPACE_RADIUS_TABLE[WORKSPACE_TABLE_SIZE - 1][0]) {
        return WORKSPACE_RADIUS_TABLE[WORKSPACE_TABLE_SIZE - 1][1];
    }
    
    for (uint8_t i = 0; i < WORKSPACE_TABLE_SIZE - 1; i++) {
        if (z >= WORKSPACE_RADIUS_TABLE[i + 1][0] && z <= WORKSPACE_RADIUS_TABLE[i][0]) {
            float z1 = WORKSPACE_RADIUS_TABLE[i][0];
            float z2 = WORKSPACE_RADIUS_TABLE[i + 1][0];
            float r1 = WORKSPACE_RADIUS_TABLE[i][1];
            float r2 = WORKSPACE_RADIUS_TABLE[i + 1][1];
            
            float t = (z - z1) / (z2 - z1);
            return r1 + t * (r2 - r1);
        }
    }
    
    return DEFAULT_WORKSPACE_RADIUS;
}

/*******************************************************************************
 * EEPROM FUNCTIONS
 ******************************************************************************/

void EEPROM_SaveConfig(void) {
    EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_NUMBER);
    EEPROM.put(EEPROM_ADDR_BIN_PAPER, g_bins[WASTE_TYPE_PAPER].position);
    EEPROM.put(EEPROM_ADDR_BIN_PLASTIC, g_bins[WASTE_TYPE_PLASTIC].position);
    EEPROM.put(EEPROM_ADDR_BIN_METAL, g_bins[WASTE_TYPE_METAL].position);
    EEPROM.put(EEPROM_ADDR_PICKUP_PAPER, g_state.pickupHeights[WASTE_TYPE_PAPER]);
    EEPROM.put(EEPROM_ADDR_PICKUP_PLASTIC, g_state.pickupHeights[WASTE_TYPE_PLASTIC]);
    EEPROM.put(EEPROM_ADDR_PICKUP_METAL, g_state.pickupHeights[WASTE_TYPE_METAL]);
    EEPROM.put(EEPROM_ADDR_SAFE_HEIGHT, g_state.safeHeight);
    EEPROM.put(EEPROM_ADDR_WORKSPACE, g_state.workspace);
    EEPROM.put(EEPROM_ADDR_TOTAL_CYCLES, g_state.totalCycles);
    EEPROM.put(EEPROM_ADDR_ERROR_COUNT, g_state.errorCount);
    
    Serial.println(F("{\"eeprom\":\"saved\"}"));
}

void EEPROM_LoadConfig(void) {
    uint32_t magic;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);
    
    if (magic == EEPROM_MAGIC_NUMBER) {
        EEPROM.get(EEPROM_ADDR_BIN_PAPER, g_bins[WASTE_TYPE_PAPER].position);
        EEPROM.get(EEPROM_ADDR_BIN_PLASTIC, g_bins[WASTE_TYPE_PLASTIC].position);
        EEPROM.get(EEPROM_ADDR_BIN_METAL, g_bins[WASTE_TYPE_METAL].position);
        EEPROM.get(EEPROM_ADDR_PICKUP_PAPER, g_state.pickupHeights[WASTE_TYPE_PAPER]);
        EEPROM.get(EEPROM_ADDR_PICKUP_PLASTIC, g_state.pickupHeights[WASTE_TYPE_PLASTIC]);
        EEPROM.get(EEPROM_ADDR_PICKUP_METAL, g_state.pickupHeights[WASTE_TYPE_METAL]);
        EEPROM.get(EEPROM_ADDR_SAFE_HEIGHT, g_state.safeHeight);
        EEPROM.get(EEPROM_ADDR_WORKSPACE, g_state.workspace);
        EEPROM.get(EEPROM_ADDR_TOTAL_CYCLES, g_state.totalCycles);
        EEPROM.get(EEPROM_ADDR_ERROR_COUNT, g_state.errorCount);
        
        Serial.print(F("{\"eeprom\":\"loaded\",\"cycles\":"));
        Serial.print(g_state.totalCycles);
        Serial.println(F("}"));
    } else {
        g_state.safeHeight = DEFAULT_SAFE_HEIGHT;
        g_state.pickupHeights[WASTE_TYPE_PAPER] = DEFAULT_PICKUP_HEIGHT_PAPER;
        g_state.pickupHeights[WASTE_TYPE_PLASTIC] = DEFAULT_PICKUP_HEIGHT_PLASTIC;
        g_state.pickupHeights[WASTE_TYPE_METAL] = DEFAULT_PICKUP_HEIGHT_METAL;
        
        g_state.workspace.x_min = -DEFAULT_WORKSPACE_RADIUS;
        g_state.workspace.x_max = DEFAULT_WORKSPACE_RADIUS;
        g_state.workspace.y_min = -DEFAULT_WORKSPACE_RADIUS;
        g_state.workspace.y_max = DEFAULT_WORKSPACE_RADIUS;
        g_state.workspace.z_min = DEFAULT_WORKSPACE_Z_MIN;
        g_state.workspace.z_max = DEFAULT_WORKSPACE_Z_MAX;
        g_state.workspace.max_radius = DEFAULT_WORKSPACE_RADIUS;
        
        Serial.println(F("{\"eeprom\":\"default\"}"));
    }
}

/*******************************************************************************
 * MOTION CONTROL
 ******************************************************************************/

void Motion_SetProfile(MotionProfile_t profile) {
    g_state.activeProfile = profile;
    
    float speed, accel;
    
    switch(profile) {
        case PROFILE_HOMING:
            speed = HOMING_MAX_SPEED;
            accel = HOMING_MAX_ACCEL;
            break;
        case PROFILE_FAST:
            speed = FAST_MAX_SPEED;
            accel = FAST_MAX_ACCEL;
            break;
        case PROFILE_PRECISION:
            speed = PRECISION_MAX_SPEED;
            accel = PRECISION_MAX_ACCEL;
            break;
        case PROFILE_SMOOTH:
            speed = SMOOTH_MAX_SPEED;
            accel = SMOOTH_MAX_ACCEL;
            break;
        default:
            speed = FAST_MAX_SPEED;
            accel = FAST_MAX_ACCEL;
    }
    
    g_motorA.setMaxSpeed(speed);
    g_motorA.setAcceleration(accel);
    g_motorB.setMaxSpeed(speed);
    g_motorB.setAcceleration(accel);
    g_motorC.setMaxSpeed(speed);
    g_motorC.setAcceleration(accel);
}

bool Motion_IsWithinWorkspace(const Position_t* pos) {
    if (pos->z < g_state.workspace.z_min || pos->z > g_state.workspace.z_max) {
        return false;
    }
    
    float max_radius = getMaxRadiusAtZ(pos->z);
    float radius = sqrt(pos->x * pos->x + pos->y * pos->y);
    
    if (radius > max_radius) {
        return false;
    }
    
    if (pos->x < g_state.workspace.x_min || pos->x > g_state.workspace.x_max) {
        return false;
    }
    if (pos->y < g_state.workspace.y_min || pos->y > g_state.workspace.y_max) {
        return false;
    }
    
    if (!g_kinematics.isReachable(pos->x, pos->y, pos->z)) {
        return false;
    }
    
    return true;
}

/**
 * Compute a distance-aware speed/acceleration for smoother motion.
 *
 * - For long moves: use full profile speed/accel.
 * - For very short moves: automatically reduce speed/accel to avoid jerk.
 */
void Motion_ComputeDynamicProfile(long maxSteps, float &outSpeed, float &outAccel) {
    // Base values from the active motion profile
    float baseSpeed;
    float baseAccel;
    
    switch (g_state.activeProfile) {
        case PROFILE_HOMING:
            baseSpeed = HOMING_MAX_SPEED;
            baseAccel = HOMING_MAX_ACCEL;
            break;
        case PROFILE_FAST:
            baseSpeed = FAST_MAX_SPEED;
            baseAccel = FAST_MAX_ACCEL;
            break;
        case PROFILE_PRECISION:
            baseSpeed = PRECISION_MAX_SPEED;
            baseAccel = PRECISION_MAX_ACCEL;
            break;
        case PROFILE_SMOOTH:
        default:
            baseSpeed = SMOOTH_MAX_SPEED;
            baseAccel = SMOOTH_MAX_ACCEL;
            break;
    }
    
    if (maxSteps <= 0) {
        outSpeed = baseSpeed;
        outAccel = baseAccel;
        return;
    }
    
    /**
     * Distance scaling:
     * - Around 4000 steps (~90°): use 100% of baseSpeed/baseAccel.
     * - Smaller moves: scale down, but never below 25% of base.
     *
     * You can tune:
     *  - 4000.0f  → larger value = smoother for big moves, smaller = more aggressive.
     *  - 0.25f    → minimum smoothness factor (raise for gentler motion, lower for faster).
     */
    float distanceFactor = (float)maxSteps / 4000.0f;
    if (distanceFactor > 1.0f) distanceFactor = 1.0f;
    if (distanceFactor < 0.25f) distanceFactor = 0.25f;
    
    outSpeed = baseSpeed * distanceFactor;
    outAccel = baseAccel * distanceFactor;
}

bool Motion_MoveTo(const Position_t* target, bool checkLimits) {
    if (checkLimits && !Motion_IsWithinWorkspace(target)) {
        Serial.println(F("{\"error\":\"workspace_violation\"}"));
        g_state.lastError = ERROR_WORKSPACE_VIOLATION;
        return false;
    }
    
    int result = g_kinematics.inverse(target->x, target->y, target->z);
    
    if (result == singularity_error) {
        Serial.println(F("{\"error\":\"singularity_detected\"}"));
        g_state.lastError = ERROR_SINGULARITY;
        g_state.errorCount++;
        return false;
    }
    
    if (result != no_error) {
        Serial.println(F("{\"error\":\"ik_failed\"}"));
        g_state.lastError = ERROR_IK_FAILED;
        g_state.errorCount++;
        return false;
    }
    
    long stepsA = (long)((g_kinematics.a - g_state.motorAngles.theta1) * MOTOR_STEPS_PER_DEGREE);
    long stepsB = (long)((g_kinematics.b - g_state.motorAngles.theta2) * MOTOR_STEPS_PER_DEGREE);
    long stepsC = (long)((g_kinematics.c - g_state.motorAngles.theta3) * MOTOR_STEPS_PER_DEGREE);
    
    long maxSteps = max(abs(stepsA), max(abs(stepsB), abs(stepsC)));
    if (maxSteps == 0) return true;
    
    float profileSpeed, profileAccel;
    Motion_ComputeDynamicProfile(maxSteps, profileSpeed, profileAccel);
    
    // Apply distance-aware acceleration for smoother starts/stops
    g_motorA.setAcceleration(profileAccel);
    g_motorB.setAcceleration(profileAccel);
    g_motorC.setAcceleration(profileAccel);
    
    float timeToMove = (float)maxSteps / profileSpeed;
    
    g_motorA.setMaxSpeed((float)abs(stepsA) / timeToMove);
    g_motorB.setMaxSpeed((float)abs(stepsB) / timeToMove);
    g_motorC.setMaxSpeed((float)abs(stepsC) / timeToMove);
    
    g_motorA.move(stepsA);
    g_motorB.move(stepsB);
    g_motorC.move(stepsC);
    
    uint32_t lastUpdate = millis();
    while (g_motorA.isRunning() || g_motorB.isRunning() || g_motorC.isRunning()) {
        g_motorA.run();
        g_motorB.run();
        g_motorC.run();
        
        if (millis() - lastUpdate > 100) {
            g_state.lastCommandTime_ms = millis();
            lastUpdate = millis();
        }
    }
    
    g_state.currentPosition = *target;
    g_state.motorAngles.theta1 = g_kinematics.a;
    g_state.motorAngles.theta2 = g_kinematics.b;
    g_state.motorAngles.theta3 = g_kinematics.c;
    
    return true;
}

bool Motion_MoveToSafe(void) {
    Position_t safePos = {0.0f, 0.0f, g_state.safeHeight};
    return Motion_MoveTo(&safePos, true);
}

/*******************************************************************************
 * HOMING SYSTEM - AUTO CẢ 3 MOTOR
 ******************************************************************************/

bool Homing_Execute(void) {
    Serial.println(F("{\"homing\":\"start\"}"));
    
    g_state.state = STATE_HOMING;
    g_state.isHomed = false;
    
    Motion_SetProfile(PROFILE_HOMING);
    
    bool aHomed = false, bHomed = false, cHomed = false;
    uint32_t startTime = millis();
    
    while ((!aHomed || !bHomed || !cHomed) && 
           ((millis() - startTime) < TIMING_HOMING_TIMEOUT_MS)) {
        
        // ✅ Motor A
        if (!aHomed) {
            if (digitalRead(PIN_LIMIT_SWITCH_A) == LOW) {
                g_motorA.stop();
                g_motorA.setCurrentPosition(0);
                aHomed = true;
                Serial.println(F("{\"homing\":\"A_OK\"}"));
            } else {
                g_motorA.move(-50);
            }
        }
        
        // ✅ Motor B
        if (!bHomed) {
            if (digitalRead(PIN_LIMIT_SWITCH_B) == LOW) {
                g_motorB.stop();
                g_motorB.setCurrentPosition(0);
                bHomed = true;
                Serial.println(F("{\"homing\":\"B_OK\"}"));
            } else {
                g_motorB.move(-50);
            }
        }
        
        // ✅ Motor C
        if (!cHomed) {
            if (digitalRead(PIN_LIMIT_SWITCH_C) == LOW) {
                g_motorC.stop();
                g_motorC.setCurrentPosition(0);
                cHomed = true;
                Serial.println(F("{\"homing\":\"C_OK\"}"));
            } else {
                g_motorC.move(-50);
            }
        }
        
        g_motorA.run();
        g_motorB.run();
        g_motorC.run();
        
        g_state.lastCommandTime_ms = millis();
    }
    
    if (!aHomed || !bHomed || !cHomed) {
        Serial.println(F("{\"error\":\"homing_timeout\"}"));
        if (!aHomed) Serial.println(F("{\"failed\":\"A\"}"));
        if (!bHomed) Serial.println(F("{\"failed\":\"B\"}"));
        if (!cHomed) Serial.println(F("{\"failed\":\"C\"}"));
        g_state.state = STATE_ERROR;
        return false;
    }
    
    g_state.isHomed = true;
    
    g_state.motorAngles.theta1 = 0.0f;
    g_state.motorAngles.theta2 = 0.0f;
    g_state.motorAngles.theta3 = 0.0f;
    
    delay(300);
    
    Motion_SetProfile(PROFILE_SMOOTH);
    Motion_MoveToSafe();
    
    g_state.state = STATE_IDLE;
    Serial.println(F("{\"homing\":\"complete\"}"));
    
    return true;
}

/*******************************************************************************
 * VACUUM CONTROL
 ******************************************************************************/

void Vacuum_On(void) {
    digitalWrite(PIN_VACUUM_PUMP, HIGH);
    g_state.vacuumActive = true;
}

void Vacuum_Off(void) {
    digitalWrite(PIN_VACUUM_PUMP, LOW);
    g_state.vacuumActive = false;
}

/*******************************************************************************
 * SORTING OPERATION
 ******************************************************************************/

bool Sorting_Execute(const Position_t* pickPos, WasteType_t wasteType) {
    if (!g_state.isHomed) {
        Serial.println(F("{\"error\":\"not_homed\"}"));
        return false;
    }
    
    if (g_state.state != STATE_AUTO) {
        Serial.println(F("{\"error\":\"not_in_auto_mode\"}"));
        return false;
    }
    
    if (wasteType <= WASTE_TYPE_UNKNOWN || wasteType >= WASTE_TYPE_MAX) {
        Serial.println(F("{\"error\":\"invalid_waste\"}"));
        return false;
    }
    
    const WasteBin_t* bin = &g_bins[wasteType];
    if (!bin->isEnabled) {
        Serial.println(F("{\"error\":\"bin_disabled\"}"));
        return false;
    }
    
    g_state.currentWasteType = wasteType;
    g_state.cycleStartTime_ms = millis();
    g_state.state = STATE_SORTING;
    
    Serial.print(F("{\"sorting\":\""));
    switch(wasteType) {
        case WASTE_TYPE_PAPER: Serial.print(F("PAPER")); break;
        case WASTE_TYPE_PLASTIC: Serial.print(F("PLASTIC")); break;
        case WASTE_TYPE_METAL: Serial.print(F("METAL")); break;
        default: Serial.print(F("UNKNOWN"));
    }
    Serial.println(F("\"}"));
    
    Position_t tempPos;
    
    Motion_SetProfile(PROFILE_SMOOTH);
    tempPos = *pickPos;
    tempPos.z = g_state.safeHeight;
    if (!Motion_MoveTo(&tempPos, true)) return false;
    
    Motion_SetProfile(PROFILE_PRECISION);
    if (!Motion_MoveTo(pickPos, true)) return false;
    delay(TIMING_MOVE_SETTLE_MS);
    
    Vacuum_On();
    delay(TIMING_VACUUM_ON_MS);
    
    Motion_SetProfile(PROFILE_SMOOTH);
    tempPos = *pickPos;
    tempPos.z = g_state.safeHeight;
    if (!Motion_MoveTo(&tempPos, true)) return false;
    
    tempPos = bin->position;
    tempPos.z = g_state.safeHeight;
    if (!Motion_MoveTo(&tempPos, true)) return false;
    
    Motion_SetProfile(PROFILE_PRECISION);
    if (!Motion_MoveTo(&bin->position, true)) return false;
    delay(TIMING_MOVE_SETTLE_MS);
    
    Vacuum_Off();
    delay(TIMING_VACUUM_OFF_MS);
    
    Motion_SetProfile(PROFILE_SMOOTH);
    tempPos = bin->position;
    tempPos.z = g_state.safeHeight;
    if (!Motion_MoveTo(&tempPos, true)) return false;
    
    if (!Motion_MoveToSafe()) return false;
    
    uint16_t cycleTime = (uint16_t)(millis() - g_state.cycleStartTime_ms);
    g_state.lastCycleTime_ms = cycleTime;
    g_state.totalCycles++;
    
    uint32_t sum = (uint32_t)g_state.averageCycleTime_ms * (g_state.totalCycles - 1) + cycleTime;
    g_state.averageCycleTime_ms = (uint16_t)(sum / g_state.totalCycles);
    
    Serial.print(F("{\"cycle\":{\"time\":"));
    Serial.print(cycleTime / 1000.0f, 2);
    Serial.print(F(",\"avg\":"));
    Serial.print(g_state.averageCycleTime_ms / 1000.0f, 2);
    Serial.print(F(",\"total\":"));
    Serial.print(g_state.totalCycles);
    Serial.println(F("}}"));
    
    if (g_state.totalCycles % 10 == 0) {
        EEPROM_SaveConfig();
    }
    
    g_state.currentWasteType = WASTE_TYPE_UNKNOWN;
    g_state.state = STATE_AUTO;
    
    return true;
}

/*******************************************************************************
 * JOG COMMANDS
 ******************************************************************************/

bool Jog_Move(float dx, float dy, float dz) {
    if (g_state.state != STATE_TEACH) {
        Serial.println(F("{\"error\":\"not_in_teach_mode\"}"));
        return false;
    }
    
    if (!g_state.isHomed) {
        Serial.println(F("{\"error\":\"not_homed\"}"));
        return false;
    }
    
    Position_t target = g_state.currentPosition;
    target.x += dx;
    target.y += dy;
    target.z += dz;
    
    Motion_SetProfile(PROFILE_PRECISION);
    
    if (Motion_MoveTo(&target, true)) {
        Serial.print(F("{\"jog\":\"ok\",\"pos\":{\"x\":"));
        Serial.print(target.x, 1);
        Serial.print(F(",\"y\":"));
        Serial.print(target.y, 1);
        Serial.print(F(",\"z\":"));
        Serial.print(target.z, 1);
        Serial.println(F("}}"));
        return true;
    }
    
    return false;
}

/*******************************************************************************
 * COMMAND PARSER
 ******************************************************************************/

void Serial_ProcessCommand(const char* cmd) {
    while (*cmd == ' ') cmd++;
    
    g_state.lastCommandTime_ms = millis();
    
    // HOMING
    if (strncmp(cmd, "G28", 3) == 0) {
        Homing_Execute();
    }
    
    // MODE SWITCHING
    else if (strncmp(cmd, "SWITCH_MODE", 11) == 0) {
        const char* mode = cmd + 12;
        
        if (!g_state.isHomed) {
            Serial.println(F("{\"error\":\"not_homed\"}"));
        }
        else if (strncmp(mode, "TEACH", 5) == 0) {
            g_state.state = STATE_TEACH;
            Serial.println(F("{\"mode\":\"TEACH\"}"));
        }
        else if (strncmp(mode, "AUTO", 4) == 0) {
            g_state.state = STATE_AUTO;
            Serial.println(F("{\"mode\":\"AUTO\"}"));
        }
        else if (strncmp(mode, "IDLE", 4) == 0) {
            g_state.state = STATE_IDLE;
            Serial.println(F("{\"mode\":\"IDLE\"}"));
        }
        else {
            Serial.println(F("{\"error\":\"invalid_mode\"}"));
        }
    }
    
    // JOG
    else if (strncmp(cmd, "JOG", 3) == 0) {
        float dx = 0.0f, dy = 0.0f, dz = 0.0f;
        
        const char* ptr = cmd + 3;
        while (*ptr) {
            if (*ptr == 'X') dx = atof(ptr + 1);
            else if (*ptr == 'Y') dy = atof(ptr + 1);
            else if (*ptr == 'Z') dz = atof(ptr + 1);
            ptr++;
        }
        
        Jog_Move(dx, dy, dz);
    }
    
    // SET_BIN
    else if (strncmp(cmd, "SET_BIN", 7) == 0) {
        if (g_state.state != STATE_TEACH) {
            Serial.println(F("{\"error\":\"not_in_teach_mode\"}"));
        }
        else {
            char typeStr[16] = "";
            sscanf(cmd + 8, "%s", typeStr);
            
            WasteType_t type = WASTE_TYPE_UNKNOWN;
            if (strcmp(typeStr, "PAPER") == 0) type = WASTE_TYPE_PAPER;
            else if (strcmp(typeStr, "PLASTIC") == 0) type = WASTE_TYPE_PLASTIC;
            else if (strcmp(typeStr, "METAL") == 0) type = WASTE_TYPE_METAL;
            
            if (type > WASTE_TYPE_UNKNOWN && type < WASTE_TYPE_MAX) {
                g_bins[type].position = g_state.currentPosition;
                g_bins[type].isEnabled = true;
                
                Serial.print(F("{\"bin\":\""));
                Serial.print(typeStr);
                Serial.print(F("\",\"pos\":{\"x\":"));
                Serial.print(g_state.currentPosition.x, 1);
                Serial.print(F(",\"y\":"));
                Serial.print(g_state.currentPosition.y, 1);
                Serial.print(F(",\"z\":"));
                Serial.print(g_state.currentPosition.z, 1);
                Serial.println(F("}}"));
            }
        }
    }
    
    // GET_POS
    else if (strncmp(cmd, "GET_POS", 7) == 0) {
        Serial.print(F("{\"pos\":{\"x\":"));
        Serial.print(g_state.currentPosition.x, 2);
        Serial.print(F(",\"y\":"));
        Serial.print(g_state.currentPosition.y, 2);
        Serial.print(F(",\"z\":"));
        Serial.print(g_state.currentPosition.z, 2);
        Serial.println(F("}}"));
    }
    
    // SORT
    else if (strncmp(cmd, "SORT", 4) == 0) {
        Position_t pickPos = {0.0f, 0.0f, -220.0f};
        char typeStr[16] = "PAPER";
        
        const char* ptr = cmd + 4;
        while (*ptr) {
            if (*ptr == 'X') pickPos.x = atof(ptr + 1);
            else if (*ptr == 'Y') pickPos.y = atof(ptr + 1);
            else if (*ptr == 'Z') {
                float z = atof(ptr + 1);
                if (z != 0.0f) pickPos.z = z;
            }
            else if (*ptr == 'T') sscanf(ptr + 1, "%s", typeStr);
            ptr++;
        }
        
        WasteType_t type = WASTE_TYPE_UNKNOWN;
        if (strcmp(typeStr, "PAPER") == 0) {
            type = WASTE_TYPE_PAPER;
            if (pickPos.z == -220.0f) pickPos.z = g_state.pickupHeights[WASTE_TYPE_PAPER];
        }
        else if (strcmp(typeStr, "PLASTIC") == 0) {
            type = WASTE_TYPE_PLASTIC;
            if (pickPos.z == -220.0f) pickPos.z = g_state.pickupHeights[WASTE_TYPE_PLASTIC];
        }
        else if (strcmp(typeStr, "METAL") == 0) {
            type = WASTE_TYPE_METAL;
            if (pickPos.z == -220.0f) pickPos.z = g_state.pickupHeights[WASTE_TYPE_METAL];
        }
        
        Sorting_Execute(&pickPos, type);
    }
    
    // G1
    else if (strncmp(cmd, "G1", 2) == 0) {
        Position_t target = g_state.currentPosition;
        
        const char* ptr = cmd + 2;
        while (*ptr) {
            if (*ptr == 'X') target.x = atof(ptr + 1);
            else if (*ptr == 'Y') target.y = atof(ptr + 1);
            else if (*ptr == 'Z') target.z = atof(ptr + 1);
            ptr++;
        }
        
        Motion_SetProfile(PROFILE_SMOOTH);
        if (Motion_MoveTo(&target, true)) {
            Serial.println(F("{\"move\":\"ok\"}"));
        }
    }
    
    // M114
    else if (strncmp(cmd, "M114", 4) == 0) {
        Serial.print(F("{\"state\":\""));
        switch(g_state.state) {
            case STATE_IDLE: Serial.print(F("IDLE")); break;
            case STATE_HOMING: Serial.print(F("HOMING")); break;
            case STATE_TEACH: Serial.print(F("TEACH")); break;
            case STATE_AUTO: Serial.print(F("AUTO")); break;
            case STATE_SORTING: Serial.print(F("SORTING")); break;
            case STATE_ERROR: Serial.print(F("ERROR")); break;
            default: Serial.print(F("UNKNOWN"));
        }
        Serial.print(F("\",\"pos\":{\"x\":"));
        Serial.print(g_state.currentPosition.x, 1);
        Serial.print(F(",\"y\":"));
        Serial.print(g_state.currentPosition.y, 1);
        Serial.print(F(",\"z\":"));
        Serial.print(g_state.currentPosition.z, 1);
        Serial.print(F("},\"homed\":"));
        Serial.print(g_state.isHomed ? F("true") : F("false"));
        Serial.print(F(",\"vacuum\":"));
        Serial.print(g_state.vacuumActive ? F("true") : F("false"));
        Serial.print(F(",\"cycles\":"));
        Serial.print(g_state.totalCycles);
        Serial.println(F("}"));
    }
    
    // M17
    else if (strncmp(cmd, "M17", 3) == 0) {
        g_motorA.enableOutputs();
        g_motorB.enableOutputs();
        g_motorC.enableOutputs();
        g_state.motorsEnabled = true;
        Serial.println(F("{\"motors\":\"enabled\"}"));
    }
    
    // M18
    else if (strncmp(cmd, "M18", 3) == 0) {
        g_motorA.disableOutputs();
        g_motorB.disableOutputs();
        g_motorC.disableOutputs();
        Vacuum_Off();
        g_state.motorsEnabled = false;
        Serial.println(F("{\"motors\":\"disabled\"}"));
    }
    
    // M400
    else if (strncmp(cmd, "M400", 4) == 0) {
        const char* ptr = strchr(cmd, 'S');
        if (ptr) {
            if (atoi(ptr + 1) == 1) {
                Vacuum_On();
                Serial.println(F("{\"vacuum\":\"ON\"}"));
            } else {
                Vacuum_Off();
                Serial.println(F("{\"vacuum\":\"OFF\"}"));
            }
        }
    }
    
    // M500
    else if (strncmp(cmd, "M500", 4) == 0) {
        EEPROM_SaveConfig();
    }
    
    // M501
    else if (strncmp(cmd, "M501", 4) == 0) {
        EEPROM_LoadConfig();
    }
    
    else {
        Serial.println(F("{\"error\":\"unknown_cmd\"}"));
    }
}

/*******************************************************************************
 * SAFETY WATCHDOG
 ******************************************************************************/

void Safety_CheckWatchdog(void) {
    static uint32_t lastCheck = 0;
    
    if (millis() - lastCheck < 1000) return;
    lastCheck = millis();
    
    if (g_state.motorsEnabled) {
        if ((millis() - g_state.lastCommandTime_ms) > TIMING_WATCHDOG_MS) {
            Serial.println(F("{\"error\":\"watchdog\"}"));
            g_motorA.stop();
            g_motorB.stop();
            g_motorC.stop();
            Vacuum_Off();
            g_state.lastCommandTime_ms = millis();
        }
    }
    
    g_state.uptimeSeconds++;
}

/*******************************************************************************
 * ARDUINO SETUP
 ******************************************************************************/

void setup(void) {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) delay(10);
    
    pinMode(PIN_LIMIT_SWITCH_A, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_B, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_C, INPUT_PULLUP);
    pinMode(PIN_VACUUM_PUMP, OUTPUT);
    digitalWrite(PIN_VACUUM_PUMP, LOW);
    
    Motion_SetProfile(PROFILE_SMOOTH);
    
    g_motorA.enableOutputs();
    g_motorA.setPinsInverted(true, false, false);
    
    g_motorB.enableOutputs();
    g_motorB.setPinsInverted(true, false, false);
    
    g_motorC.enableOutputs();
    g_motorC.setPinsInverted(true, false, false);
    
    memset(&g_state, 0, sizeof(g_state));
    g_state.state = STATE_IDLE;
    g_state.motorsEnabled = true;
    g_state.lastCommandTime_ms = millis();
    
    EEPROM_LoadConfig();
    
    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║  DELTA ROBOT v8.3 - ALL SENSORS WORKING!            ║"));
    Serial.println(F("║  Tran Gia Bao - Thesis Project 2024-2025            ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
    Serial.println(F("✅ HARDWARE VERIFIED:"));
    Serial.println(F("  • Sensor A (Pin 19) - WORKING"));
    Serial.println(F("  • Sensor B (Pin 21) - WORKING"));
    Serial.println(F("  • Sensor C (Pin 24) - WORKING"));
    Serial.println(F(""));
    Serial.println(F("═══════════════════════════════════════════════════════"));
    Serial.println(F("COMMANDS:"));
    Serial.println(F("  G28               → Auto homing (A, B, C)"));
    Serial.println(F("  SWITCH_MODE TEACH → Enter teach mode"));
    Serial.println(F("  SWITCH_MODE AUTO  → Enter auto mode"));
    Serial.println(F("  M114              → Get status"));
    Serial.println(F("═══════════════════════════════════════════════════════"));
    Serial.println(F(""));
    
    Serial.print(F("{\"version\":\"8.3\",\"ready\":true,\"sram\":"));
    extern int __heap_start, *__brkval;
    int free_mem = ((int)&free_mem) - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
    Serial.print(free_mem);
    Serial.println(F("}"));
    
    Serial.println(F("Send G28 to start auto homing..."));
}

/*******************************************************************************
 * ARDUINO LOOP
 ******************************************************************************/

void loop(void) {
    Safety_CheckWatchdog();
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (g_cmdBufferIndex > 0) {
                g_cmdBuffer[g_cmdBufferIndex] = '\0';
                Serial_ProcessCommand(g_cmdBuffer);
                g_cmdBufferIndex = 0;
            }
        }
        else if (g_cmdBufferIndex < CMD_BUFFER_SIZE - 1) {
            g_cmdBuffer[g_cmdBufferIndex++] = c;
        }
        else {
            g_cmdBufferIndex = 0;
        }
    }
}

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/
