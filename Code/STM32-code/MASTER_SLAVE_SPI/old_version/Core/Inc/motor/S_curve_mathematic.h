/*
 * S_curve_mathematic.h
 *
 *  Created on: Oct 31, 2025
 *      Author: ostro
 */

#ifndef INC_MOTOR_S_CURVE_MATHEMATIC_H_
#define INC_MOTOR_S_CURVE_MATHEMATIC_H_

#ifndef SCURVE_TRAJECTORY_H
#define SCURVE_TRAJECTORY_H

#include "main.h"

// ==============================================================================
// TRAJECTORY POINT STRUCTURE
// ==============================================================================

/**
 * @brief Single trajectory point with time and velocity
 *
 * Time is stored as 16-bit milliseconds (0-65535 ms)
 * Velocity is stored as integer steps per second
 */
typedef struct {
    uint16_t time_ms;     // Time in milliseconds [ms] - 0 to 65535 ms
    int32_t velocity_raw; // Velocity in steps/second (integer value)
} TrajectoryPoint;

// ==============================================================================
// MOTOR CONFIGURATION STRUCTURE
// ==============================================================================

/**
 * @brief Motor and mechanical configuration
 *
 * Contains parameters for converting degrees to motor steps
 */
typedef struct {
    double gear_ratio;        // Gear ratio (e.g., 3.0 means 3:1 reduction)
    uint16_t microsteps;      // Microstepping setting (1, 2, 4, 8, 16, 32, etc.)
    uint16_t steps_per_rev;   // Steps per revolution of motor (typically 200 for 1.8° motor)
} MotorConfig;

// ==============================================================================
// SYSTEM CONFIGURATION STRUCTURE
// ==============================================================================

/**
 * @brief Complete system configuration including motion limits and motor params
 */
typedef struct {
    double max_velocity;      // Maximum velocity limit [deg/s]
    double max_acceleration;  // Maximum acceleration limit [deg/s²]
    double max_jerk;          // Maximum jerk limit [deg/s³]
    double current_angle;     // Current position angle [deg]
    int interpolation_points; // Number of interpolation points per phase
    MotorConfig motor;        // Motor configuration
} SystemConfig;

// ==============================================================================
// MOTION PROFILE TYPES
// ==============================================================================

/**
 * @brief Motion profile types according to Blejan article, Fig. 5
 */
typedef enum {
    PROFILE_A,    // Triangular acc, constant velocity phase (v_max < v_a; s > s_a)
    PROFILE_B,    // Triangular acc, no constant velocity (v_max > v_a; s < s_a)
    PROFILE_C1,   // Triangular acc, with gap (v_max < v_a; s > s_v)
    PROFILE_C2,   // Triangular acc, continuous (v_max < v_a; s < s_v)
    PROFILE_D1,   // Trapezoidal acc, with gap (v_max > v_a; s > s_v)
    PROFILE_D2    // Trapezoidal acc, continuous (v_max > v_a; s < s_v)
} ProfileType;

// ==============================================================================
// S-CURVE PROFILE PARAMETERS
// ==============================================================================

/**
 * @brief Internal S-curve profile parameters
 */
typedef struct {
    double t_j;           // Jerk phase duration [s]
    double t_a;           // Acceleration phase duration [s]
    double t_v;           // Time to start deceleration [s]
    double t1, t2, t3, t4, t5, t6, t7;  // Characteristic time points [s]
    double v_a;           // Max velocity without constant acceleration phase [deg/s]
    double s_a;           // Max distance without constant acceleration phase [deg]
    double s_v;           // Max distance without constant velocity phase [deg]
    ProfileType profile;  // Type of motion profile
    int direction;        // Direction flag: +1 (forward) or -1 (backward)
} ProfileParams;

// ==============================================================================
// GLOBAL CONSTANTS
// ==============================================================================

#define MAX_BUFFER_SIZE 1000  // Maximum number of trajectory points

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

/**
 * @brief Global trajectory buffer
 *
 * Contains generated trajectory points accessible via trajectory_buffer[i]
 */
extern TrajectoryPoint trajectory_buffer[MAX_BUFFER_SIZE];

/**
 * @brief Number of points currently in the trajectory buffer
 */
extern int buffer_count;

/**
 * @brief Global system configuration
 *
 * Contains all motion limits, current position, and motor parameters
 */
extern SystemConfig config;

// ==============================================================================
// CONVERSION FUNCTIONS
// ==============================================================================

/**
 * @brief Convert degrees per second to steps per second
 *
 * Uses current motor configuration to calculate steps/second
 * Formula: steps/s = (deg/s) * (steps_per_rev * microsteps * gear_ratio) / 360°
 *
 * @param deg_per_sec Velocity in degrees per second
 * @return Velocity in steps per second (rounded to nearest integer)
 */
int32_t deg_per_sec_to_steps_per_sec(double deg_per_sec);

/**
 * @brief Convert seconds to milliseconds with 16-bit limit
 *
 * Saturates at 65535 ms if input exceeds limit
 *
 * @param seconds Time in seconds
 * @return Time in milliseconds (0-65535)
 */
uint16_t sec_to_ms_limited(double seconds);

// ==============================================================================
// BUFFER MANAGEMENT FUNCTIONS
// ==============================================================================

/**
 * @brief Add a trajectory point to the buffer
 *
 * Automatically converts time to milliseconds and velocity to steps/second
 *
 * @param time_sec Time in seconds
 * @param velocity_deg_per_sec Velocity in degrees per second
 * @return true if point was added successfully, false if buffer is full
 */
bool add_trajectory_point(double time_sec, double velocity_deg_per_sec);

/**
 * @brief Clear all points from the trajectory buffer
 *
 * Resets buffer_count to 0
 */
void clear_trajectory_buffer(void);

// ==============================================================================
// LIMIT PARAMETERS CALCULATION
// ==============================================================================

/**
 * @brief Calculate maximum velocity without constant acceleration phase
 *
 * Formula (46) from Blejan article: v_a = a_max² / J
 *
 * @param a_max Maximum acceleration [deg/s²]
 * @param J Maximum jerk [deg/s³]
 * @return Maximum velocity v_a [deg/s]
 */
double calculate_v_a(double a_max, double J);

/**
 * @brief Calculate maximum distance without constant acceleration phase
 *
 * Formula (48) from Blejan article: s_a = 2*a_max³ / J²
 *
 * @param a_max Maximum acceleration [deg/s²]
 * @param J Maximum jerk [deg/s³]
 * @return Maximum distance s_a [deg]
 */
double calculate_s_a(double a_max, double J);

/**
 * @brief Calculate maximum distance without constant velocity phase
 *
 * Formulas (54) or (61) from Blejan article
 *
 * @param v_max Maximum velocity [deg/s]
 * @param a_max Maximum acceleration [deg/s²]
 * @param J Maximum jerk [deg/s³]
 * @return Maximum distance s_v [deg]
 */
double calculate_s_v(double v_max, double a_max, double J);

// ==============================================================================
// PROFILE TYPE DETERMINATION
// ==============================================================================

/**
 * @brief Determine the appropriate motion profile type
 *
 * Based on Section 3.2 of Blejan article
 *
 * @param s Distance to travel [deg]
 * @param v_a Maximum velocity without constant acceleration [deg/s]
 * @param s_a Maximum distance without constant acceleration [deg]
 * @param s_v Maximum distance without constant velocity [deg]
 * @param v_max Maximum velocity [deg/s]
 * @return Appropriate ProfileType for the motion
 */
ProfileType determine_profile_type(double s, double v_a, double s_a,
                                   double s_v, double v_max);

// ==============================================================================
// PROFILE PARAMETERS CALCULATION
// ==============================================================================

/**
 * @brief Calculate complete S-curve profile parameters
 *
 * Determines profile type and calculates all time points
 * Based on Section 3.3 of Blejan article
 *
 * @param target_angle Target position in degrees
 * @return Complete ProfileParams structure with all time points
 */
ProfileParams calculate_profile(double target_angle);

// ==============================================================================
// MOTION EQUATIONS
// ==============================================================================

/**
 * @brief Calculate motion parameters at a specific time
 *
 * Implements 7-phase S-curve motion equations from Section 2 of Blejan article
 *
 * @param t Time point [s]
 * @param params Pointer to profile parameters
 * @param position Output: position at time t [deg]
 * @param velocity Output: velocity at time t [deg/s]
 * @param acceleration Output: acceleration at time t [deg/s²]
 * @param jerk Output: jerk at time t [deg/s³]
 */
void calculate_motion_at_time(double t, ProfileParams *params,
                              double *position, double *velocity,
                              double *acceleration, double *jerk);

// ==============================================================================
// MAIN TRAJECTORY GENERATION FUNCTION
// ==============================================================================

/**
 * @brief Generate complete S-curve trajectory from current angle to target angle
 *
 * Main function that:
 * 1. Clears the trajectory buffer
 * 2. Calculates optimal profile parameters
 * 3. Generates interpolated trajectory points
 * 4. Fills the global trajectory_buffer with points
 * 5. Updates config.current_angle
 *
 * After successful generation:
 * - trajectory_buffer contains all points
 * - buffer_count contains number of points
 * - Each point has time_ms and velocity_raw
 *
 * @param target_angle Target position in degrees
 * @return true if trajectory was generated successfully, false otherwise
 *
 * @note Will return false if:
 *       - Target angle is too close to current angle (< 0.001 deg)
 *
 * @warning Will print warning if total time exceeds 65.535 seconds
 */
bool generate_scurve_trajectory(double target_angle);

// ==============================================================================
// UTILITY FUNCTIONS
// ==============================================================================

/**
 * @brief Print current motor configuration to stdout
 *
 * Displays:
 * - Gear ratio
 * - Microstepping setting
 * - Steps per revolution
 * - Total steps per output revolution
 */
void print_motor_config(void);

/**
 * @brief Set motor configuration parameters
 *
 * Updates the global config.motor structure
 *
 * @param gear_ratio Gear reduction ratio (e.g., 3.0 for 3:1)
 * @param microsteps Microstepping divisor (1, 2, 4, 8, 16, 32, etc.)
 * @param steps_per_rev Full steps per motor revolution (typically 200)
 *
 * @example
 * // Configure for NEMA17 motor with 16 microsteps and 5:1 gearbox
 * set_motor_config(5.0, 16, 200);
 */
void set_motor_config(double gear_ratio, uint16_t microsteps, uint16_t steps_per_rev);

// ==============================================================================
// USAGE EXAMPLE
// ==============================================================================

/**
 * @example Basic Usage
 *
 * // 1. Configure system
 * config.max_velocity = 100.0;      // 100 deg/s
 * config.max_acceleration = 200.0;  // 200 deg/s²
 * config.max_jerk = 1000.0;         // 1000 deg/s³
 * config.interpolation_points = 10;
 *
 * // 2. Configure motor
 * set_motor_config(3.0, 16, 200);  // 3:1 gear, 16 microsteps, 200 steps/rev
 *
 * // 3. Generate trajectory
 * if (generate_scurve_trajectory(90.0)) {
 *     // 4. Use trajectory points
 *     for (int i = 0; i < buffer_count; i++) {
 *         uint16_t time_ms = trajectory_buffer[i].time_ms;
 *         int32_t velocity = trajectory_buffer[i].velocity_raw;
 *         // Send to motor controller...
 *     }
 * }
 */

#endif // SCURVE_TRAJECTORY_H

#endif /* INC_MOTOR_S_CURVE_MATHEMATIC_H_ */
