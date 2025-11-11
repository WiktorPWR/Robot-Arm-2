 /*
 * S_curve_mathematic.c
 *
 *  Created on: Oct 31, 2025
 *      Author: ostro
 */

#include "main.h"
// ==============================================================================
// TRAJECTORY POINT STRUCTURE (MODIFIED - 16-bit time in ms, integer velocity)
// ==============================================================================
typedef struct {
    uint16_t time_ms;     // Time in milliseconds [ms] - 0 to 65535 ms
    int32_t velocity_raw; // Velocity in steps/second (integer value)
} TrajectoryPoint;

// ==============================================================================
// MOTOR CONFIGURATION STRUCTURE (NEW)
// ==============================================================================
typedef struct {
    double gear_ratio;        // Gear ratio (e.g., 3.0 means 3:1 reduction)
    uint16_t microsteps;      // Microstepping setting (1, 2, 4, 8, 16, 32, etc.)
    uint16_t steps_per_rev;   // Steps per revolution of motor (typically 200 for 1.8° motor)
} MotorConfig;

// ==============================================================================
// SYSTEM CONFIGURATION STRUCTURE (MODIFIED)
// ==============================================================================
typedef struct {
    double max_velocity;      // Maximum velocity limit [deg/s]
    double max_acceleration;  // Maximum acceleration limit [deg/s²]
    double max_jerk;          // Maximum jerk limit [deg/s³]
    double current_angle;     // Current position angle [deg]
    int interpolation_points; // Number of interpolation points per phase
    MotorConfig motor;        // Motor configuration
} SystemConfig;

// ==============================================================================
// GLOBAL TRAJECTORY BUFFER
// ==============================================================================
#define MAX_BUFFER_SIZE 1000
TrajectoryPoint trajectory_buffer[MAX_BUFFER_SIZE];
int buffer_count = 0;

// ==============================================================================
// DEFAULT GLOBAL CONFIGURATION
// ==============================================================================
SystemConfig config = {
    .max_velocity = 100.0,        // 100 deg/s
    .max_acceleration = 200.0,    // 200 deg/s²
    .max_jerk = 1000.0,           // 1000 deg/s³
    .current_angle = 0.0,         // Starting at 0 degrees
    .interpolation_points = 10,   // 10 points per phase
    .motor = {
        .gear_ratio = 1.0,        // No gear reduction by default
        .microsteps = 16,         // 16 microsteps
        .steps_per_rev = 200      // 200 steps/rev (1.8° motor)
    }
};

// ==============================================================================
// CONVERSION FUNCTIONS
// ==============================================================================

// Convert degrees/second to steps/second
int32_t deg_per_sec_to_steps_per_sec(double deg_per_sec) {
    // steps_per_sec = (deg/s) * (steps_per_rev * microsteps) / 360° * gear_ratio
    double steps_per_degree = (config.motor.steps_per_rev * config.motor.microsteps * config.motor.gear_ratio) / 360.0;
    return (int32_t)round(deg_per_sec * steps_per_degree);
}

// Convert seconds to milliseconds (with 16-bit limit check)
uint16_t sec_to_ms_limited(double seconds) {
    double ms = seconds * 1000.0;
    if (ms > 65535.0) {
        return 65535; // Saturate at max 16-bit value
    }
    return (uint16_t)round(ms);
}

// ==============================================================================
// MOTION PROFILE TYPES (According to Blejan article, Fig. 5)
// ==============================================================================
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
// BUFFER MANAGEMENT FUNCTIONS (MODIFIED)
// ==============================================================================

bool add_trajectory_point(double time_sec, double velocity_deg_per_sec) {
    if (buffer_count >= MAX_BUFFER_SIZE) {
        return false;
    }

    // Convert time to milliseconds (16-bit)
    trajectory_buffer[buffer_count].time_ms = sec_to_ms_limited(time_sec);

    // Convert velocity to steps/second (integer)
    trajectory_buffer[buffer_count].velocity_raw = deg_per_sec_to_steps_per_sec(velocity_deg_per_sec);

    buffer_count++;
    return true;
}

void clear_trajectory_buffer() {
    buffer_count = 0;
}

// ==============================================================================
// LIMIT PARAMETERS CALCULATION (Section 3.1 of Blejan article)
// ==============================================================================

double calculate_v_a(double a_max, double J) {
    return (a_max * a_max) / J;
}

double calculate_s_a(double a_max, double J) {
    return (2.0 * pow(a_max, 3)) / (J * J);
}

double calculate_s_v(double v_max, double a_max, double J) {
    if (v_max * J < a_max * a_max) {
        return 2.0 * v_max * sqrt(v_max / J);
    } else {
        return v_max * (v_max / a_max + a_max / J);
    }
}

// ==============================================================================
// PROFILE TYPE DETERMINATION (Section 3.2 of Blejan article)
// ==============================================================================

ProfileType determine_profile_type(double s, double v_a, double s_a, double s_v, double v_max) {
    if (v_max < v_a) {
        if (s > s_a) {
            return PROFILE_A;
        } else if (s > s_v) {
            return PROFILE_C1;
        } else {
            return PROFILE_C2;
        }
    } else {
        if (s < s_a) {
            return PROFILE_B;
        } else if (s > s_v) {
            return PROFILE_D1;
        } else {
            return PROFILE_D2;
        }
    }
}

// ==============================================================================
// PROFILE PARAMETERS CALCULATION (Section 3.3 of Blejan article)
// ==============================================================================

ProfileParams calculate_profile(double target_angle) {
    ProfileParams params = {0};

    double angle_delta = target_angle - config.current_angle;
    double s = fabs(angle_delta);
    params.direction = (angle_delta >= 0) ? 1 : -1;

    double J = config.max_jerk;
    double a_max = config.max_acceleration;
    double v_max = config.max_velocity;

    params.v_a = calculate_v_a(a_max, J);
    params.s_a = calculate_s_a(a_max, J);
    params.s_v = calculate_s_v(v_max, a_max, J);

    params.profile = determine_profile_type(s, params.v_a, params.s_a, params.s_v, v_max);

    switch (params.profile) {
        case PROFILE_B:
        case PROFILE_C2:
            params.t_j = pow(s / (2.0 * J), 1.0/3.0);
            params.t_a = params.t_j;
            params.t_v = 2.0 * params.t_j;
            break;

        case PROFILE_A:
        case PROFILE_C1:
            params.t_j = sqrt(v_max / J);
            params.t_v = s / v_max;
            params.t_a = params.t_j;
            break;

        case PROFILE_D1:
            params.t_j = a_max / J;
            params.t_a = v_max / a_max;
            params.t_v = s / v_max;
            break;

        case PROFILE_D2:
            params.t_j = a_max / J;
            params.t_a = 0.5 * (sqrt((4.0 * s * J * J + a_max * a_max * a_max) /
                        (a_max * J * J)) - a_max / J);
            params.t_v = params.t_a + params.t_j;
            break;
    }

    params.t1 = params.t_j;
    params.t2 = params.t_a;
    params.t3 = params.t_j + params.t_a;
    params.t4 = params.t_v;
    params.t5 = params.t_j + params.t_v;
    params.t6 = params.t_v + params.t_a;
    params.t7 = params.t_v + params.t_a + params.t_j;

    return params;
}

// ==============================================================================
// MOTION EQUATIONS (Section 2 of Blejan article)
// ==============================================================================

void calculate_motion_at_time(double t, ProfileParams *params,
                             double *position, double *velocity,
                             double *acceleration, double *jerk) {
    double J = config.max_jerk * params->direction;

    *jerk = 0;
    *acceleration = 0;
    *velocity = 0;
    *position = 0;

    if (t >= 0 && t < params->t1) {
        *jerk = J;
        *acceleration = J * t;
        *velocity = J * t * t / 2.0;
        *position = J * t * t * t / 6.0;
    }
    else if (t >= params->t1 && t < params->t2) {
        double dt = t - params->t1;
        double a1 = J * params->t1;
        double v1 = J * params->t1 * params->t1 / 2.0;
        double p1 = J * params->t1 * params->t1 * params->t1 / 6.0;

        *jerk = 0;
        *acceleration = a1;
        *velocity = v1 + a1 * dt;
        *position = p1 + v1 * dt + a1 * dt * dt / 2.0;
    }
    else if (t >= params->t2 && t < params->t3) {
        double dt = t - params->t2;
        double a2 = J * params->t1;
        double v2 = J * params->t1 * params->t1 / 2.0 + a2 * (params->t2 - params->t1);
        double p2 = J * params->t1 * params->t1 * params->t1 / 6.0 +
                   J * params->t1 * params->t1 / 2.0 * (params->t2 - params->t1) +
                   a2 * (params->t2 - params->t1) * (params->t2 - params->t1) / 2.0;

        *jerk = -J;
        *acceleration = a2 - J * dt;
        *velocity = v2 + a2 * dt - J * dt * dt / 2.0;
        *position = p2 + v2 * dt + a2 * dt * dt / 2.0 - J * dt * dt * dt / 6.0;
    }
    else if (t >= params->t3 && t < params->t4) {
        double dt = t - params->t3;

        double a2 = J * params->t1;
        double v2 = J * params->t1 * params->t1 / 2.0 + a2 * (params->t2 - params->t1);
        double p2 = J * params->t1 * params->t1 * params->t1 / 6.0 +
                   J * params->t1 * params->t1 / 2.0 * (params->t2 - params->t1) +
                   a2 * (params->t2 - params->t1) * (params->t2 - params->t1) / 2.0;

        double dt23 = params->t3 - params->t2;
        double v3 = v2 + a2 * dt23 - J * dt23 * dt23 / 2.0;
        double p3 = p2 + v2 * dt23 + a2 * dt23 * dt23 / 2.0 - J * dt23 * dt23 * dt23 / 6.0;

        *jerk = 0;
        *acceleration = 0;
        *velocity = v3;
        *position = p3 + v3 * dt;
    }
    else if (t >= params->t4 && t < params->t5) {
        double dt = t - params->t4;

        double a2 = J * params->t1;
        double v2 = J * params->t1 * params->t1 / 2.0 + a2 * (params->t2 - params->t1);
        double p2 = J * params->t1 * params->t1 * params->t1 / 6.0 +
                   J * params->t1 * params->t1 / 2.0 * (params->t2 - params->t1) +
                   a2 * (params->t2 - params->t1) * (params->t2 - params->t1) / 2.0;

        double dt23 = params->t3 - params->t2;
        double v3 = v2 + a2 * dt23 - J * dt23 * dt23 / 2.0;
        double p3 = p2 + v2 * dt23 + a2 * dt23 * dt23 / 2.0 - J * dt23 * dt23 * dt23 / 6.0;

        double v4 = v3;
        double p4 = p3 + v3 * (params->t4 - params->t3);

        *jerk = -J;
        *acceleration = -J * dt;
        *velocity = v4 - J * dt * dt / 2.0;
        *position = p4 + v4 * dt - J * dt * dt * dt / 6.0;
    }
    else if (t >= params->t5 && t < params->t6) {
        double dt = t - params->t5;

        double a2 = J * params->t1;
        double v2 = J * params->t1 * params->t1 / 2.0 + a2 * (params->t2 - params->t1);
        double p2 = J * params->t1 * params->t1 * params->t1 / 6.0 +
                   J * params->t1 * params->t1 / 2.0 * (params->t2 - params->t1) +
                   a2 * (params->t2 - params->t1) * (params->t2 - params->t1) / 2.0;

        double dt23 = params->t3 - params->t2;
        double v3 = v2 + a2 * dt23 - J * dt23 * dt23 / 2.0;
        double p3 = p2 + v2 * dt23 + a2 * dt23 * dt23 / 2.0 - J * dt23 * dt23 * dt23 / 6.0;

        double v4 = v3;
        double p4 = p3 + v3 * (params->t4 - params->t3);

        double dt45 = params->t5 - params->t4;
        double a5 = -J * dt45;
        double v5 = v4 - J * dt45 * dt45 / 2.0;
        double p5 = p4 + v4 * dt45 - J * dt45 * dt45 * dt45 / 6.0;

        *jerk = 0;
        *acceleration = a5;
        *velocity = v5 + a5 * dt;
        *position = p5 + v5 * dt + a5 * dt * dt / 2.0;
    }
    else if (t >= params->t6 && t < params->t7) {
        double dt = t - params->t6;

        double a2 = J * params->t1;
        double v2 = J * params->t1 * params->t1 / 2.0 + a2 * (params->t2 - params->t1);
        double p2 = J * params->t1 * params->t1 * params->t1 / 6.0 +
                   J * params->t1 * params->t1 / 2.0 * (params->t2 - params->t1) +
                   a2 * (params->t2 - params->t1) * (params->t2 - params->t1) / 2.0;

        double dt23 = params->t3 - params->t2;
        double v3 = v2 + a2 * dt23 - J * dt23 * dt23 / 2.0;
        double p3 = p2 + v2 * dt23 + a2 * dt23 * dt23 / 2.0 - J * dt23 * dt23 * dt23 / 6.0;

        double v4 = v3;
        double p4 = p3 + v3 * (params->t4 - params->t3);

        double dt45 = params->t5 - params->t4;
        double a5 = -J * dt45;
        double v5 = v4 - J * dt45 * dt45 / 2.0;
        double p5 = p4 + v4 * dt45 - J * dt45 * dt45 * dt45 / 6.0;

        double dt56 = params->t6 - params->t5;
        double a6 = a5;
        double v6 = v5 + a5 * dt56;
        double p6 = p5 + v5 * dt56 + a5 * dt56 * dt56 / 2.0;

        *jerk = J;
        *acceleration = a6 + J * dt;
        *velocity = v6 + a6 * dt + J * dt * dt / 2.0;
        *position = p6 + v6 * dt + a6 * dt * dt / 2.0 + J * dt * dt * dt / 6.0;
    }
    else {
        *jerk = 0;
        *acceleration = 0;
        *velocity = 0;
        *position = fabs(config.current_angle);
    }
}

// ==============================================================================
// MAIN TRAJECTORY GENERATION FUNCTION
// ==============================================================================

bool generate_scurve_trajectory(double target_angle) {
    clear_trajectory_buffer();

    double angle_delta = target_angle - config.current_angle;

    if (fabs(angle_delta) < 0.001) {
        return false;
    }

    ProfileParams params = calculate_profile(target_angle);

    // Check if total time exceeds 16-bit millisecond limit
//    if (params.t7 * 1000.0 > 65535.0) {
//        printf("Warning: Total trajectory time (%.2f s) exceeds 16-bit ms limit (65.535 s)\n", params.t7);
//        // You might want to return false or adjust parameters here
//    }

    int phase_points = config.interpolation_points;

    // Phase 1: [0, t1) - Jerk-up
    for (int i = 0; i <= phase_points; i++) {
        double t = params.t1 * i / (double)phase_points;
        double pos, vel, acc, jrk;
        calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(t, vel);
    }

    // Phase 2: [t1, t2) - Constant acceleration
    if (params.t2 > params.t1 + 0.0001) {
        for (int i = 1; i <= phase_points; i++) {
            double t = params.t1 + (params.t2 - params.t1) * i / (double)phase_points;
            double pos, vel, acc, jrk;
            calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
            add_trajectory_point(t, vel);
        }
    }

    // Phase 3: [t2, t3) - Jerk-down
    for (int i = 1; i <= phase_points; i++) {
        double t = params.t2 + (params.t3 - params.t2) * i / (double)phase_points;
        double pos, vel, acc, jrk;
        calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(t, vel);
    }

    // Phase 4: [t3, t4) - Constant velocity
    if (params.t4 > params.t3 + 0.0001) {
        double pos, vel, acc, jrk;
        calculate_motion_at_time(params.t3, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(params.t3, vel);
        calculate_motion_at_time(params.t4, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(params.t4, vel);
    }

    // Phase 5: [t4, t5) - Jerk-down (deceleration)
    for (int i = 1; i <= phase_points; i++) {
        double t = params.t4 + (params.t5 - params.t4) * i / (double)phase_points;
        double pos, vel, acc, jrk;
        calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(t, vel);
    }

    // Phase 6: [t5, t6) - Constant deceleration
    if (params.t6 > params.t5 + 0.0001) {
        for (int i = 1; i <= phase_points; i++) {
            double t = params.t5 + (params.t6 - params.t5) * i / (double)phase_points;
            double pos, vel, acc, jrk;
            calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
            add_trajectory_point(t, vel);
        }
    }

    // Phase 7: [t6, t7) - Jerk-up (end)
    for (int i = 1; i <= phase_points + 1; i++) {
        double t = params.t6 + (params.t7 - params.t6) * i / (double)phase_points;
        if (t > params.t7) t = params.t7;
        double pos, vel, acc, jrk;
        calculate_motion_at_time(t, &params, &pos, &vel, &acc, &jrk);
        add_trajectory_point(t, vel);
    }

    config.current_angle = target_angle;

    return true;
}

void set_motor_config(double gear_ratio, uint16_t microsteps, uint16_t steps_per_rev) {
    if (gear_ratio <= 0.0 || microsteps == 0 || steps_per_rev == 0) {
        return;
    }

    config.motor.gear_ratio = gear_ratio;
    config.motor.microsteps = microsteps;
    config.motor.steps_per_rev = steps_per_rev;
}







