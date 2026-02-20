/*
 * cascade_controller.c
 *
 *  Created on: Nov 24, 2025
 *      Author: ostro
 *
 *  Implementacja kaskadowej struktury regulatorów PD (pozycja) + PID (prędkość)
 *  dla sterowania silnikiem krokowym manipulatora.
 *
 *  Struktura kaskadowa:
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │  θ_ref  ──►[PD]──► v_ref ──►[PID]──► f_step ──►[STEROWNIK]──► SILNIK│
 *  │            ▲                 ▲                                  │   │
 *  │            │                 │                                  │   │
 *  │            └─── θ_actual ────┴─── v_actual ─────────────────────┘   │
 *  │                (enkoder)          (enkoder)                         │
 *  └─────────────────────────────────────────────────────────────────────┘
 */

#include "cascade_controller.h"
#include "motor/motor_functions.h"
#include <math.h>
#include <string.h>

/* ==========================================================
 *  ZEWNĘTRZNE ZALEŻNOŚCI
 * ========================================================== */

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

/* ==========================================================
 *  ZMIENNE GLOBALNE
 * ========================================================== */

/**
 * @brief Globalny obiekt kaskadowego regulatora
 */
CascadeController cascade_ctrl;

/* ==========================================================
 *  ZMIENNE LOKALNE DO SYMULACJI/TESTÓW
 * ========================================================== */

// Symulowana pozycja (do testów bez enkodera)
static float simulated_position_deg = 0.0f;
static float simulated_velocity_deg_s = 0.0f;

/* ==========================================================
 *  IMPLEMENTACJA FUNKCJI ODCZYTU CZUJNIKÓW
 *  (Placeholder - do dostosowania przez użytkownika)
 * ========================================================== */

/**
 * @brief Odczyt aktualnej pozycji z enkodera
 *
 * TODO: Zaimplementować odczyt z rzeczywistego enkodera
 * Dla osi 1-3: enkoder inkrementalny z procedurą homingu
 * Dla osi 4-6: enkoder absolutny AS5600 (I2C)
 */
__attribute__((weak)) float actual_position_read(void)
{
    // Placeholder - symulacja pozycji na podstawie licznika kroków
    // W rzeczywistej implementacji: odczyt z enkodera AS5600 lub inkrementalnego
    return steps_to_degrees(cascade_ctrl.state.step_counter);
}

/**
 * @brief Odczyt aktualnej prędkości z enkodera
 *
 * TODO: Zaimplementować odczyt z rzeczywistego enkodera
 * Enkoder inkrementalny 600 impulsów/obrót
 */
__attribute__((weak)) float actual_velocity_read(void)
{
    // Placeholder - symulacja prędkości na podstawie częstotliwości kroków
    // W rzeczywistej implementacji: obliczenie z różnicy pozycji enkodera
    return step_frequency_to_velocity(cascade_ctrl.state.step_frequency_hz);
}

/* ==========================================================
 *  FUNKCJE INICJALIZACJI
 * ========================================================== */

void cascade_controller_init(void)
{
    // Wyzerowanie całej struktury
    memset(&cascade_ctrl, 0, sizeof(CascadeController));

    // ===== Domyślne parametry regulatora PD (pętla pozycji) =====
    // Dobrane eksperymentalnie dla silnika krokowego z przekładnią
    cascade_ctrl.position_controller.Kp = 5.0f;     // Wzmocnienie proporcjonalne
    cascade_ctrl.position_controller.Kd = 0.1f;     // Wzmocnienie różniczkujące (tłumienie)
    cascade_ctrl.position_controller.prev_error = 0.0f;
    cascade_ctrl.position_controller.output_limit = CTRL_MAX_VELOCITY_DEG_S;

    // ===== Domyślne parametry regulatora PID (pętla prędkości) =====
    // Człon całkujący eliminuje błąd ustalony prędkości
    cascade_ctrl.velocity_controller.Kp = 10.0f;    // Wzmocnienie proporcjonalne
    cascade_ctrl.velocity_controller.Ki = 2.0f;     // Wzmocnienie całkujące
    cascade_ctrl.velocity_controller.Kd = 0.01f;    // Wzmocnienie różniczkujące
    cascade_ctrl.velocity_controller.integral = 0.0f;
    cascade_ctrl.velocity_controller.prev_error = 0.0f;
    cascade_ctrl.velocity_controller.integral_limit = 1000.0f;  // Anty-windup
    cascade_ctrl.velocity_controller.output_limit = CTRL_MAX_STEP_FREQ;

    // ===== Inicjalizacja stanu =====
    cascade_ctrl.state.target_position_deg = 0.0f;
    cascade_ctrl.state.actual_position_deg = 0.0f;
    cascade_ctrl.state.actual_velocity_deg_s = 0.0f;
    cascade_ctrl.state.velocity_setpoint_deg_s = 0.0f;
    cascade_ctrl.state.step_frequency_hz = 0.0f;
    cascade_ctrl.state.step_counter = 0;
    cascade_ctrl.state.target_steps = 0;
    cascade_ctrl.state.is_moving = false;
    cascade_ctrl.state.position_reached = true;
    cascade_ctrl.state.controller_enabled = false;
    cascade_ctrl.state.direction = 1;

    // Inicjalizacja timestampów
    cascade_ctrl.last_position_update_ms = HAL_GetTick();
    cascade_ctrl.last_velocity_update_ms = HAL_GetTick();
}

void cascade_set_position_gains(float Kp, float Kd)
{
    cascade_ctrl.position_controller.Kp = Kp;
    cascade_ctrl.position_controller.Kd = Kd;
}

void cascade_set_velocity_gains(float Kp, float Ki, float Kd)
{
    cascade_ctrl.velocity_controller.Kp = Kp;
    cascade_ctrl.velocity_controller.Ki = Ki;
    cascade_ctrl.velocity_controller.Kd = Kd;
}

/* ==========================================================
 *  IMPLEMENTACJA REGULATORÓW
 * ========================================================== */

/**
 * @brief Obliczenie wyjścia regulatora PD (pętla pozycji)
 *
 * Równanie:
 * u_pos(t) = Kp * e_pos(t) + Kd * (de_pos/dt)
 *
 * Uzasadnienie braku członu I:
 * - W układach gdzie sygnał sterujący wpływa na prędkość (nie moment),
 *   człon całkujący może powodować integrator wind-up
 * - Człon D pełni rolę tłumienia, redukując przeregulowania
 */
float pd_controller_update(PD_Controller *pd, float setpoint, float measurement, float dt)
{
    // Obliczenie błędu pozycji: e_pos = θ_ref - θ_actual
    float error = setpoint - measurement;

    // Człon proporcjonalny: P = Kp * e_pos
    float p_term = pd->Kp * error;

    // Człon różniczkujący: D = Kd * (de_pos/dt)
    // Aproksymacja pochodnej metodą różnic wstecznych
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pd->prev_error) / dt;
    }
    float d_term = pd->Kd * derivative;

    // Zapisanie błędu dla następnej iteracji
    pd->prev_error = error;

    // Suma: u_pos = P + D
    float output = p_term + d_term;

    // Ograniczenie wyjścia (saturacja)
    if (output > pd->output_limit) {
        output = pd->output_limit;
    } else if (output < -pd->output_limit) {
        output = -pd->output_limit;
    }

    return output;
}

/**
 * @brief Obliczenie wyjścia regulatora PID (pętla prędkości)
 *
 * Równanie:
 * u_vel(t) = Kp_v * e_vel(t) + Ki_v * ∫e_vel(τ)dτ + Kd_v * (de_vel/dt)
 *
 * Uzasadnienie członu I:
 * - Eliminuje błąd ustalony prędkości wynikający z tarcia
 * - Kompensuje zmienne obciążenia mechaniczne
 * - Niweluje niedokładności przekładni
 */
float pid_controller_update(PID_Controller *pid, float setpoint, float measurement, float dt)
{
    // Obliczenie błędu prędkości: e_vel = v_ref - v_actual
    float error = setpoint - measurement;

    // Człon proporcjonalny: P = Kp * e_vel
    float p_term = pid->Kp * error;

    // Człon całkujący: I = Ki * ∫e_vel dt
    // Całkowanie metodą prostokątów (Euler)
    pid->integral += error * dt;

    // Anty-windup: ograniczenie akumulatora całki
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->Ki * pid->integral;

    // Człon różniczkujący: D = Kd * (de_vel/dt)
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
    float d_term = pid->Kd * derivative;

    // Zapisanie błędu dla następnej iteracji
    pid->prev_error = error;

    // Suma: u_vel = P + I + D
    float output = p_term + i_term + d_term;

    // Ograniczenie wyjścia (częstotliwość kroków)
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}

void pid_controller_reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pd_controller_reset(PD_Controller *pd)
{
    pd->prev_error = 0.0f;
}

/* ==========================================================
 *  GŁÓWNE FUNKCJE STEROWANIA
 * ========================================================== */

bool cascade_move_to_position(float target_angle)
{
    // Reset regulatorów przed nowym ruchem
    pd_controller_reset(&cascade_ctrl.position_controller);
    pid_controller_reset(&cascade_ctrl.velocity_controller);

    // Odczyt aktualnej pozycji
    cascade_ctrl.state.actual_position_deg = actual_position_read();

    // Ustawienie celu
    cascade_ctrl.state.target_position_deg = target_angle;

    // Obliczenie kierunku ruchu
    float position_error = target_angle - cascade_ctrl.state.actual_position_deg;
    if (position_error >= 0) {
        cascade_ctrl.state.direction = 1;   // Ruch w prawo (CW)
    } else {
        cascade_ctrl.state.direction = -1;  // Ruch w lewo (CCW)
    }

    // Ustawienie kierunku na pinie GPIO
    set_motor_direction(cascade_ctrl.state.direction);

    // Obliczenie docelowej liczby kroków
    cascade_ctrl.state.target_steps = degrees_to_steps(target_angle);

    // Włączenie regulatora
    cascade_ctrl.state.is_moving = true;
    cascade_ctrl.state.position_reached = false;
    cascade_ctrl.state.controller_enabled = true;

    // Inicjalizacja timestampów
    cascade_ctrl.last_position_update_ms = HAL_GetTick();
    cascade_ctrl.last_velocity_update_ms = HAL_GetTick();

    // Start generowania kroków
    start_step_generation();

    return true;
}

/**
 * @brief Główna funkcja aktualizacji regulatora kaskadowego
 *
 * Struktura kaskadowa zapewnia:
 * - Separację dynamiki: pętla prędkości działa szybciej niż pętla pozycji
 * - Stabilność: wewnętrzny PID stabilizuje dynamikę prędkości
 * - Eliminację błędu ustalonego: człon I w pętli prędkości
 * - Redukcję oscylacji: człon D w pętli pozycji
 */
bool cascade_controller_update(void)
{
    if (!cascade_ctrl.state.controller_enabled || !cascade_ctrl.state.is_moving) {
        return false;
    }

    uint32_t current_time_ms = HAL_GetTick();

    // ===== PĘTLA ZEWNĘTRZNA (POZYCJA) - wolniejsza =====
    // Okres: CTRL_POSITION_LOOP_PERIOD_MS (np. 10ms)
    if ((current_time_ms - cascade_ctrl.last_position_update_ms) >= CTRL_POSITION_LOOP_PERIOD_MS) {

        float dt_pos = (current_time_ms - cascade_ctrl.last_position_update_ms) / 1000.0f;
        cascade_ctrl.last_position_update_ms = current_time_ms;

        // Odczyt aktualnej pozycji z enkodera
        cascade_ctrl.state.actual_position_deg = actual_position_read();

        // Obliczenie regulatora PD pozycji
        // Wyjście: zadana prędkość dla wewnętrznej pętli
        cascade_ctrl.state.velocity_setpoint_deg_s = pd_controller_update(
            &cascade_ctrl.position_controller,
            cascade_ctrl.state.target_position_deg,
            cascade_ctrl.state.actual_position_deg,
            dt_pos
        );

        // Sprawdzenie warunku zakończenia ruchu
        float position_error = fabsf(cascade_ctrl.state.target_position_deg -
                                     cascade_ctrl.state.actual_position_deg);

        if (position_error < CTRL_POSITION_TOLERANCE_DEG) {
            // Pozycja osiągnięta
            cascade_ctrl.state.position_reached = true;
            cascade_ctrl.state.is_moving = false;
            cascade_controller_stop();
            return false;
        }
    }

    // ===== PĘTLA WEWNĘTRZNA (PRĘDKOŚĆ) - szybsza =====
    // Okres: CTRL_VELOCITY_LOOP_PERIOD_MS (np. 1ms)
    if ((current_time_ms - cascade_ctrl.last_velocity_update_ms) >= CTRL_VELOCITY_LOOP_PERIOD_MS) {

        float dt_vel = (current_time_ms - cascade_ctrl.last_velocity_update_ms) / 1000.0f;
        cascade_ctrl.last_velocity_update_ms = current_time_ms;

        // Odczyt aktualnej prędkości z enkodera
        cascade_ctrl.state.actual_velocity_deg_s = actual_velocity_read();

        // Obliczenie regulatora PID prędkości
        // Wyjście: częstotliwość kroków dla sterownika silnika
        float step_freq = pid_controller_update(
            &cascade_ctrl.velocity_controller,
            cascade_ctrl.state.velocity_setpoint_deg_s,
            cascade_ctrl.state.actual_velocity_deg_s,
            dt_vel
        );

        // Wartość bezwzględna częstotliwości (kierunek ustawiony osobno)
        cascade_ctrl.state.step_frequency_hz = fabsf(step_freq);

        // Aktualizacja kierunku na podstawie znaku wyjścia
        if (step_freq >= 0) {
            set_motor_direction(1);
        } else {
            set_motor_direction(-1);
        }

        // Ograniczenie minimalnej częstotliwości
        if (cascade_ctrl.state.step_frequency_hz < CTRL_MIN_STEP_FREQ) {
            cascade_ctrl.state.step_frequency_hz = 0;
            stop_step_generation();
        } else {
            // Ustawienie nowej częstotliwości PWM
            set_step_frequency(cascade_ctrl.state.step_frequency_hz);
            start_step_generation();
        }
    }

    return true;
}

void cascade_controller_stop(void)
{
    cascade_ctrl.state.controller_enabled = false;
    cascade_ctrl.state.is_moving = false;
    cascade_ctrl.state.step_frequency_hz = 0;

    // Zatrzymanie generowania kroków
    stop_step_generation();

    // Reset regulatorów
    pd_controller_reset(&cascade_ctrl.position_controller);
    pid_controller_reset(&cascade_ctrl.velocity_controller);
}

bool cascade_is_position_reached(void)
{
    return cascade_ctrl.state.position_reached;
}

/* ==========================================================
 *  FUNKCJE SPRZĘTOWE
 * ========================================================== */

void set_step_frequency(float frequency_hz)
{
    if (frequency_hz <= 0) {
        stop_step_generation();
        return;
    }

    // Ograniczenie częstotliwości
    if (frequency_hz > CTRL_MAX_STEP_FREQ) {
        frequency_hz = CTRL_MAX_STEP_FREQ;
    }
    if (frequency_hz < CTRL_MIN_STEP_FREQ) {
        frequency_hz = CTRL_MIN_STEP_FREQ;
    }

    // Użycie istniejącej funkcji set_speed z motor_functions
    // Konwersja częstotliwości [Hz] na wartość speed dla set_speed
    uint16_t speed = (uint16_t)frequency_hz;
    set_speed(speed);
}

void set_motor_direction(int8_t direction)
{
    if (direction >= 0) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    }
    cascade_ctrl.state.direction = direction;
}

void start_step_generation(void)
{
    // Włączenie silnika (ENABLE active low dla większości sterowników)
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

    // Start PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void stop_step_generation(void)
{
    // Stop PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Wyłączenie silnika (opcjonalne - można zostawić włączony dla utrzymania pozycji)
    // HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
}

/* ==========================================================
 *  FUNKCJE KONWERSJI
 * ========================================================== */

int32_t degrees_to_steps(float degrees)
{
    return (int32_t)(degrees * CTRL_STEPS_PER_DEG);
}

float steps_to_degrees(int32_t steps)
{
    return (float)steps / CTRL_STEPS_PER_DEG;
}

float velocity_to_step_frequency(float velocity_deg_s)
{
    // steps/s = (deg/s) * (steps/deg)
    return fabsf(velocity_deg_s) * CTRL_STEPS_PER_DEG;
}

float step_frequency_to_velocity(float frequency_hz)
{
    // deg/s = (steps/s) / (steps/deg)
    return frequency_hz / CTRL_STEPS_PER_DEG;
}

/* ==========================================================
 *  FUNKCJE DIAGNOSTYCZNE
 * ========================================================== */

ControllerState* cascade_get_state(void)
{
    return &cascade_ctrl.state;
}

void cascade_print_debug_info(void)
{
    char debug_buffer[200];

    sprintf(debug_buffer,
        "[CASCADE] Pos: %.2f/%.2f deg | Vel: %.2f/%.2f deg/s | Freq: %.1f Hz | Moving: %d\r\n",
        cascade_ctrl.state.actual_position_deg,
        cascade_ctrl.state.target_position_deg,
        cascade_ctrl.state.actual_velocity_deg_s,
        cascade_ctrl.state.velocity_setpoint_deg_s,
        cascade_ctrl.state.step_frequency_hz,
        cascade_ctrl.state.is_moving
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
}

/* ==========================================================
 *  FUNKCJA ZASTĘPCZA DLA move_via_angle (kompatybilność)
 * ========================================================== */

/**
 * @brief Wrapper dla kompatybilności z istniejącym kodem
 *
 * Zastępuje poprzednią funkcję move_via_angle() opartą na S-curve
 * nową implementacją z regulatorem kaskadowym PD+PID.
 *
 * @param angle Docelowy kąt w stopniach
 * @return 1 jeśli sukces, 0 w przypadku błędu
 */
uint8_t move_via_angle_cascade(float angle)
{
    // Inicjalizacja regulatora (jeśli nie była wcześniej wykonana)
    static bool initialized = false;
    if (!initialized) {
        cascade_controller_init();
        initialized = true;
    }

    // Rozpoczęcie ruchu
    if (!cascade_move_to_position(angle)) {
        return 0;
    }

    // Pętla główna sterowania - blokująca
    // (alternatywnie można wywołać cascade_controller_update() z pętli głównej main)
    while (cascade_ctrl.state.is_moving) {
        cascade_controller_update();

        // Opcjonalnie: debug info co 100ms
        static uint32_t last_debug = 0;
        if (HAL_GetTick() - last_debug > 100) {
            cascade_print_debug_info();
            last_debug = HAL_GetTick();
        }

        // Krótkie opóźnienie dla stabilności
        HAL_Delay(1);
    }

    // Sygnalizacja zakończenia zadania
    HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_SET);

    return 1;
}
