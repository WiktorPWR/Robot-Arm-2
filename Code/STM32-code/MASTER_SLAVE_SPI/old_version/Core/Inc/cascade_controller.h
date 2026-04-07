/*
 * cascade_controller.h
 *
 *  Created on: Nov 24, 2025
 *      Author: ostro
 *
 *  Kaskadowa struktura regulatorów PD (pozycja) + PID (prędkość)
 *  dla sterowania silnikiem krokowym manipulatora.
 *
 *  Zgodnie z teorią sterowania układami mechanicznymi:
 *  - Zewnętrzna pętla pozycji: regulator PD
 *  - Wewnętrzna pętla prędkości: regulator PID
 */

#ifndef INC_CASCADE_CONTROLLER_H_
#define INC_CASCADE_CONTROLLER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ==========================================================
 *  STAŁE KONFIGURACYJNE
 * ========================================================== */

// Parametry sprzętowe silnika
#define CTRL_STEPS_PER_REVOLUTION   200
#define CTRL_MICROSTEPPING          16
#define CTRL_GEAR_RATIO             17
#define CTRL_STEPS_PER_DEG          ((CTRL_STEPS_PER_REVOLUTION * CTRL_MICROSTEPPING * CTRL_GEAR_RATIO) / 360.0f)

// Limity systemowe
#define CTRL_MAX_VELOCITY_DEG_S     100.0f    // Maksymalna prędkość [deg/s]
#define CTRL_MAX_ACCELERATION       200.0f    // Maksymalne przyspieszenie [deg/s²]
#define CTRL_MIN_STEP_FREQ          10        // Minimalna częstotliwość kroków [Hz]
#define CTRL_MAX_STEP_FREQ          5000      // Maksymalna częstotliwość kroków [Hz]

// Okres próbkowania regulatorów
#define CTRL_POSITION_LOOP_PERIOD_MS    10    // Okres pętli pozycji [ms]
#define CTRL_VELOCITY_LOOP_PERIOD_MS    1     // Okres pętli prędkości [ms]

// Tolerancja pozycjonowania
#define CTRL_POSITION_TOLERANCE_DEG     0.1f  // Tolerancja błędu pozycji [deg]
#define CTRL_VELOCITY_TOLERANCE_DEG_S   0.5f  // Tolerancja błędu prędkości [deg/s]

/* ==========================================================
 *  STRUKTURY DANYCH
 * ========================================================== */

/**
 * @brief Parametry regulatora PD dla pętli pozycji
 *
 * Równanie regulatora PD:
 * u_pos(t) = Kp * e_pos(t) + Kd * (de_pos/dt)
 *
 * gdzie:
 * - e_pos = theta_ref - theta_actual (błąd położenia)
 * - u_pos = zadana prędkość dla wewnętrznej pętli
 */
typedef struct {
    float Kp;           // Wzmocnienie proporcjonalne
    float Kd;           // Wzmocnienie różniczkujące
    float prev_error;   // Poprzedni błąd (do obliczenia pochodnej)
    float output_limit; // Limit wyjścia (max prędkość zadana) [deg/s]
} PD_Controller;

/**
 * @brief Parametry regulatora PID dla pętli prędkości
 *
 * Równanie regulatora PID:
 * u_vel(t) = Kp_v * e_vel(t) + Ki_v * ∫e_vel(τ)dτ + Kd_v * (de_vel/dt)
 *
 * gdzie:
 * - e_vel = v_ref - v_actual (błąd prędkości)
 * - u_vel = częstotliwość impulsów kroków
 */
typedef struct {
    float Kp;           // Wzmocnienie proporcjonalne
    float Ki;           // Wzmocnienie całkujące
    float Kd;           // Wzmocnienie różniczkujące
    float integral;     // Akumulator całki
    float prev_error;   // Poprzedni błąd (do obliczenia pochodnej)
    float integral_limit;   // Limit anty-windup dla całki
    float output_limit;     // Limit wyjścia (max częstotliwość kroków) [Hz]
} PID_Controller;

/**
 * @brief Stan systemu sterowania
 */
typedef struct {
    // Wartości zadane
    float target_position_deg;      // Pozycja docelowa [deg]
    float target_velocity_deg_s;    // Prędkość zadana z pętli pozycji [deg/s]

    // Wartości zmierzone (aktualne)
    float actual_position_deg;      // Aktualna pozycja z enkodera [deg]
    float actual_velocity_deg_s;    // Aktualna prędkość z enkodera [deg/s]

    // Wyjścia regulatorów
    float velocity_setpoint_deg_s;  // Wyjście regulatora PD (zadana prędkość)
    float step_frequency_hz;        // Wyjście regulatora PID (częstotliwość kroków)

    // Zliczanie kroków dla pozycji
    volatile int32_t step_counter;  // Licznik wykonanych kroków
    int32_t target_steps;           // Docelowa liczba kroków

    // Flagi stanu
    bool is_moving;                 // Flaga ruchu w toku
    bool position_reached;          // Flaga osiągnięcia pozycji
    bool controller_enabled;        // Flaga włączenia regulatora

    // Kierunek ruchu
    int8_t direction;               // +1 = prawo, -1 = lewo
} ControllerState;

/**
 * @brief Kompletna struktura kaskadowego regulatora
 */
typedef struct {
    PD_Controller position_controller;   // Regulator PD pozycji (zewnętrzna pętla)
    PID_Controller velocity_controller;  // Regulator PID prędkości (wewnętrzna pętla)
    ControllerState state;               // Stan systemu
    uint32_t last_position_update_ms;    // Timestamp ostatniej aktualizacji pozycji
    uint32_t last_velocity_update_ms;    // Timestamp ostatniej aktualizacji prędkości
} CascadeController;

/* ==========================================================
 *  ZMIENNE GLOBALNE
 * ========================================================== */

/**
 * @brief Globalny obiekt kaskadowego regulatora
 */
extern CascadeController cascade_ctrl;

/* ==========================================================
 *  FUNKCJE POMOCNICZE - ODCZYT CZUJNIKÓW
 *  (Do zaimplementowania przez użytkownika)
 * ========================================================== */

/**
 * @brief Odczyt aktualnej pozycji z enkodera
 *
 * Dla osi 1-3: enkoder inkrementalny z procedurą homingu
 * Dla osi 4-6: enkoder absolutny AS5600
 *
 * @return Aktualna pozycja kątowa [deg]
 */
float actual_position_read(void);

/**
 * @brief Odczyt aktualnej prędkości z enkodera
 *
 * Enkoder inkrementalny 600 impulsów/obrót
 *
 * @return Aktualna prędkość kątowa [deg/s]
 */
float actual_velocity_read(void);

/* ==========================================================
 *  FUNKCJE INICJALIZACJI
 * ========================================================== */

/**
 * @brief Inicjalizacja kaskadowego regulatora z domyślnymi parametrami
 *
 * Ustawia domyślne wartości Kp, Ki, Kd dla obu pętli
 * oraz inicjalizuje stan systemu.
 */
void cascade_controller_init(void);

/**
 * @brief Ustawienie parametrów regulatora PD (pętla pozycji)
 *
 * @param Kp Wzmocnienie proporcjonalne
 * @param Kd Wzmocnienie różniczkujące
 */
void cascade_set_position_gains(float Kp, float Kd);

/**
 * @brief Ustawienie parametrów regulatora PID (pętla prędkości)
 *
 * @param Kp Wzmocnienie proporcjonalne
 * @param Ki Wzmocnienie całkujące
 * @param Kd Wzmocnienie różniczkujące
 */
void cascade_set_velocity_gains(float Kp, float Ki, float Kd);

/* ==========================================================
 *  FUNKCJE REGULACJI
 * ========================================================== */

/**
 * @brief Obliczenie wyjścia regulatora PD (pętla pozycji)
 *
 * Implementuje równanie:
 * u_pos = Kp * e_pos + Kd * (de_pos/dt)
 *
 * @param pd Wskaźnik na strukturę regulatora PD
 * @param setpoint Pozycja zadana [deg]
 * @param measurement Pozycja zmierzona [deg]
 * @param dt Okres próbkowania [s]
 * @return Zadana prędkość (wyjście regulatora) [deg/s]
 */
float pd_controller_update(PD_Controller *pd, float setpoint, float measurement, float dt);

/**
 * @brief Obliczenie wyjścia regulatora PID (pętla prędkości)
 *
 * Implementuje równanie:
 * u_vel = Kp * e_vel + Ki * ∫e_vel dt + Kd * (de_vel/dt)
 *
 * @param pid Wskaźnik na strukturę regulatora PID
 * @param setpoint Prędkość zadana [deg/s]
 * @param measurement Prędkość zmierzona [deg/s]
 * @param dt Okres próbkowania [s]
 * @return Częstotliwość kroków (wyjście regulatora) [Hz]
 */
float pid_controller_update(PID_Controller *pid, float setpoint, float measurement, float dt);

/**
 * @brief Reset stanu regulatora PID (wyzerowanie całki i błędu)
 *
 * @param pid Wskaźnik na strukturę regulatora PID
 */
void pid_controller_reset(PID_Controller *pid);

/**
 * @brief Reset stanu regulatora PD
 *
 * @param pd Wskaźnik na strukturę regulatora PD
 */
void pd_controller_reset(PD_Controller *pd);

/* ==========================================================
 *  GŁÓWNE FUNKCJE STEROWANIA
 * ========================================================== */

/**
 * @brief Ustawienie nowej pozycji docelowej
 *
 * Rozpoczyna ruch do zadanej pozycji z użyciem regulacji kaskadowej.
 *
 * @param target_angle Docelowy kąt [deg]
 * @return true jeśli ruch został rozpoczęty, false w przypadku błędu
 */
bool cascade_move_to_position(float target_angle);

/**
 * @brief Główna funkcja aktualizacji regulatora kaskadowego
 *
 * Powinna być wywoływana cyklicznie w pętli głównej lub z przerwania timera.
 * Realizuje:
 * 1. Odczyt pozycji i prędkości z enkoderów
 * 2. Obliczenie pętli pozycji (PD) - wolniejsza
 * 3. Obliczenie pętli prędkości (PID) - szybsza
 * 4. Ustawienie częstotliwości PWM dla sterownika silnika
 *
 * @return true jeśli ruch jest w toku, false jeśli pozycja osiągnięta
 */
bool cascade_controller_update(void);

/**
 * @brief Zatrzymanie ruchu i wyłączenie regulatora
 */
void cascade_controller_stop(void);

/**
 * @brief Sprawdzenie czy pozycja została osiągnięta
 *
 * @return true jeśli pozycja docelowa została osiągnięta
 */
bool cascade_is_position_reached(void);

/* ==========================================================
 *  FUNKCJE SPRZĘTOWE
 * ========================================================== */

/**
 * @brief Ustawienie częstotliwości PWM dla generowania kroków
 *
 * @param frequency_hz Częstotliwość kroków [Hz]
 */
void set_step_frequency(float frequency_hz);

/**
 * @brief Ustawienie kierunku obrotu silnika
 *
 * @param direction +1 = prawo (CW), -1 = lewo (CCW)
 */
void set_motor_direction(int8_t direction);

/**
 * @brief Włączenie generowania kroków (start PWM)
 */
void start_step_generation(void);

/**
 * @brief Zatrzymanie generowania kroków (stop PWM)
 */
void stop_step_generation(void);

/* ==========================================================
 *  FUNKCJE KONWERSJI
 * ========================================================== */

/**
 * @brief Konwersja stopni na kroki silnika
 *
 * @param degrees Kąt w stopniach
 * @return Liczba kroków
 */
int32_t degrees_to_steps(float degrees);

/**
 * @brief Konwersja kroków na stopnie
 *
 * @param steps Liczba kroków
 * @return Kąt w stopniach
 */
float steps_to_degrees(int32_t steps);

/**
 * @brief Konwersja prędkości [deg/s] na częstotliwość kroków [Hz]
 *
 * @param velocity_deg_s Prędkość w deg/s
 * @return Częstotliwość kroków w Hz
 */
float velocity_to_step_frequency(float velocity_deg_s);

/**
 * @brief Konwersja częstotliwości kroków [Hz] na prędkość [deg/s]
 *
 * @param frequency_hz Częstotliwość kroków w Hz
 * @return Prędkość w deg/s
 */
float step_frequency_to_velocity(float frequency_hz);

/* ==========================================================
 *  FUNKCJE DIAGNOSTYCZNE
 * ========================================================== */

/**
 * @brief Pobranie aktualnego stanu regulatora
 *
 * @return Wskaźnik na strukturę stanu
 */
ControllerState* cascade_get_state(void);

/**
 * @brief Wypisanie stanu regulatora przez UART (debug)
 */
void cascade_print_debug_info(void);

#endif /* INC_CASCADE_CONTROLLER_H_ */
