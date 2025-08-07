#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// Define the GPIO pin connected to the servo's signal wire
#define SERVO_GPIO 21

// Parameters for MCPWM, typical for servos
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microseconds
#define SERVO_MAX_DEGREE 180         // Maximum angle of the servo

/**
 * @brief Function to calculate the pulse width in microseconds for a given angle
 * @param degree_of_rotation: The desired angle (0-180)
 * @return pulse width in microseconds
 */
static uint32_t servo_angle_to_pulsewidth(uint32_t degree_of_rotation)
{
    // Constrain the angle to the valid range
    if (degree_of_rotation > SERVO_MAX_DEGREE) {
        degree_of_rotation = SERVO_MAX_DEGREE;
    }
    // Calculate the pulse width
    return (SERVO_MIN_PULSEWIDTH_US + (((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * degree_of_rotation) / SERVO_MAX_DEGREE));
}

void setup() {}

void loop()
{
    printf("Initializing MCPWM Servo Control...\n");

    // 1. MCPWM Configuration
    // Each MCPWM unit has 2 timers (0, 1, 2) and 3 operators (A, B) per timer.
    // We will use MCPWM unit 0, timer 0, and generator A.
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO);

    mcpwm_config_t pwm_config = {
        .frequency = 50, // Standard servo frequency is 50Hz (20ms period)
        .cmpr_a = 0,     // Duty cycle for A
        .cmpr_b = 0,     // Duty cycle for B
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    printf("MCPWM Initialized.\n");

    // These are the three pulse widths we will test
    uint32_t low_pulse_us = 700;
    uint32_t mid_pulse_us = 1500;
    uint32_t high_pulse_us = 2300;

    while (1) {
        // --- Test State 1: Low Pulse ---
        printf("Sending LOW pulse: %ldus\n", low_pulse_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, low_pulse_us);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds

        // --- Test State 2: Mid Pulse ---
        printf("Sending MID pulse: %ldus\n", mid_pulse_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, mid_pulse_us);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds

        // --- Test State 3: High Pulse ---
        printf("Sending HIGH pulse: %ldus\n", high_pulse_us);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, high_pulse_us);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds
    }
}