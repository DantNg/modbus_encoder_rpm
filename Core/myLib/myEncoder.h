#ifndef ENCODER_INTERRUPT_H
#define ENCODER_INTERRUPT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#define M_PI 3.14159265f

// External functions for precise timing (implemented in main.c)
extern uint32_t GetMicros(void);
extern uint32_t GetMillis(void);
extern uint64_t GetMicros64(void);

// RPM Moving Average Configuration
#define RPM_BUFFER_SIZE 30

// Encoder configuration structure
typedef struct {
    TIM_HandleTypeDef* htim;
    int16_t ppr;                 // Pulse per revolution (đã nhân 4 nếu dùng encoder x4)
    float wheel_diameter_m;     // Đường kính bánh xe (m)
    uint16_t update_ms;         // Chu kỳ lấy mẫu tính RPM (ms)
    int64_t total_pulse;        // Tổng số xung encoder (gồm cả phần tràn)
    uint32_t last_time_ms;      // Lần đo trước đó (milliseconds)
    int64_t last_total_pulse;   // Tổng xung lần đo trước
    
    // Moving average for RPM
    float rpm_buffer[RPM_BUFFER_SIZE];
    uint8_t rpm_buffer_index;
    uint8_t rpm_buffer_count;
    float rpm_average;
    float current_rpm;
    
    // Timing control
    uint32_t last_update_time_us;
    uint32_t update_interval_us;
} Encoder_t;

static inline void Encoder_Init(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint32_t update_interval_us) {
    enc->htim = htim;
    enc->ppr = ppr * 4;  // dùng chế độ encoder x4
    enc->wheel_diameter_m = diameter_m;
    enc->update_ms = update_interval_us / 1000;
    enc->total_pulse = 0;
    enc->last_total_pulse = 0;
    enc->last_time_ms = GetMicros() / 1000;  // Use precise microsecond timing converted to ms
    
    // Initialize moving average
    for(int i = 0; i < RPM_BUFFER_SIZE; i++) {
        enc->rpm_buffer[i] = 0.0f;
    }
    enc->rpm_buffer_index = 0;
    enc->rpm_buffer_count = 0;
    enc->rpm_average = 0.0f;
    enc->current_rpm = 0.0f;
    
    // Initialize timing control
    enc->last_update_time_us = GetMicros();
    enc->update_interval_us = update_interval_us;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0);

    // Kích hoạt ngắt tràn timer
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    if (htim->Instance == TIM2) {
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    } else if (htim->Instance == TIM3) {
        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

// Gọi hàm này định kỳ để lấy RPM
static inline float Encoder_GetRPM(Encoder_t* enc) {
    uint32_t now_ms = GetMicros() / 1000;  // Use precise microsecond timing
    uint32_t dt_ms = now_ms - enc->last_time_ms;
    
    // Optional: Check if enough time has passed
//    if (dt_ms < enc->update_ms) return 0.0f;

    int64_t pulse_now = enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int64_t delta = pulse_now - enc->last_total_pulse;

    enc->last_total_pulse = pulse_now;
    enc->last_time_ms = now_ms;

    float rev = (float)delta / (float)enc->ppr;
    float minutes = (float)dt_ms / 60000.0f;  // Convert milliseconds to minutes
    return rev / minutes;
}

// Moving average function for RPM smoothing
static inline float Encoder_UpdateRPMAverage(Encoder_t* enc, float new_rpm) {
    // Add new value to circular buffer
    enc->rpm_buffer[enc->rpm_buffer_index] = new_rpm;
    enc->rpm_buffer_index = (enc->rpm_buffer_index + 1) % RPM_BUFFER_SIZE;
    
    // Track how many values we have
    if (enc->rpm_buffer_count < RPM_BUFFER_SIZE) {
        enc->rpm_buffer_count++;
    }
    
    // Calculate average
    float sum = 0.0f;
    for (uint8_t i = 0; i < enc->rpm_buffer_count; i++) {
        sum += enc->rpm_buffer[i];
    }
    
    enc->rpm_average = sum / enc->rpm_buffer_count;
    return enc->rpm_average;
}

// Main update function - call this periodically
static inline void Encoder_Update(Encoder_t* enc) {
    uint32_t now_us = GetMicros();
    
    if ((now_us - enc->last_update_time_us) >= enc->update_interval_us) {
        float raw_rpm = Encoder_GetRPM(enc);
        enc->current_rpm = Encoder_UpdateRPMAverage(enc, raw_rpm);
        
        // Debug output with speed information
        uint32_t dt_us = now_us - enc->last_update_time_us;
        float m_per_min = Encoder_GetMetersPerMinute(enc);
        printf("RPM: %.1f | Speed: %.1f m/min, %.2f m/s, %.2f km/h (dt=%lu.%03lums)\r\n", 
               enc->current_rpm, m_per_min, Encoder_GetMetersPerSecond(enc), 
               Encoder_GetKmPerHour(enc), dt_us/1000, dt_us%1000);
               
        enc->last_update_time_us = now_us;
    }
}

// Get current smoothed RPM value
static inline float Encoder_GetSmoothedRPM(Encoder_t* enc) {
    return enc->current_rpm;
}

// Get RPM as integer for display/transmission
static inline int Encoder_GetRPMInt(Encoder_t* enc) {
    return (int)roundf(enc->current_rpm);
}

// Calculate meters per minute based on wheel diameter and RPM
static inline float Encoder_GetMetersPerMinute(Encoder_t* enc) {
    float circumference_m = enc->wheel_diameter_m * M_PI;  // Circumference in meters
    return enc->current_rpm * circumference_m;  // RPM * circumference = meters/minute
}

// Get m/min as integer for display/transmission  
static inline int Encoder_GetMetersPerMinuteInt(Encoder_t* enc) {
    return (int)roundf(Encoder_GetMetersPerMinute(enc));
}

// Calculate speed in m/s (meters per second)
static inline float Encoder_GetMetersPerSecond(Encoder_t* enc) {
    return Encoder_GetMetersPerMinute(enc) / 60.0f;
}

// Calculate speed in km/h (kilometers per hour)
static inline float Encoder_GetKmPerHour(Encoder_t* enc) {
    float m_per_min = Encoder_GetMetersPerMinute(enc);
    return (m_per_min * 60.0f) / 1000.0f;  // Convert m/min to km/h
}

// Tính quãng đường đã đi (mét)
static inline float Encoder_GetLengthMeter(Encoder_t* enc) {
    int64_t pulse_now = enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
    float rev = (float)pulse_now / (float)enc->ppr;
    float circumference = enc->wheel_diameter_m * M_PI;
    return rev * circumference;
}

// Lấy tổng xung encoder hiện tại
static inline int64_t Encoder_GetPulse(Encoder_t* enc) {
    return enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

#endif // ENCODER_INTERRUPT_H





//#ifndef MY_ENCODER_H
//#define MY_ENCODER_H
//
//#include "stm32f1xx_hal.h"
//#include <stdlib.h>
//#define M_PI 3.14f
//
////typedef struct {
////	int8_t sign;
////	uint64_t part[4];
////} uint256_t;
//
//typedef struct {
//    TIM_HandleTypeDef* htim;
//    int16_t ppr;              // Pulse per revolution
//    float wheel_diameter_m;    // Đường kính bánh xe
//    uint16_t update_ms;        // Chu kỳ cập nhật
//
//    int64_t total_pulse;
//    uint32_t last_count;
//    uint32_t last_time;
//} Encoder_t;
//
//// Khởi tạo
//static inline void Encoder_Init(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint16_t update_ms) {
//    enc->htim = htim;
//    enc->ppr = ppr*4;
//    enc->wheel_diameter_m = diameter_m;
//    enc->update_ms = update_ms;
//
//    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
//    __HAL_TIM_SET_COUNTER(htim, 0);
//
//    enc->last_count = 0;
//    enc->last_time = HAL_GetTick();
//    enc->total_pulse = 0;
//}
//
//// Cập nhật encoder
//static inline void Encoder_Update(Encoder_t* enc) {
//    uint32_t now = HAL_GetTick();
//    if (now - enc->last_time < enc->update_ms) return;
//
//    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
//    int32_t delta = (current - (int32_t)enc->last_count);
//
////    if (abs(delta) > 10000) return;
//    if (delta > 32767) {
//        delta -= 65536;
//    } else if (delta < -32767) {
//        delta += 65536;
//    }
//
//    enc->total_pulse += delta;
//    enc->last_count = current;
//    enc->last_time = now;
////    float rev = (float)enc->total_pulse / enc->ppr;
//}
//
//static inline int64_t Encoder_GetPulse(Encoder_t* enc) {
//	return enc->total_pulse / 4;
//}
//
//// Tính chiều dài
//static inline float Encoder_GetLengthMeter(Encoder_t* enc) {
//	float rev = (float)enc->total_pulse / enc->ppr;
//	float circumference = enc->wheel_diameter_m * (float)M_PI;
//  return rev * circumference;
//}
//
//// Tính RPM
//static inline float Encoder_GetRPM(Encoder_t* enc) {
//    uint32_t now = HAL_GetTick();
//    uint32_t dt = now - enc->last_time;
//    if(dt == 0) return 0.0f;
//    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
//    int32_t delta = (current - (int32_t)enc->last_count);
//
//    if (delta > 32767) {
//        delta -= 65536;
//    } else if (delta < -32767) {
//        delta += 65536;
//    }
//
//    float rev = (float)delta / (float)enc->ppr;  // số vòng quay trong update_ms
//    float minutes = (float)dt / 60000.0f;
//
//    return rev / minutes;
//}
//
//#endif
