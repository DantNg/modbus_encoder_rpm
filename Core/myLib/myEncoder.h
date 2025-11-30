#ifndef ENCODER_INTERRUPT_H
#define ENCODER_INTERRUPT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#define M_PI 3.14159265f

// External functions for precise timing (implemented in main.c)
extern uint32_t GetMicros(void);
extern uint32_t GetMillis(void);
extern uint64_t GetMicros64(void);

typedef struct {
    TIM_HandleTypeDef* htim;
    int16_t ppr;                 // Pulse per revolution (đã nhân 4 nếu dùng encoder x4)
    float wheel_diameter_m;     // Đường kính bánh xe (m)
    uint16_t update_ms;         // Chu kỳ lấy mẫu tính RPM (ms)
    int64_t total_pulse;        // Tổng số xung encoder (gồm cả phần tràn)
    uint64_t last_time_us;      // Lần đo trước đó (microseconds)
    int64_t last_total_pulse;   // Tổng xung lần đo trước
} Encoder_t;

static inline void Encoder_Init(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint16_t update_ms) {
    enc->htim = htim;
    enc->ppr = ppr * 4;  // dùng chế độ encoder x4
    enc->wheel_diameter_m = diameter_m;
    enc->update_ms = update_ms;
    enc->total_pulse = 0;
    enc->last_total_pulse = 0;
    enc->last_time_us = GetMicros64();  // Use microsecond-based timing

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
    uint64_t now_us = GetMicros64();  // Use precise microsecond timing
    uint64_t dt_us = now_us - enc->last_time_us;
    
    // Convert to milliseconds for comparison
    uint32_t dt_ms = (uint32_t)(dt_us / 1000ULL);
//    if (dt_ms < enc->update_ms) return 0.0f;

    int64_t pulse_now = enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int64_t delta = pulse_now - enc->last_total_pulse;

    enc->last_total_pulse = pulse_now;
    enc->last_time_us = now_us;

    float rev = (float)delta / (float)enc->ppr;
    float minutes = (float)dt_us / 60000000.0f;  // Convert microseconds to minutes
    return rev / minutes;
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
