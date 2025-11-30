#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include "stm32f1xx_hal.h"  // Đổi nếu dùng chip khác như stm32f4xx_hal.h

// ==== CẤU HÌNH ====
#define FLASH_PAGE_LENGTH  	0x0801FC00  // Địa chỉ page cuối (1KB) của STM32F103C8
#define FLASH_PAGE_DIA		0x0801F800
#define FLASH_PAGE_PPR		0x0801F400
#define FLASH_PAGE_TIME		0x0801F000
#define FLASH_PAGE_BAUD		0x0801EC00
#define FLASH_PAGE_PARITY	0x0801E800
// ==== GHI 32-BIT DỮ LIỆU VÀO FLASH ====
static inline void myFlash_Write(uint32_t address, uint32_t data)
{
    HAL_FLASH_Unlock();

    // Xóa page chứa address
    FLASH_EraseInitTypeDef erase;
    uint32_t PageError;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages     = 1;

    HAL_FLASHEx_Erase(&erase, &PageError);

    // Ghi dữ liệu 32-bit
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);

    HAL_FLASH_Lock();
}

// ==== ĐỌC 32-BIT DỮ LIỆU TỪ FLASH ====
static inline uint32_t myFlash_Read(uint32_t address)
{
    return *(volatile uint32_t*)address;
}

// ==== XÓA DỮ LIỆU FLASH ==============
static inline void myFlash_ErasePage(uint32_t pageAddress)
{
    HAL_FLASH_Unlock();  // Mở khóa flash để ghi/xóa

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = pageAddress;
    eraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        // Nếu xóa lỗi, xử lý tại đây
        Error_Handler();
    }

    HAL_FLASH_Lock();  // Khóa lại flash sau khi xóa
}

#endif
