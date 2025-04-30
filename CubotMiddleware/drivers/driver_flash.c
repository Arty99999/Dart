#include "driver_flash.h"
#define PARAM_ADDR 0x0800E000 // Bank1 最后一个扇区

SystemParams params = {
    .version = 0x0100,
    .calibration_value = 25.5f,
};
float aoo=2.3453;
float abb;
void Flash_ReadData(uint32_t address, uint8_t *buffer, uint32_t size);
HAL_StatusTypeDef Flash_WriteData(uint32_t address, uint8_t *data, uint32_t size) ;
HAL_StatusTypeDef Flash_EraseSector(uint32_t address) ;
// 保存参数到 Flash
void Save_Params(SystemParams *params) {
	
	  HAL_FLASH_Unlock();
	uint32_t PageError=0;
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;   //擦除方式有bank擦除和sector擦除，这里选择sector擦除
	EraseInit.Sector=FLASH_SECTOR_7;             //选择擦除的扇区为sector7
	EraseInit.Banks=FLASH_BANK_1;               //H750有两个bank，每个bank有7个扇区（sector）
	EraseInit.NbSectors=1;                     //要删除的扇区个数，一个
	EraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;//用于设置编程的并行位数，电压不同，位数不同
	HAL_FLASHEx_Erase(&EraseInit,&PageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,0x080E0000,(uint32_t)&aoo);
  HAL_FLASH_Lock();
	
	
	 abb=*((float*)(0x080E0000));
	
    Flash_EraseSector(PARAM_ADDR);
	__disable_irq();
    Flash_WriteData(PARAM_ADDR, (uint8_t*)params, sizeof(SystemParams));
	__enable_irq();
}

// 从 Flash 加载参数
void Load_Params(SystemParams *params) {
    Flash_ReadData(PARAM_ADDR, (uint8_t*)params, sizeof(SystemParams));
}

// 擦除指定地址所在的扇区（8KB）
HAL_StatusTypeDef Flash_EraseSector(uint32_t address) {
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0;

    // 判断 Bank
    if (address >= 0x08020000) {
        erase.Banks = 0x02;
        erase.Sector = (address - 0x08020000) / 0x2000; // Bank2 扇区号
    } else {
        erase.Banks = FLASH_BANK_1;
        erase.Sector = (address - 0x08000000) / 0x2000; // Bank1 扇区号
    }

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.NbSectors = 1; // 擦除 1 个扇区
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V-3.6V

    HAL_FLASH_Unlock(); // 解锁 Flash
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();   // 重新锁定 Flash

    return status;
}
uint8_t* ak;
uint32_t ag;
// 写入数据（地址必须 4 字节对齐，数据长度需为 4 的倍数）
HAL_StatusTypeDef Flash_WriteData(uint32_t address, uint8_t *data, uint32_t size) {

    if (address % 4 != 0 || size % 4 != 0) {
        return HAL_ERROR; // 地址或长度未对齐
    }
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);  // 清除 Bank1 所有错误标志
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t word = *((uint32_t*)(data + i)); // 转换为 32 位数据
        HAL_StatusTypeDef status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD, // H7 必须用 FLASHWORD
            address + i,
            word
        );
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}
// 直接通过指针读取（无需 HAL 库）
void Flash_ReadData(uint32_t address, uint8_t *buffer, uint32_t size) {
    memcpy(buffer, (void*)address, size);
}