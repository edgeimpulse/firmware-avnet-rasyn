/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "sdcard.h"
#include "ff.h"

/* Local global variables */
static FATFS fatfs_obj;
static FIL fil;
// state Holders
static uint8_t state_mount = 0;
static uint8_t state_unmount = 0;

/**
 * @brief 
 * 
 */
void _mountSdcard(void)
{
	if(state_mount == SDCARD_MOUNTED)
			return;
	else
	{
			if(f_mount(&fatfs_obj , "" , 1) != FR_OK) { // mount immediately (?)

				state_mount = SDCARD_NOT_MOUNTED;
			}
			else {
				state_mount = SDCARD_MOUNTED;
				state_unmount = SDCARD_NOT_UNMOUNTED;
			}
	}
}

/**
 * @brief 
 * 
 */
void _unmountSdcard(void)
{
	if(state_unmount == SDCARD_UNMOUNTED) {
        return;
    }	
	else {
			if(f_mount(NULL , "" , 0) != FR_OK) {
				
				state_unmount = SDCARD_NOT_UNMOUNTED;
			}
			else {
				state_unmount = SDCARD_UNMOUNTED;
				state_mount = SDCARD_NOT_MOUNTED;
			}
	}
}

/**
 * @brief 
 * 
 */
void sdcard_init(void)
{
    // TODO
    // done in fat_load, unify !
	//MX_FATFS_Init();
    _mountSdcard();
}

/**
 * @brief 
 * 
 * @param filename 
 * @param mode 
 * @return int32_t 
 */
int32_t sdcard_open_file(const char* filename, uint8_t mode)
{
    FRESULT res;

    res = f_open(&fil, filename, mode);

    return res;
}

/**
 * @brief 
 * 
 * @return int32_t 
 */
int32_t sdcard_close_file(void)
{
    FRESULT res;

    res =  f_close(&fil);

    return res;
}

/**
 * @brief 
 * 
 * @param buff 
 * @param len 
 * @return uint32_t 
 */
uint32_t sdcard_write_file(const uint8_t *buff,  uint32_t len)
{
    uint32_t res;
    uint32_t written_byte = 0;

    res = f_write(&fil, buff, len, &written_byte);

    return written_byte;
}

/**
 * @brief 
 * 
 * @param buff 
 * @param max_len 
 * @return uint32_t 
 */
uint32_t sdcard_read_file(uint8_t *buff, uint32_t max_len)
{
    uint32_t res;
    uint32_t read_byte;

    res = f_read(&fil, buff, max_len, &read_byte);

    return read_byte;
}

/**
 * @brief 
 * 
 * @param file_name 
 * @return uint32_t 
 */
uint32_t sdcard_delete_file(char * file_name)
{
    FRESULT res;

    res = f_unlink(file_name);

    return (uint32_t)res;
}

/**
 *
 * @return
 */
uint32_t sdcard_return_entires(void)
{
    uint32_t how_many = 0;

    return how_many;
}
