/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
