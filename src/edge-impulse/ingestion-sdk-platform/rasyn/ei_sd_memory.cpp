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

#include "ei_sd_memory.h"
#include "peripheral/sdcard.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/**
 * @brief Construct a new EiSDMemory object
 * 
 */
EiSDMemory::EiSDMemory():
    EiDeviceMemory(0, 90, 512, 512)
{
    //
    audio_sample_file_list = 0;
    imu_sample_file_list = 0;

    sd_card_inserted = false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::init(void)
{
    sdcard_init();

    sd_card_inserted = true;
    // TODO
    // check if sd in

    // count file ?

    return sd_card_inserted;
}

/**
 * @brief 
 * 
 * @param data 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    (void)address;

    if (sd_card_inserted == false){
		return 0;
    }
    // address
    uint32_t read_bytes;

    read_bytes = sdcard_read_file(data, num_bytes);

    return read_bytes;
}

/**
 * @brief 
 * 
 * @param data 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    (void)address;

    if (sd_card_inserted == false){
		return 0;
    }
    // address ?
    return sdcard_write_file(data, num_bytes);
}

/**
 * @brief 
 * 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    // useful ?
    (void)address;
    (void)num_bytes;

    return num_bytes;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::close_sample_file(void)
{
    if (sd_card_inserted == false){
		return false;
    }

    return sdcard_close_file();
}

/**
 * @brief 
 * 
 * @param filename 
 * @param write 
 * @return true 
 * @return false 
 */
bool EiSDMemory::open_sample_file(char* filename, bool write)
{
    uint8_t write_mode = FA_READ;
    char path[64] = "0:/";
    int32_t fres;

    strcat(path, filename);

    if (sd_card_inserted == false){
		return false;
    }

    if (write == true) {
        write_mode = FA_WRITE | FA_CREATE_ALWAYS;

        // check if file exist to open as create always or append
    }
    fres = sdcard_open_file(path, write_mode);

    if (fres == 0) {
        latest_file = path;
        file_is_open = true;
    }
    else {
        latest_file.clear();
    }

    return (fres == 0);
}

/**
 *
 * @param write
 * @return
 */
bool EiSDMemory::open_latest_file(bool write)
{
    uint8_t write_mode = FA_READ;
    int32_t fres;

    if (sd_card_inserted == false){
        return false;
    }

    if (write == true) {
        write_mode = FA_WRITE | FA_CREATE_ALWAYS;

        // check if file exist to open as create always or append
    }
    fres = sdcard_open_file(latest_file.c_str(), write_mode);

    if (fres == 0) {
        file_is_open = true;
    }
    else {
        latest_file.clear();
    }

    return (fres == 0);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::read_content(void)
{
    FRESULT res;
    DIR dir;
    FILINFO fno;
    int nfile, ndir;

    res = f_opendir(&dir, local_path.c_str());                       /* Open the directory */
    if (res == FR_OK) {
        nfile = ndir = 0;
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Error or end of dir */
            if (fno.fattrib & AM_DIR) {            /* Directory */
                ei_printf("   <DIR>   %s\n", fno.fname);
                ndir++;
            } else {                               /* File */
                ei_printf("%10u %s\n", fno.fsize, fno.fname);
                nfile++;
            }
        }
        f_closedir(&dir);
        ei_printf("%d dirs, %d files.\n", ndir, nfile);
    } else {
        printf("Failed to open \"%s\". (%u)\n", local_path.c_str(), res);
        return false;
    }

    return true;
}

/**
 * @brief Test SD interface functionality
 * @return
 */
uint32_t EiSDMemory::test(void)
{
    uint8_t write_test[100] = {0};
    uint8_t read_test[256] = {0};
    char file_wr_test[] = "test_file.txt";
    uint32_t ret = 0;
    uint16_t i;

    for (i = 0; i < 100; i++) {
        write_test[i] = (uint8_t)i;
    }

    // open wr file
    ret = open_sample_file(file_wr_test, true);
    ei_printf("open_sample_file %d \r\n", ret);

    // write something
    ret = write_data(write_test, 0, 100);
    ei_printf("write_data %d \r\n", ret);

    //close it
    close_sample_file();

    // open read file
    ret = open_sample_file(file_wr_test, false);
    ei_printf("open_sample_file %d \r\n", ret);

    // read content
    ret = read_data(read_test, 0, 100);
    ei_printf("read_data %d \r\n", ret);

    for (i = 0; i < ret; i++) {
        ei_printf("%d ", read_test[i]);

        if ((i %10) == 0) {
            ei_printf("\r\n");
        }
    }

    //close it
    close_sample_file();

    return ret;
}
