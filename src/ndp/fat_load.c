/*
 * fat_load.c
 *
 *  Created on: 2022年12月29日
 *      Author: david
 */

#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "fatfs.h"
#include "fat_load.h"
#include "led.h"
#include "minIni.h"
#include "spi_drv.h"
#include "version_string.h"

void printConfg(void);

/* Parse config.ini to save the settings */
char mcu_file_name[32] = { MCU_FILE_NAME };
char dsp_file_name[64] = { DSP_FILE_NAME };
char model_file_name[64] = { MODEL_FILE_NAME };

struct config_ini_items config_items ={  /* default settings */
		.button_switch = {"audio"},
		.led_event_color = {  \
				LED_COLOR_YELLOW,  \
				LED_COLOR_CYAN, \
				LED_COLOR_MAGENTA, \
				LED_COLOR_RED, \
				LED_COLOR_GREEN, \
				LED_EVENT_NONE },

		.recording_period = 10,
		.imu_conversion_enabled = 1,
		.imu_write_to_file = IMU_FUNC_ENABLE,
		.imu_print_to_terminal = IMU_FUNC_DISABLE,
		.low_power_mode = DOWN_DOWN_LP_MODE,
		.ble_mode = BLE_ENABLE,

	    .cert_location = LOAD_CERTS_USE_DA16600_CERTS,
		.target_cloud = CLOUD_NONE,
        .wifi_config = USE_RENESAS_TOOL_FOR_CONFIG,
		.wifi_ap_name = {0},
		.wifi_passwd = {0},
		.wifi_cc = {0},
		.iotc_uid = {0},
		.iotc_cpid = {0},
		.iotc_env = {0},
	    .ble_name = {0},
	    .ntp_time_server = {0},
	    .aws_endpoint = {0},
	    .aws_device_id = {0},
	    .aws_pub_topic = {0},
	    .aws_sub_topic = {0},
};

/* Local global variables */
static FATFS fatfs_obj;
#if 1
static int fatfs_mounted = 0;
static FIL extract_file_fil;
static int extract_file_opened = 0;
#endif
static uint32_t fatfs_total_sectors;
static int boot_mode =  BOOT_MODE_NONE;
static int sdcard_slot_status =  SDCARD_IN_SLOT;
static int print_console_type = CONSOLE_UART;
static uint32_t event_watch_mode = WATCH_TYPE_AUDIO; //default

char mode_description[64] = {0};
int mode;

// AWS certificate configuration items
char aws_rootCA_file_name[64] = { AWS_ROOT_CERT_FILE_NAME };
char aws_device_cert_file_name[64] = { DEVICE_CERT_FILE_NAME };
char aws_device_private_key_file_name[64] = { DEVICE_PUBLIC_KEY_FILENAME };
int cert_location = LOAD_CERTS_FROM_HEADER;

void init_fatfs(void)
{
    MX_FATFS_Init();
}

int binary_loading(char * file_name)
{
    FRESULT res;
    FIL fil;
    //FILINFO fno;
    char path[64];


    sprintf(path, "0:/");
    strcat(path, file_name);

    //printf("Loading %s\r\n",path);

    // mount
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
    }

#if 0
    res = f_stat(path, &fno);
    if(res != FR_OK){
        //printf("f_stat fail %d\r\n",res);
    }else{
        //printf("Size: %lu\r\n", fno.fsize);
        //printf("Timestamp: %u-%02u-%02u, %02u:%02u\r\n",
               (fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
               fno.ftime >> 11, fno.ftime >> 5 & 63);
        //printf("Attributes: %c%c%c%c%c\r\n",
               (fno.fattrib & AM_DIR) ? 'D' : '-',
               (fno.fattrib & AM_RDO) ? 'R' : '-',
               (fno.fattrib & AM_HID) ? 'H' : '-',
               (fno.fattrib & AM_SYS) ? 'S' : '-',
               (fno.fattrib & AM_ARC) ? 'A' : '-');

    }
#endif

    res = f_open(&fil, path, FA_READ);
    if(res != FR_OK){
        //printf("f_open fail %d\r\n",res);
    }

    //printf("%d bytes\r\n", f_size(&fil));


    res =  f_close(&fil);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }


    // unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }

    return res;
}

uint32_t get_synpkg_size(char * file_name)
{
    FRESULT res;
    FIL fil;
    uint32_t size = 0;
    char path[64];

    sprintf(path, "0:/");
    strcat(path, file_name);

    // mount
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
    }

    res = f_open(&fil, path, FA_READ);
    if(res != FR_OK){
        //printf("f_open fail %d\r\n",res);
        return size;
    }

    size = f_size(&fil);

    res =  f_close(&fil);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }

    // unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }

    return size;
}

uint32_t read_synpkg_block(char * file_name, uint32_t offset, uint8_t *buff,  uint32_t split_len)
{
    FRESULT res;
    FIL fil;
    char path[64];
    uint32_t br;

    sprintf(path, "0:/");
    strcat(path, file_name);

    // mount
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return res;
    }

    res = f_open(&fil, path, FA_READ);
    if(res != FR_OK){
        //printf("f_open fail %d\r\n",res);
        return res;
    }

    res = f_lseek(&fil, offset);
    if(res != FR_OK){
        //printf("f_lseek fail %d\r\n",res);
        return res;
    }
    res = f_read(&fil, buff, split_len, &br);
    if(res != FR_OK){
        //printf("f_read fail %d\r\n",res);
        return res;
    }

    res =  f_close(&fil);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }

    // unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }
    return br;
}

int check_sdcard_env(void)
{
    int ret = -1;
    uint32_t size;

    size = get_synpkg_size(mcu_file_name);
    if (size == 0)
        return ret;

    size = get_synpkg_size(dsp_file_name);
    if (size == 0)
        return ret;

    size = get_synpkg_size(model_file_name);
    if (size == 0)
        return ret;

    return 0;
}

uint32_t cat_file(char * src_file, char * dst_file, int flag)
{
    FRESULT res;
    FIL fil_rd, fil_wr;
    char path[64];
	uint8_t buffer[512];
    uint32_t br, bw;

    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return res;
    }

	/* open src file to read */
	memset(path, 0, sizeof(path));
	sprintf(path, "0:/%s", src_file);
	res = f_open(&fil_rd, path, FA_READ);
	if(res != FR_OK){
		//printf("f_open fail %d\r\n",res);
		return res;
	}

	/* open src file to read */
	memset(path, 0, sizeof(path));
	sprintf(path, "0:/%s", dst_file);
	if ( flag == 0) {
		/* create a new file */
		res = f_open(&fil_wr, path, FA_CREATE_ALWAYS | FA_WRITE);
	} else {
		/* append data to file */
		res = f_open(&fil_wr, path, FA_OPEN_APPEND | FA_WRITE);
	}
	if(res != FR_OK){
		//printf("f_open fail %d\r\n",res);
		return res;
	}

	/* Copy source to destination */
    while (1) {
		res = f_read(&fil_rd, buffer, sizeof(buffer), &br); 
		if(res != FR_OK){
			//printf("f_read fail %d\r\n",res);
			return res;
		}

        if (br == 0)
			break; /* error or eof */

        res = f_write(&fil_wr, buffer, br, &bw);
		if(res != FR_OK){
			//printf("f_write fail %d\r\n",res);
			return res;
		}
        if (bw < br)
			break; /* error or disk full */
    }

    res =  f_close(&fil_rd);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }
    res =  f_close(&fil_wr);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }

    res = f_unmount("");
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }
    return bw;
}

uint32_t remove_file(char * file_name)
{
    FRESULT res;
    char path[64];

    sprintf(path, "0:/%s", file_name);

    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return res;
    }

	/* delete file */
	res = f_unlink(path);
	if(res != FR_OK){
		//printf("f_unlink fail %d\r\n",res);
		return res;
	}

  res = f_unmount("");
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }
    return res;
}

uint32_t write_wav_file(char * file_name, uint8_t *buff,  uint32_t len,  int header)
{
    FRESULT res;
    char path[64];
    uint32_t bw;

    sprintf(path, "0:/%s", file_name);

    if (!fatfs_mounted) {
        res = f_mount(&fatfs_obj, "", 1);
        if(res != FR_OK){
            //printf("f_mount fail %d\r\n",res);
            return res;
        }
        fatfs_mounted = 1;
        //printf("mount fs\n");
    }
    
    if ( header == 1 ) {
        /* create a new file */
        res = f_open(&extract_file_fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    } else {
        /* append data to file */
        if (!extract_file_opened) {
            res = f_open(&extract_file_fil, path, FA_OPEN_APPEND | FA_WRITE);
            if (res == FR_OK) extract_file_opened = 1;
        }
    }
    if(res != FR_OK){
        //printf("f_open fail %d\r\n",res);
        return res;
    }

    res = f_write(&extract_file_fil, buff, len, &bw);
    if(res != FR_OK){
        //printf("f_write fail %d\r\n",res);
        return res;
    }

    if ( header == 1 ) {
        res =  f_close(&extract_file_fil);
        if(res != FR_OK){
            //printf("f_close fail %d\r\n",res);
        }
        extract_file_opened = 0;
    } 

    return bw;
}

uint32_t write_sensor_file(char * file_name, uint32_t sample_size, 
        int16_t *acc_samples, int header, float *acc_converted_samples)
{
    FRESULT res;
    char path[64];
    char buff[128];
    uint32_t buff_len;
    uint32_t bw;

    sprintf(path, "0:/%s", file_name);

    if (!fatfs_mounted) {
        res = f_mount(&fatfs_obj, "", 1);
        if(res != FR_OK){
            //printf("f_mount fail %d\r\n",res);
            return res;
        }
        fatfs_mounted = 1;
    }

	if ( header == 1 ) {
		/* create a new file */
		res = f_open(&extract_file_fil, path, FA_CREATE_ALWAYS | FA_WRITE);
	} else {
		/* append data to file */
        if (!extract_file_opened) {
            res = f_open(&extract_file_fil, path, FA_OPEN_APPEND | FA_WRITE);
            if (res == FR_OK) extract_file_opened = 1;
        }
	}
	if(res != FR_OK){
		//printf("f_open fail %d\r\n",res);
		return res;
	}

	if ( header == 1 ) {
		strcpy(buff, "accX,accY,accZ,gyrX,gyrY,gyrZ\n");
		buff_len = strlen(buff);
	}
    else {
        uint32_t buff_offset = 0;

        // If we're converting the IMU ADC values to units, then write the converted
        // values to the file
        if(is_imu_convertion_enabled()){

            for (int i = 0; i < sample_size / 2; i++) {
                buff_offset += snprintf(&buff[buff_offset], 128,
                        "%f,", acc_converted_samples[i]);
            }
        }
        // Write the ADC values to the file
        else{

            for (int i = 0; i < sample_size / 2; i++) {
                buff_offset += snprintf(&buff[buff_offset], 128,
                    "%d,", acc_samples[i]);
            }
        }

        buff_offset --; //Truncate the last comma in each line
		buff_offset += snprintf(&buff[buff_offset], 128,"\r\n");

        buff_len = buff_offset;
    }

    res = f_write(&extract_file_fil, buff, buff_len, &bw);
    if(res != FR_OK){
        //printf("f_write fail %d\r\n",res);
        return res;
    }

    if ( header == 1 ) {
        res =  f_close(&extract_file_fil);
        if(res != FR_OK){
            //printf("f_close fail %d\r\n",res);
        }
        extract_file_opened = 0;
    } 

    return bw;
}

void write_extraction_file_end(void)
{
    FRESULT res;

    if (extract_file_opened) {
        res =  f_close(&extract_file_fil);
        if(res != FR_OK){
            //printf("f_close fail %d\r\n",res);
        }
        extract_file_opened = 0;
    }

    if (fatfs_mounted) {
        res = f_mount(NULL, "", 0);
        if(res != FR_OK){
            //printf("f_mount umount fail %d\r\n",res);
        }
        fatfs_mounted = 0;
    }
}

static void parse_event_watch_mode( void )
{
    switch (mode)
    {
        case 3:
        case 5:
            event_watch_mode = WATCH_TYPE_MOTION;
            break;

        case 8:
            event_watch_mode = WATCH_TYPE_AUDIO | WATCH_TYPE_MOTION;
            break;

        case 1:
        case 2:
        case 4:
        case 6:
        case 7:
        default:
            event_watch_mode = WATCH_TYPE_AUDIO;
            break;
    }
}

static uint32_t read_config_file( void )
{
    FRESULT res;
    FILINFO fno;
    char inifile[] = "0:/config.ini";
    char color[16] = {0};
    char key[6] = {0};
    char section[24] = {0};

    // mount
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return res;
    }

    /* Get total sectors */
    fatfs_total_sectors = (fatfs_obj.n_fatent - 2) * fatfs_obj.csize;

    /* checks the existence of a file */
    res = f_stat (inifile, &fno);
    if(res == FR_NO_FILE){
        f_unmount("");
        return res;
    }

	/* Read config.ini from sdcard */
	mode = ini_getl("NDP Firmware", "Mode", 0, inifile);
	sprintf(section, "Function_%d", mode);
    parse_event_watch_mode();

    ini_gets(section, "Description", NULL, config_items.mode_description, sizeof(config_items.mode_description), inifile);

	ini_gets(section, "MCU", MCU_FILE_NAME, \
						mcu_file_name, sizeof(mcu_file_name), inifile);
	ini_gets(section, "DSP", DSP_FILE_NAME, \
						dsp_file_name, sizeof(dsp_file_name), inifile);
	ini_gets(section, "DNN", MODEL_FILE_NAME, \
						model_file_name, sizeof(model_file_name), inifile);
	ini_gets(section, "Button_shift", "audio", \
						config_items.button_switch, sizeof(config_items.button_switch), inifile);
	config_items.dec_inshift_value = ini_getl(section, "DECIMATION_INSHIFT_VALUE", DEC_INSHIFT_VALUE_DEFAULT, inifile);
    config_items.dec_inshift_offset = ini_getl(section, "DECIMATION_INSHIFT_OFFSET", DEC_INSHIFT_OFFSET_DEFAULT, inifile);

	/* Get led color according according to voice command */
	for (int idx = 0; idx < LED_EVENT_NUM; idx++)
	{
		sprintf(key, "IDX%d", idx);
		ini_gets("Led", key, "-", color, sizeof(color), inifile);

		if( strncmp(color, "red", 3) == 0)
		{
			config_items.led_event_color[idx] = LED_COLOR_RED;
		} else if( strncmp(color, "green", 5) == 0)
		{
			config_items.led_event_color[idx] = LED_COLOR_GREEN;
		} else if( strncmp(color, "blue", 4) == 0)
		{
			config_items.led_event_color[idx] = LED_COLOR_BLUE;
		} else if( strncmp(color, "cyan", 4) == 0)
		{
			config_items.led_event_color[idx] = LED_COLOR_CYAN;
		} else if( strncmp(color, "magenta", 6) == 0)
		{
		    config_items.led_event_color[idx] = LED_COLOR_MAGENTA;
		} else if( strncmp(color, "yellow", 6) == 0)
		{
			config_items.led_event_color[idx] = LED_COLOR_YELLOW;
		} else
		{
			config_items.led_event_color[idx] = LED_EVENT_NONE;
		}
	}

	print_console_type = ini_getl("Debug Print", "Port", CONSOLE_UART, inifile);
	config_items.recording_period = ini_getl("Recording Period", "Recording_Period", 10, inifile);
	config_items.low_power_mode = ini_getl("Low Power Mode", "Power_Mode",DOWN_DOWN_LP_MODE, inifile);
	config_items.imu_write_to_file = ini_getl("IMU data stream", "Write_to_file", \
										IMU_FUNC_ENABLE, inifile);
	config_items.imu_print_to_terminal = ini_getl("IMU data stream", "Print_to_terminal", \
										IMU_FUNC_DISABLE, inifile);

    config_items.imu_conversion_enabled = ini_getl("IMU Recording Format", "Convert_Data", 1, inifile);

	// BLE Configuration
	config_items.ble_mode = ini_getl("BLE Mode", "BLE_Enabled", BLE_DISABLE, inifile);
    ini_gets("BLE Mode", "BLE_Name", BLE_DEFAULT_NAME, \
            config_items.ble_name, sizeof(config_items.ble_name), inifile);

    // WiFi configuration
    config_items.wifi_config = ini_getl("WIFI", "Use_Config_AP_Details", USE_RENESAS_TOOL_FOR_CONFIG, inifile);

    ini_gets("WIFI", "Access_Point", "WiFi AP Name Undefined", \
                        config_items.wifi_ap_name, sizeof(config_items.wifi_ap_name), inifile);

    ini_gets("WIFI", "Access_Point_Password", "WiFi PasswordUndefined", \
                        config_items.wifi_passwd, sizeof(config_items.wifi_passwd), inifile);

    ini_gets("WIFI", "Country_Code", "US", \
                        config_items.wifi_cc, sizeof(config_items.wifi_cc), inifile);

    ini_gets("WIFI", "NTP_Time_Server", "pool.ntp.org", \
                        config_items.ntp_time_server, sizeof(config_items.ntp_time_server), inifile);

    config_items.target_cloud = ini_getl("Cloud Connectivity", "Target_Cloud", CLOUD_NONE, inifile);

    // Only read the configuration items needed for the current cloud configuration
    switch(config_items.target_cloud){
        case CLOUD_IOTCONNECT:
            // IoTConnect configuration
            ini_gets("IoTConnect", "CPID", "Undefined", \
                                config_items.iotc_cpid, sizeof(config_items.iotc_cpid), inifile);

            ini_gets("IoTConnect", "Device_Unique_ID", "Undefined", \
                                config_items.iotc_uid, sizeof(config_items.iotc_uid), inifile);

            ini_gets("IoTConnect", "Environment", "Undefined", \
                                config_items.iotc_env, sizeof(config_items.iotc_env), inifile);

            break;
        case CLOUD_AWS:
            // AWS configuration items
            ini_gets("AWS", "Endpoint", "Undefined", \
                                config_items.aws_endpoint, sizeof(config_items.aws_endpoint), inifile);

            ini_gets("AWS", "Device_Unique_ID", "Undefined", \
                                config_items.aws_device_id, sizeof(config_items.aws_device_id), inifile);

            ini_gets("AWS", "MQTT_Pub_Topic", "Undefined", \
                                config_items.aws_pub_topic, sizeof(config_items.aws_pub_topic), inifile);

            ini_gets("AWS", "MQTT_Sub_Topic", "Undefined", \
                                config_items.aws_sub_topic, sizeof(config_items.aws_sub_topic), inifile);

            break;
        case CLOUD_AZURE:
            break;
        default:
            break;
    }

    // Certificate config items
    config_items.cert_location = ini_getl("Certs", "Cert_Location", LOAD_CERTS_USE_DA16600_CERTS, inifile);

    ini_gets("Certs", "Root_CA_Filename", "Undefined", \
                        aws_rootCA_file_name, sizeof(aws_rootCA_file_name), inifile);

    ini_gets("Certs", "Device_Cert_Filename", "Undefined", \
            aws_device_cert_file_name, sizeof(aws_device_cert_file_name), inifile);

    ini_gets("Certs", "Device_Private_Key_Filename", "Undefined", \
            aws_device_private_key_file_name, sizeof(aws_device_private_key_file_name), inifile);

    // unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }

    return res;
}

uint32_t get_synpkg_config_info( void )
{
	uint32_t res = 0;
	bool sdcard, flag = false;

	sdcard = (sdmmc_exist_check() == 1) ?  true : false;
	if (sdcard){
		sdcard_slot_status =  SDCARD_IN_SLOT;
	}else{
		sdcard_slot_status =  SDCARD_NOT_IN_SLOT;
	}

	if (SDCARD_IN_SLOT == sdcard_slot_status){
		/* attempt to read config.ini from sdcard */
		res = read_config_file();
		if(res != FR_OK){
			//printf("Cannot find config.txt in sdcard, try to boot from Flash\n");
			flag = true; /* Indicates that the SD card is empty */
		} else {
			boot_mode = BOOT_MODE_SD;
		}
	}

	if ((SDCARD_NOT_IN_SLOT == sdcard_slot_status) || (flag)){
		/* must start NDP120 to initialize its SPI bus first and then read configs from Flash */
		boot_mode = BOOT_MODE_FLASH;
		print_console_type = flag ? CONSOLE_UART : CONSOLE_USB_CDC;
	}

	if (BOOT_MODE_SD == boot_mode){
		// Output the current configuration for the user
		printConfg();

		//printf("NDP120 images identified . . . \n");
		//printf("    MCU : %s\n", mcu_file_name);
		//printf("    DSP : %s\n", dsp_file_name);
		//printf("    DNN : %s\n", model_file_name);
	}

	return res;
}

uint32_t get_sdcard_total_sectors( void )
{
    return fatfs_total_sectors;
}

uint32_t get_sdcard_slot_status( void )
{
	return sdcard_slot_status;
}

uint32_t get_synpkg_boot_mode( void )
{
	return boot_mode;
}

int get_print_console_type( void )
{
    return print_console_type;
}

uint32_t get_event_watch_mode()
{
    return event_watch_mode;
}

void set_event_watch_mode(uint32_t watch_mode)
{
    event_watch_mode = watch_mode;
}

// Returns number of seconds to record data (audio or IMU data)
int get_recording_period( void )
{
    return config_items.recording_period;
}

// Returns the low power mode
int get_low_power_mode( void )
{
    return config_items.low_power_mode;
}

int get_wifi_config( void )
{
    return config_items.wifi_config;
}

int is_imu_data_to_file( void )
{
    return config_items.imu_write_to_file;
}

int is_imu_data_to_terminal( void )
{
    return config_items.imu_print_to_terminal;
}

int is_file_exist_in_sdcard( char *filename )
{
    FRESULT res;
    FILINFO fno;
    int status = 0;

    // mount
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return res;
    }

    /* checks the existence of a file */
    res = f_stat (filename, &fno);
    if(res == FR_NO_FILE){
        status = 1;
    }

    // unmount
    res = f_unmount("");
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }

    return status;
}

void printConfg(void)
{

    // Output application information to user
    //printf("\n\nApplication Version: %s\n", VERSION_STRING);
    //printf("Release Date       : %s\n\n", RELEASE_DATE);
    //printf("Features enabled in config.ini file:\n");

    //printf("\n  Operation mode=%d selected: %s\r\n", mode, config_items.mode_description);
    //printf("    Event Watching Mode: %s %s\r\n",
            //(event_watch_mode&WATCH_TYPE_AUDIO)?"Key-Word":"",
            //(event_watch_mode&WATCH_TYPE_MOTION)?"IMU-Motion":"");

    // Output recording feature driven by Low Power Mode Selection
    if(config_items.low_power_mode == DOWN_DOWN_LP_MODE){

        //printf("  The Recording feature is enabled!\n");
        //printf("    Press user button < 400ms to record %d seconds of %s data\n",
                //config_items.recording_period, config_items.button_switch);
        //printf("    Press user button > 3sec to flash the NDP120 firmware to FLASH\n\n");

        if(0 == strcmp(config_items.button_switch, "imu")){
            if(IMU_FUNC_ENABLE == config_items.imu_print_to_terminal){
                //printf("    IMU data will be streamed to the debug UART\n");
            }
            if(IMU_FUNC_ENABLE == config_items.imu_write_to_file){
                //printf("    IMU data will be captured to the microSD card\n");
            }

            if((IMU_FUNC_ENABLE == config_items.imu_print_to_terminal) || (IMU_FUNC_ENABLE == config_items.imu_write_to_file)){
                //printf("    IMU data will %s converted from ADC values\n", is_imu_convertion_enabled() ? "be": "not be");
            }

            if((IMU_FUNC_DISABLE == config_items.imu_print_to_terminal) && (IMU_FUNC_DISABLE == config_items.imu_write_to_file)){
                //printf("    WARNING: The application is configured to capture IMU data, but the configuration does indicate where to capture the IMU data!\n");
                //printf("             Please edit the config.ini file, section [IMU data stream]\n");
            }
        }
    }
    else {

        //printf("  Note: The recording feature is disabled due to low power mode being set to 1!\n");
        //printf("    To enable the recording feature, edit config.ini on the microSD\n");
        //printf("    card and set \"[Low Power Mode] -> Power_Mode=0\"\n\n");
    }

    // Output BLE mode
    //printf("\n  BLE Mode: %s\n", (config_items.ble_mode) ? "Enabled": "Disabled");
    if(BLE_ENABLE == config_items.ble_mode){
        //printf("  BLE Advertisement Name: %s\n", config_items.ble_name);
    }

    // Output Cloud configuration
    //printf("\n\n  Cloud connectivity: ");
    if(CLOUD_NONE == config_items.target_cloud){
        //printf("Disabled\n");
    }
    else{
        switch(config_items.target_cloud){
            case CLOUD_IOTCONNECT:
                //printf("Avnet's IoTConnect\n");
                //printf("    Device Unique ID: %s\n", config_items.iotc_uid);
                //printf("    Environment     : %s\n", config_items.iotc_env);
                //printf("    CPID            : %.*s********************%.*s\n", \
						6, config_items.iotc_cpid, 6, &(config_items.iotc_cpid[26]));
                break;
            case CLOUD_AWS:
                //printf("AWS IoT Core\n");
                //printf("    Device Unique ID: %s\n", config_items.aws_device_id);
                //printf("    Endpoint        : %s\n", config_items.aws_endpoint);
                //printf("    Pub Topic       : %s\n", config_items.aws_pub_topic);
                //printf("    Sub Topic       : %s\n", config_items.aws_sub_topic);
                break;
            case CLOUD_AZURE:
                //printf("Azure <currently not supported>\n");
                break;
            default:
                break;
        }

        //printf("\n  Using Cloud Certificates ");
        switch(get_load_certificate_from()){
            case LOAD_CERTS_FROM_HEADER:
                //printf("from certs.h header file\n");
                break;

            case LOAD_CERTS_USE_DA16600_CERTS:
                //printf("previously already loaded on DA16600\n");
                break;

            case LOAD_CERTS_FROM_FILES:
                //printf("from files on microSD card\n");
                //printf("      Root CA          : %s\n", get_certificate_file_name(ROOT_CA));
                //printf("      Device Cert      : %s\n", get_certificate_file_name(DEVICE_CERT));
                //printf("      Device Public Key: %s\n", get_certificate_file_name(DEVICE_PUBLIC_KEY));
                break;
        }

        //printf("\n  WiFi Configuration\n");

        if(USE_CONFIG_WIFI_SETTINGS == get_wifi_config()){

            //printf("    Access Point (SSID)  : %s\n", config_items.wifi_ap_name);
            //printf("    Access Point password: %s\n", config_items.wifi_passwd);
            //printf("    Country Code         : %s\n", config_items.wifi_cc);
        }
        else{
            //printf("    Please use the Renesas Wi-Fi Provisioning Tool from your app store to \nconfigure the Wi-Fi network.\n");
        }
        //printf("    NTP Time Server      : %s\n\n", config_items.ntp_time_server);

    }
}

bool get_certificate_data(char* fileName, int certificate_id, char returnCertData[])
{

    FRESULT res;
    FILINFO fno;
    FIL fil;
    char certFileName[64] = { '\0'};
    char lineData[128] = {'\0'};

    // Generate the filename used to operate on the filesystem
    snprintf(certFileName, sizeof(certFileName), "0:/%s", fileName);
    ////printf("Certificate file name: %s\n", certFileName);

    // mount the microSD card
    res = f_mount(&fatfs_obj, "", 1);
    if(res != FR_OK){
        //printf("f_mount fail %d\r\n",res);
        return false;
    }

    // Verify that the file exists on the microSD card
    res = f_stat (certFileName, &fno);
    if(res == FR_NO_FILE){
        f_unmount("");
        //printf("File NOT found!!\n");
        return false;
    }

    // Open the file
    res = f_open(&fil, certFileName, FA_READ);
    if(res != FR_OK){
        //printf("f_open fail %d\r\n",res);
        return false;
    }

    // Define in index into the return array.  We'll move this index
    // as we add data to the return array to keep track of where to write
    // new data.
    int returnCertDataIndex = 0;

    // Generate the string used when we send the certificate to the DA16600.
    // C0, == RootCA cert
    // C1, == Device cert
    // C2, == Device Public Key
    snprintf(returnCertData, 16, "C%d,", certificate_id);
    returnCertDataIndex += strlen(returnCertData);

    // Read the certificate file into the passed in array.
    do{

        // Read the next line in the file
        f_gets (lineData, sizeof(lineData), &fil);

        // Copy the new line of data to the return array
        strcpy(&returnCertData[returnCertDataIndex], lineData);

        // Increment the index to the end of the data
        returnCertDataIndex += strlen(lineData);

        // If we just read the last line of the certificate/public key file, then exit the loop
    } while (0 != strncmp(lineData, "-----END ", 9));

    // Close the file
    res =  f_close(&fil);
    if(res != FR_OK){
        //printf("f_close fail %d\r\n",res);
    }

    // Unmount the microSD card
    res = f_mount(NULL, "", 0);
    if(res != FR_OK){
        //printf("f_mount umount fail %d\r\n",res);
    }

    return true;

}

int get_ble_mode( void )
{
    return config_items.ble_mode;
}

char* get_ble_name( void )
{
    return config_items.ble_name;
}

char* get_wifi_ap( void )
{
    return config_items.wifi_ap_name;
}

char* get_wifi_pw( void )
{
    return config_items.wifi_passwd;
}

char* get_wifi_cc( void ){
    return config_items.wifi_cc;
}

char* get_device_uid( void )
{
    return config_items.iotc_uid;
}

char* get_iotc_env( void )
{
    return config_items.iotc_env;
}

char* get_iotc_cpid( void )
{
    return config_items.iotc_cpid;
}

int get_target_cloud( void )
{
    return config_items.target_cloud;
}

int get_load_certificate_from( void )
{
    return config_items.cert_location;
}

char* get_certificate_file_name(int certID){

    switch (certID) {
        case ROOT_CA:
            return aws_rootCA_file_name;

        case DEVICE_CERT:
            return aws_device_cert_file_name;

        case DEVICE_PUBLIC_KEY:
            return aws_device_private_key_file_name;
        default:
            return NULL;
    }
}

char* get_ntp_time_server( void ){

    return config_items.ntp_time_server;
}

int get_dec_inshift_value( void )
{
    return config_items.dec_inshift_value;
}

int get_dec_inshift_offset( void )
{
    return config_items.dec_inshift_offset;
}

char* get_aws_endpoint( void ){
    return config_items.aws_endpoint;
}

char* get_aws_deviceId( void ){
    return config_items.aws_device_id;
}

char* get_aws_sub_topic( void )
{
    return config_items.aws_sub_topic;
}

char* get_aws_pub_topic( void )
{
    return config_items.aws_pub_topic;
}


char* get_mode_description( void )
{
    return config_items.mode_description;
}

bool is_imu_convertion_enabled( void ){
    return (1 == config_items.imu_conversion_enabled);
}
