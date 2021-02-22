/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * VENDOR_EDIT
 * File       : goodix_drivers_gt9886.h
 * Description: header file for Goodix GT9886 driver
 * Version   : 1.0
 * Date        : 2019-08-27
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 ****************************************************************/

#ifndef TOUCHPANEL_GOODIX_BRL_H
#define TOUCHPANEL_GOODIX_BRL_H

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include "../goodix_common.h"
#include "../gtx8_tools.h"

#define TPD_DEVICE "Goodix-brl"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
        do {\
                if (LEVEL_DEBUG == tp_debug) {\
                        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
                }\
        }while(0)

#define TPD_DETAIL(a, arg...)\
        do {\
                if (LEVEL_BASIC != tp_debug) {\
                        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
                }\
        }while(0)

#define TPD_DEBUG_NTAG(a, arg...)\
        do {\
                if (tp_debug) {\
                        printk(a, ##arg);\
                }\
        }while(0)

/****************************Start of define declare****************************/

#define set_reg_bit(reg, pos, val)        ((reg) = ((reg) & (~(1 << (pos)))) | (!!(val) << (pos)))

#define MAX_POINT_NUM               10       //max touch point number this ic support
#define MAX_GT_IRQ_DATA_LENGTH      95       //irq data(points,key,checksum) size read from irq
#define MAX_GT_EDGE_DATA_LENGTH     50       //irq edge data read from irq

#define MAX_GESTURE_POINT_NUM       128      //max point number of black gesture

#define GTP_DRIVER_SEND_CFG      	1        // send config to TP while initializing (for no config built in TP's flash)

//Request type define
#define GTP_RQST_RESPONDED          0x00
#define GTP_RQST_CONFIG             0x01
#define GTP_RQST_FRE                0x02
#define GTP_RQST_RESET              0x03
#define GTP_RQST_CLOCK              0x04
#define GTP_RQST_IDLE               0xFF

//triger event
#define GOODIX_TOUCH_EVENT          0x80
#define GOODIX_REQUEST_EVENT        0x40
#define GOODIX_GESTURE_EVENT        0x20
// TODO need confirm those event value
#define GOODIX_FINGER_PRINT_EVENT   0x08
#define GOODIX_FINGER_STATUS_EVENT  0x02
#define GOODIX_FINGER_IDLE_EVENT    0x04
//config define
#define IRQ_EVENT_HEAD_LEN		    8
#define BYTES_PER_POINT			    8
#define COOR_DATA_CHECKSUM_SIZE		2

#define POINT_NUM_OFFSET            2
#define IRQ_EVENT_TYPE_OFFSET       0
#define REQUEST_EVENT_TYPE_OFFSET   2

#define BYTES_PER_EDGE                      2
#define TS_MAX_SENSORID                     9
#define TS_CFG_MAX_LEN                      4096
#define TS_CFG_HEAD_LEN                     4
#define TS_CFG_BAG_NUM_INDEX                2
#define TS_CFG_BAG_START_INDEX              4
#define GOODIX_CFG_MAX_SIZE                 4096

//command define
#define GTP_CMD_NORMAL                  0x00
#define GTP_CMD_RAWDATA                 0x01
#define GTP_CMD_SLEEP                   0x84
#define GTP_CMD_CHARGER_ON              0x10
#define GTP_CMD_CHARGER_OFF             0x11
#define GTP_CMD_GESTURE_ON              0x12
#define GTP_CMD_GESTURE_OFF             0x13
#define GTP_CMD_CLEAR_CFG				0X55


#define GTP_CMD_SHORT_TEST              0x85
#define GTM_CMD_EDGE_LIMIT_LANDSCAPE    0x17
#define GTM_CMD_EDGE_LIMIT_VERTICAL     0x18
//#define GTP_CMD_ENTER_DOZE_TIME         0x1A
#define GTP_CMD_DEFULT_DOZE_TIME        0
#define GTP_CMD_FACE_DETECT_ON          0
#define GTP_CMD_FACE_DETECT_OFF         0
#define GTP_CMD_FOD_FINGER_PRINT        0x14
#define GTP_CMD_GESTURE_ENTER_IDLE      0

#define GTP_CMD_FINGER_PRINT_AREA       0x40 //change the finger print detect area
#define GTP_CMD_FILTER                  0x41 //changer filter
#define GTP_CMD_DEBUG                   0x42
#define GTP_CMD_DOWN_DELTA              0x43
#define GTP_CMD_GESTURE_DEBUG           0x44
#define GTP_CMD_ENTER_GAME_MODE         0x1A
#define GTP_CMD_EXIT_GAME_MODE			0x1B
#define GTP_CMD_GESTURE_MASK            0x46


/******************************************************************/
//gesture type fw send
#define DTAP_DETECT                     0xCC
#define STAP_DETECT                     0x4C
#define UP_VEE_DETECT                   0x76
#define DOWN_VEE_DETECT                 0x5e
#define LEFT_VEE_DETECT                 0x3e
#define RIGHT_VEE_DETECT                0x63 //this gesture is C
#define RIGHT_VEE_DETECT2               0x3c //this gesture is <
#define CIRCLE_DETECT                   0x6f
#define DOUSWIP_DETECT                  0x48
#define RIGHT_SLIDE_DETECT              0xAA
#define LEFT_SLIDE_DETECT               0xbb
#define DOWN_SLIDE_DETECT               0xAB
#define UP_SLIDE_DETECT                 0xBA
#define M_DETECT                        0x6D
#define W_DETECT                        0x77
#define FP_DOWN_DETECT                  0x46
#define FP_UP_DETECT                    0x55
//gesture define
#define GSX_KEY_DATA_LEN                42
#define GSX_GESTURE_TYPE_LEN            32

/****************************Start of auto test ********************/
#define GOODIX_RETRY_NUM_3               3
#define GOODIX_CONFIG_REFRESH_DATA       0x01

#define FLOAT_AMPLIFIER                1000
#define MAX_U16_VALUE                  65535
#define RAWDATA_TEST_TIMES             10

#define MAX_TEST_ITEMS                 10 /* 0P-1P-2P-3P-5P total test items */

#define GTP_INTPIN_TEST                0
#define GTP_CAP_TEST                   1
#define GTP_DELTA_TEST                 2
#define GTP_NOISE_TEST                 3
#define GTP_SHORT_TEST                 5
#define GTP_SELFCAP_TEST               6
#define GTP_SELFNOISE_TEST             7

static char *test_item_name[MAX_TEST_ITEMS] = {
    "GTP_INTPIN_TEST",
    "GTP_CAP_TEST",
    "GTP_DELTA_TEST",
    "GTP_NOISE_TEST",
    "no test",
    "GTP_SHORT_TEST",
    "GTP_SELFCAP_TEST",
    "GTP_SELFNOISE_TEST",
    "no test",
    "no test",
};

#define GTP_TEST_PASS                1
#define GTP_PANEL_REASON             2
#define SYS_SOFTWARE_REASON          3

/* error code */
#define NO_ERR                     0
#define RESULT_ERR                -1
#define RAWDATA_SIZE_LIMIT        -2

/*param key word in .csv */
#define CSV_TP_SPECIAL_RAW_MIN        "specail_raw_min"
#define CSV_TP_SPECIAL_RAW_MAX        "specail_raw_max"
#define CSV_TP_SPECIAL_RAW_DELTA      "special_raw_delta"
#define CSV_TP_SHORT_THRESHOLD        "shortciurt_threshold"
#define CSV_TP_SPECIAL_SELFRAW_MAX    "special_selfraw_max"
#define CSV_TP_SPECIAL_SELFRAW_MIN    "special_selfraw_min"
#define CSV_TP_SELFNOISE_LIMIT        "noise_selfdata_limit"
#define CSV_TP_TEST_CONFIG            "test_config"
#define CSV_TP_NOISE_CONFIG           "noise_config"
#define CSV_TP_NOISE_LIMIT            "noise_data_limit"

#define MAX_DRV_NUM                 21
#define MAX_SEN_NUM                 42
static u8 gt9897_drv_map[] = {42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62};
static u8 gt9897_sen_map[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41};

/****************************Start of Firmware update info********************/

/*mtk max i2c size is 4k*/
#define ISP_MAX_BUFFERSIZE          (1024 * 4)

#define I2C_DATA_MAX_BUFFERSIZE     (1024 * 3)

/* cfg parse from bin */
#define CFG_BIN_SIZE_MIN        279
#define BIN_CFG_START_LOCAL     6
#define MODULE_NUM              22
#define CFG_NUM                 23
#define CFG_INFO_BLOCK_BYTES    8
#define CFG_HEAD_BYTES          32

#define GOODIX_EXIST              1
#define GOODIX_NOT_EXIST          0

/* GTX8 cfg name */
#define GOODIX_NORMAL_CONFIG              "normal_config"             //config_type: 0x01
#define GOODIX_TEST_CONFIG                "tptest_config"             //config_type: 0x00, test config
#define GOODIX_NORMAL_NOISE_CONFIG        "normal_noise_config"       //config_type: 0x02,normal sensitivity, use for charging
#define GOODIX_GLOVE_CONFIG               "glove_config"              //config_type: 0x03,high sensitivity
#define GOODIX_GLOVE_NOISE_CONFIG         "glove_noise_config"        //config_type: 0x04,high sensitivity, use for charging
#define GOODIX_HOLSTER_CONFIG             "holster_config"            //config_type: 0x05,holster
#define GOODIX_HOLSTER_NOISE_CONFIG       "holster_noise_config"      //config_type: 0x06,holster ,use for charging
#define GOODIX_NOISE_TEST_CONFIG          "tpnoise_test_config"       //config_type: 0x07,noise test config

#define getU32(a) ((u32)getUint((u8 *)(a), 4))
#define getU16(a) ((u16)getUint((u8 *)(a), 2))
/****************************End of Firmware update info********************/

/****************************Start of config data****************************/
#define GTP_CONFIG_MIN_LENGTH    186
#define GTP_CONFIG_MAX_LENGTH    10000


struct goodix_register {
  //    uint16_t GTP_REG_FW_CHK_MAINSYS;        /*mainsys reg used to check fw status*/
  // uint16_t GTP_REG_FW_CHK_SUBSYS;         /*subsys reg used to check fw status*/
 //   uint16_t GTP_REG_CONFIG_DATA;           /*configure firmware*/
 // uint16_t GTP_REG_READ_COOR;             /*touch state and info*/
 //   uint16_t GTP_REG_PRODUCT_VER;           /*product id & version*/
  //  uint16_t GTP_REG_WAKEUP_GESTURE;        /*gesture type*/
  //  uint16_t GTP_REG_GESTURE_COOR;          /*gesture point data*/
  //  uint16_t GTP_REG_CMD;                   /*recevice cmd from host*/
  //  uint16_t GTP_REG_RQST;                  /*request from ic*/
  //  uint16_t GTP_REG_NOISE_DETECT;          /*noise state*/
  //  uint16_t GTP_REG_ESD_WRITE;             /*esd state write*/
  //  uint16_t GTP_REG_ESD_READ;              /*esd state read*/
  //  uint16_t GTP_REG_DEBUG;                 /*debug log*/
  //  uint16_t GTP_REG_DOWN_DIFFDATA;         /*down diff data log*/
  //  uint16_t GTP_REG_EDGE_INFO;             /*edge points' info:ewx/ewy/xer/yer*/

  //  uint16_t GTP_REG_RAWDATA;
  //  uint16_t GTP_REG_DIFFDATA;
  //  uint16_t GTP_REG_BASEDATA;
};

struct goodix_fp_coor {
    u16 fp_x_coor;
    u16 fp_y_coor;
    u16 fp_area;
};

#define MAX_SCAN_FREQ_NUM            5
#define MAX_SCAN_RATE_NUM            5
#define MAX_FREQ_NUM_STYLUS          8
#define MAX_STYLUS_SCAN_FREQ_NUM     6
#pragma pack(1)
struct goodix_fw_version {
	u8 rom_pid[6];               /* rom PID */
	u8 rom_vid[3];               /* Mask VID */
	u8 rom_vid_reserved;
	u8 patch_pid[8];              /* Patch PID */
	u8 patch_vid[4];              /* Patch VID */
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	u16 checksum;
};

struct goodix_ic_info_version {
	u8 info_customer_id;
	u8 info_version_id;
	u8 ic_die_id;
	u8 ic_version_id;
	u32 config_id;
	u8 config_version;
	u8 frame_data_customer_id;
	u8 frame_data_version_id;
	u8 touch_data_customer_id;
	u8 touch_data_version_id;
	u8 reserved[3];
};

struct goodix_ic_info_feature { /* feature info*/
	u16 freqhop_feature;
	u16 calibration_feature;
	u16 gesture_feature;
	u16 side_touch_feature;
	u16 stylus_feature;
};

struct goodix_ic_info_param { /* param */
	u8 drv_num;
	u8 sen_num;
	u8 button_num;
	u8 force_num;
	u8 active_scan_rate_num;
	u16 active_scan_rate[MAX_SCAN_RATE_NUM];
	u8 mutual_freq_num;
	u16 mutual_freq[MAX_SCAN_FREQ_NUM];
	u8 self_tx_freq_num;
	u16 self_tx_freq[MAX_SCAN_FREQ_NUM];
	u8 self_rx_freq_num;
	u16 self_rx_freq[MAX_SCAN_FREQ_NUM];
	u8 stylus_freq_num;
	u16 stylus_freq[MAX_FREQ_NUM_STYLUS];
};

struct goodix_ic_info_misc { /* other data */
	u32 cmd_addr;
	u16 cmd_max_len;
	u32 cmd_reply_addr;
	u16 cmd_reply_len;
	u32 fw_state_addr;
	u16 fw_state_len;
	u32 fw_buffer_addr;
	u16 fw_buffer_max_len;
	u32 frame_data_addr;
	u16 frame_data_head_len;
	u16 fw_attr_len;
	u16 fw_log_len;
	u8 pack_max_num;
	u8 pack_compress_version;
	u16 stylus_struct_len;
	u16 mutual_struct_len;
	u16 self_struct_len;
	u16 noise_struct_len;
	u32 touch_data_addr;
	u16 touch_data_head_len;
	u16 point_struct_len;
	u16 reserved1;
	u16 reserved2;
	u32 mutual_rawdata_addr;
	u32 mutual_diffdata_addr;
	u32 mutual_refdata_addr;
	u32 self_rawdata_addr;
	u32 self_diffdata_addr;
	u32 self_refdata_addr;
	u32 iq_rawdata_addr;
	u32 iq_refdata_addr;
	u32 im_rawdata_addr;
	u16 im_readata_len;
	u32 noise_rawdata_addr;
	u16 noise_rawdata_len;
	u32 stylus_rawdata_addr;
	u16 stylus_rawdata_len;
	u32 noise_data_addr;
	u32 esd_addr;
};

struct goodix_ic_info {
	u16 length;
	struct goodix_ic_info_version version;
	struct goodix_ic_info_feature feature;
	struct goodix_ic_info_param parm;
	struct goodix_ic_info_misc misc;
};
#pragma pack()

#define MAX_CMD_DATA_LEN 10
#define MAX_CMD_BUF_LEN  16
#pragma pack(1)
struct goodix_ts_cmd {
	union {
		struct {
			u8 state;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 data[MAX_CMD_DATA_LEN];
		};
		u8 buf[MAX_CMD_BUF_LEN];
	};
};
#pragma pack()

struct config_info {
    u8      goodix_int_type;
    u32     goodix_abs_x_max;
    u32     goodix_abs_y_max;
};
#define MAX_STR_LEN                 32

enum CHECKSUM_MODE {
	CHECKSUM_MODE_U8_LE,
	CHECKSUM_MODE_U16_LE,
};

/*
 * struct goodix_ts_config - chip config data
 * @initialized: whether intialized
 * @name: name of this config
 * @lock: mutex
 * @reg_base: register base of config data
 * @length: bytes of the config
 * @delay: delay time after sending config
 * @data: config data buffer
 */
struct goodix_ts_config {
    unsigned int length;
    char name[MAX_STR_LEN + 1];
    unsigned char data[GOODIX_CFG_MAX_SIZE];
};

struct chip_data_brl {
    bool                                halt_status;                    //1: need ic reset
    u8                                  *touch_data;
    u8                                  *edge_data;
    tp_dev                              tp_type;
    u16                                 *spuri_fp_touch_raw_data;
    struct i2c_client                   *client;
    struct goodix_fw_version            ver_info;
    struct goodix_ic_info               ic_info;
    struct config_info                  config_info;
    struct goodix_register              reg_info;
    struct fw_update_info               update_info;
    struct hw_resource                  *hw_res;
    struct goodix_proc_operations       *goodix_ops;                    //goodix func provide for debug
    struct goodix_fp_coor               fp_coor_report;
    struct goodix_health_info           health_info;

    struct goodix_ts_config             normal_cfg;
    struct goodix_ts_config             normal_noise_cfg;
    struct goodix_ts_config             test_cfg;
    struct goodix_ts_config             noise_test_cfg;

    bool                                esd_check_enabled;
    bool                                fp_down_flag;
    bool                                single_tap_flag;
    uint8_t                             touch_direction;
    char                                *p_tp_fw;
    bool                                kernel_grip_support;
    //add for healthinfo
    unsigned int esd_err_count;
    unsigned int send_cmd_err_count;
};

/**
 * struct ts_test_params - test parameters
 * drv_num: touch panel tx(driver) number
 * sen_num: touch panel tx(sensor) number
 * max_limits: max limits of rawdata
 * min_limits: min limits of rawdata
 * deviation_limits: channel deviation limits
 * short_threshold: short resistance threshold
 * r_drv_drv_threshold: resistance threshold between drv and drv
 * r_drv_sen_threshold: resistance threshold between drv and sen
 * r_sen_sen_threshold: resistance threshold between sen and sen
 * r_drv_gnd_threshold: resistance threshold between drv and gnd
 * r_sen_gnd_threshold: resistance threshold between sen and gnd
 * avdd_value: avdd voltage value
 */
struct ts_test_params {
    u32 rawdata_addr;
    u32 noisedata_addr;
    u32 self_rawdata_addr;
    u32 self_noisedata_addr;

    u32 basedata_addr;
    u32 max_drv_num;
    u32 max_sen_num;
    u32 drv_num;
    u32 sen_num;
    u8 *drv_map;
    u8 *sen_map;

    u32 *max_limits;
    u32 *min_limits;

    u32 *deviation_limits;
    u32 *self_max_limits;
    u32 *self_min_limits;

    u32 noise_threshold;
    u32 self_noise_threshold;
    u32 short_threshold;
    u32 r_drv_drv_threshold;
    u32 r_drv_sen_threshold;
    u32 r_sen_sen_threshold;
    u32 r_drv_gnd_threshold;
    u32 r_sen_gnd_threshold;
    u32 avdd_value;
};

/**
 * struct ts_test_rawdata - rawdata structure
 * data: rawdata buffer
 * size: rawdata size
 */
struct ts_test_rawdata {
    int16_t *data;
    u32 size;
};

struct ts_test_self_rawdata {
    u16 *data;
    u32 size;
};

/**
 * struct goodix_ts_test - main data structrue
 * ts: goodix touch screen data
 * test_config: test mode config data
 * orig_config: original config data
 * noise_config: noise config data
 * test_param: test parameters from limit img
 * rawdata: raw data structure from ic data
 * noisedata: noise data structure from ic data
 * self_rawdata: self raw data structure from ic data
 * self_noisedata: self noise data structure from ic data
 * test_result: test result string
 */
struct goodix_ts_test {
    void *ts;
    struct goodix_ts_config test_config;
    struct goodix_ts_config orig_config;
    struct goodix_ts_config noise_config;
    struct ts_test_params test_params;
    bool   is_item_support[MAX_TEST_ITEMS];
    struct ts_test_rawdata rawdata;
    struct ts_test_rawdata noisedata;
    struct ts_test_self_rawdata self_rawdata;
    struct ts_test_self_rawdata self_noisedata;

    struct goodix_testdata *p_testdata;
    struct seq_file *p_seq_file;
    /*[0][0][0][0][0]..  0 without test; 1 pass, 2 panel failed; 3 software failed */
    char test_result[MAX_TEST_ITEMS];
    int error_count;
    uint64_t      device_tp_fw;
};

struct short_record {
    u32 master;
    u32 slave;
    u16 short_code;
    u8 group1;
    u8 group2;
};

/****************************End of struct declare***************************/

extern int gt8x_rawdiff_mode;

static inline u8 checksum_u8(u8 *data, u32 size)
{
    u8 checksum = 0;
    u32 i = 0;
    for(i = 0; i < size; i++) {
        checksum += data[i];
    }
    return checksum;
}

#endif /*TOUCHPANEL_GOODIX_BRL_H*/
