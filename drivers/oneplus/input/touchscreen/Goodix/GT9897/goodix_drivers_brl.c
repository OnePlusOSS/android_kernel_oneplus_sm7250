/**************************************************************
 * VENDOR_EDIT
 * File       : goodix_drivers_brl.c
 * Description: Source file for Goodix GT9897 driver
 * Version   : 1.0
 * Date        : 2019-08-27
 * Author    : bsp
 * TAG         : BSP.TP.Init
 ****************************************************************/
#include <linux/module.h>
#include <linux/ctype.h>
#include "goodix_drivers_brl.h"

extern int tp_register_times;
extern struct touchpanel_data *g_tp;

/*
 * goodix_i2c_read_wrapper
 *
 * */
/*static int brl_i2c_read(struct i2c_client *client, u32 addr, unsigned int len, u8 *buffer)
  {
  int ret = -EINVAL;

  ret = touch_i2c_read_block_u32(client, addr, len, buffer);

  return ret;
  }

  static int brl_i2c_write(struct i2c_client *client, u32 addr, unsigned int len, u8 *buffer)
  {
  int ret = -EINVAL;

  ret = touch_i2c_write_block_u32(client, addr, len, buffer);

  return ret;
  }*/

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int goodix_get_usb_state(void)
{
	int ret = 0;
	ret = upmu_get_rgs_chrdet();
	TPD_INFO("%s get usb status %d\n", __func__, ret);
	return ret;
}
#else
static int goodix_get_usb_state(void)
{
	return 0;
}
#endif

/*****************************************************************************
 * goodix_append_checksum
 * @summary
 *    Calcualte data checksum with the specified mode.
 *
 * @param data
 *   data need to be calculate
 * @param len
 *   data length
 * @param mode
 *   calculate for u8 or u16 checksum
 * @return
 *   return the data checksum value.
 *
 *****************************************************************************/
u32 goodix_append_checksum(u8 *data, int len, int mode)
{
	u32 checksum = 0;
	int i;

	checksum = 0;
	if (mode == CHECKSUM_MODE_U8_LE) {
		for (i = 0; i < len; i++)
			checksum += data[i];
	} else {
		for (i = 0; i < len; i+=2)
			checksum += (data[i] + (data[i+1] << 8));
	}

	if (mode == CHECKSUM_MODE_U8_LE) {
		data[len] = checksum & 0xff;
		data[len + 1] = (checksum >> 8) & 0xff;
		return 0xFFFF & checksum;
	}
	data[len] = checksum & 0xff;
	data[len + 1] = (checksum >> 8) & 0xff;
	data[len + 2] = (checksum >> 16) & 0xff;
	data[len + 3] = (checksum >> 24) & 0xff;
	return checksum;
}

/* checksum_cmp: check data valid or not
 * @data: data need to be check
 * @size: data length need to be check(include the checksum bytes)
 * @mode: compare with U8 or U16 mode
 * */
int checksum_cmp(const u8 *data, int size, int mode)
{
	u32 cal_checksum = 0;
	u32 r_checksum = 0;
	u32 i;

	if (mode == CHECKSUM_MODE_U8_LE) {
		if (size < 2)
			return 1;
		for (i = 0; i < size - 2; i++)
			cal_checksum += data[i];

		r_checksum = data[size - 2] + (data[size - 1] << 8);
		return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
	}

	if (size < 4)
		return 1;
	for (i = 0; i < size - 4; i += 2)
		cal_checksum += data[i] + (data[i + 1] << 8);
	r_checksum = data[size - 4] + (data[size - 3] << 8) +
		(data[size - 2] << 16) + (data[size - 1] << 24);
	return cal_checksum == r_checksum ? 0 : 1;
}

/* command ack info */
#define CMD_ACK_IDLE             0x01
#define CMD_ACK_BUSY             0x02
#define CMD_ACK_BUFFER_OVERFLOW  0x03
#define CMD_ACK_CHECKSUM_ERROR   0x04
#define CMD_ACK_OK               0x80

#define GOODIX_CMD_RETRY 		6
static int brl_send_cmd(struct chip_data_brl *chip_info,
		struct goodix_ts_cmd *cmd)
{
	int ret, retry, i;
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (!misc->cmd_addr) {
		TPD_INFO("%s: invalied cmd addr\n", __func__);
		return -EINVAL;
	}
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(&(cmd->buf[2]), cmd->len - 2,
			CHECKSUM_MODE_U8_LE);
	TPD_DEBUG("cmd data %*ph\n", cmd->len, &(cmd->buf[2]));

	retry = 0;
	while (retry++ < GOODIX_CMD_RETRY) {
		ret = touch_i2c_write_block_u32(chip_info->client,
				misc->cmd_addr, sizeof(*cmd), cmd->buf);
		if (ret < 0) {
			TPD_INFO("%s: failed write command\n", __func__);
			return ret;
		}
		for (i = 0; i < GOODIX_CMD_RETRY; i++) {
			/* check command result */
			ret = touch_i2c_read_block_u32(chip_info->client,
					misc->cmd_addr, sizeof(cmd_ack), cmd_ack.buf);
			if (ret < 0) {
				TPD_INFO("%s: failed read command ack, %d\n",
						__func__, ret);
				return ret;
			}
			TPD_DEBUG("cmd ack data %*ph\n",
					(int)sizeof(cmd_ack), cmd_ack.buf);
			if (cmd_ack.ack == CMD_ACK_OK) {
				usleep_range(2000, 2100);
				return 0;
			}

			if (cmd_ack.ack == CMD_ACK_BUSY ||
					cmd_ack.ack == 0x00) {
				usleep_range(1000, 1100);
				continue;
			}
			if (cmd_ack.ack == CMD_ACK_BUFFER_OVERFLOW)
				usleep_range(10000, 11000);
			usleep_range(1000, 1100);
			break;
		}
	}
	TPD_INFO("%s, failed get valid cmd ack\n", __func__);
	return -EINVAL;
}

/* this is for send cmd with one byte cmd_data parameters */
static int goodix_send_cmd_simple(struct chip_data_brl *chip_info,
		u8 cmd_type, u8 cmd_data)
{

	struct goodix_ts_cmd cmd;
	/*
	   if (cmd_type == 0) {
	   TPD_INFO("%s: goodix unsupproted cmd\n", __func__);
	   return 0;
	   }
	 */
	cmd.len = 4;
	cmd.cmd = cmd_type;
	// cmd.data[0] = cmd_data;
	return brl_send_cmd(chip_info, &cmd);
}
static int goodix_send_cmd_simple_onedata(struct chip_data_brl *chip_info,
		u8 cmd_type, u8 cmd_data)
{

	struct goodix_ts_cmd cmd;

	if (cmd_type == 0) {
		TPD_INFO("%s: goodix unsupproted cmd\n", __func__);
		return 0;
	}

	cmd.len = 5;
	cmd.cmd = cmd_type;
	cmd.data[0] = cmd_data;
	return brl_send_cmd(chip_info, &cmd);
}


/********* Start of function that work for touchpanel_operations callbacks***************/
static int goodix_clear_irq(void *chip_data)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 clean_flag = 0;

	if (!gt8x_rawdiff_mode) {
		ret = touch_i2c_write_block_u32(chip_info->client,
				chip_info->ic_info.misc.touch_data_addr, 1, &clean_flag);
		if (ret < 0) {
			TPD_INFO("I2C write end_cmd  error!\n");
		}
	}

	return ret;
}

static void getSpecialCornerPoint(uint8_t *buf, int n, struct Coordinate *point)
{
	int x, y, i;

	point[0].x = (buf[0] & 0xFF) | (buf[1] & 0x0F) << 8;
	point[0].y = (buf[2] & 0xFF) | (buf[3] & 0x0F) << 8;
	point[1] = point[0];
	point[2] = point[0];
	point[3] = point[0];
	for (i = 0; i < n; i++) {
		x = (buf[0 + 4 * i] & 0xFF) | (buf[1 + 4 * i] & 0x0F) << 8;
		y = (buf[2 + 4 * i] & 0xFF) | (buf[3 + 4 * i] & 0x0F) << 8;
		if (point[3].x < x) {   //xmax
			point[3].x = x;
			point[3].y = y;
		}
		if (point[1].x > x) {   //xmin
			point[1].x = x;
			point[1].y = y;
		}
		if (point[2].y < y) {   //ymax
			point[2].y = y;
			point[2].x = x;
		}
		if (point[0].y > y) {   //ymin
			point[0].y = y;
			point[0].x = x;
		}
	}
}

static int clockWise(uint8_t *buf, int n)
{
	int i, j, k;
	int count = 0;
	struct Coordinate p[3];
	long int z;

	if (n < 3)
		return 1;
	for (i = 0; i < n; i++) {
		j = (i + 1) % n;
		k = (i + 2) % n;
		p[0].x = (buf[0 + 4 * i] & 0xFF) | (buf[1 + 4 * i] & 0x0F) << 8;
		p[0].y = (buf[2 + 4 * i] & 0xFF) | (buf[3 + 4 * i] & 0x0F) << 8;
		p[1].x = (buf[0 + 4 * j] & 0xFF) | (buf[1 + 4 * j] & 0x0F) << 8;
		p[1].y = (buf[2 + 4 * j] & 0xFF) | (buf[3 + 4 * j] & 0x0F) << 8;
		p[2].x = (buf[0 + 4 * k] & 0xFF) | (buf[1 + 4 * k] & 0x0F) << 8;
		p[2].y = (buf[2 + 4 * k] & 0xFF) | (buf[3 + 4 * k] & 0x0F) << 8;
		if ((p[0].x == p[1].x) && (p[1].x == p[1].y))
			continue;
		z = (p[1].x - p[0].x) * (p[2].y - p[1].y);
		z -= (p[1].y - p[0].y) * (p[2].x - p[1].x);
		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}

	TPD_INFO("ClockWise count = %d\n", count);

	if (count > 0)
		return 1;
	else
		return 0;
	return 1;
}

static void goodix_esd_check_enable(struct chip_data_brl *chip_info, bool enable)
{
	TPD_INFO("%s %s\n", __func__, enable ? "enable" : "disable");
	/* enable/disable esd check flag */
	chip_info->esd_check_enabled = enable;
}

static int goodix_enter_sleep(struct chip_data_brl *chip_info, bool config)
{
	s32 retry = 0;

	if (config) {
		while (retry++ < 3) {
			if (!goodix_send_cmd_simple(chip_info, GTP_CMD_SLEEP, 0)) {
				chip_info->halt_status = true;
				TPD_INFO("enter sleep mode!\n");
				return 0;
			}
			msleep(10);
		}
	}

	if (retry >= 3) {
		TPD_INFO("Enter sleep mode failed.\n");
	}

	return -1;
}

static int goodix_enable_gesture(struct chip_data_brl *chip_info, bool enable)
{
	int ret = -1;

	TPD_INFO("%s, gesture enable = %d\n", __func__, enable);

	if (enable) {
		if (chip_info->single_tap_flag) {
			ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GESTURE_ON, 1);
		} else {
			ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GESTURE_ON, 0);
		}
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_GESTURE_OFF, 0);
	}

	return ret;
}

static int goodix_enable_edge_limit(struct chip_data_brl *chip_info, int state)
{
	int ret = -1;

	TPD_INFO("%s, edge limit enable = %d\n", __func__, state);
	TPD_INFO("%s, direction is %d\n", __func__, g_tp->limit_switch);

	if (state == 1) {
		if (g_tp->limit_switch == 1)//LANDSCRAPE_SCREEN_90
			ret = goodix_send_cmd_simple_onedata(chip_info, GTM_CMD_EDGE_LIMIT_LANDSCAPE, 0x00);
		if (g_tp->limit_switch == 3)//LANDSCRAPE_SCREEN_270
			ret = goodix_send_cmd_simple_onedata(chip_info, GTM_CMD_EDGE_LIMIT_LANDSCAPE, 0x01);
	} else //VERTICAL_SCREEN
		ret = goodix_send_cmd_simple(chip_info, GTM_CMD_EDGE_LIMIT_VERTICAL, 0x00);

	return ret;
}

static int goodix_enable_charge_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = -1;

	TPD_INFO("%s, charge mode enable = %d\n", __func__, enable);

	if (enable) {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER_ON, 0);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER_OFF, 0);
	}

	return ret;
}

static int goodix_enable_game_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = 0;
	struct goodix_ts_cmd cmd;
	TPD_INFO("%s, game mode enable = %d\n", __func__, enable);
	if(enable) {
		cmd.len = 6;
		cmd.cmd = GTP_CMD_ENTER_GAME_MODE;
		cmd.data[0] = 0x01;			//enter game mode
		cmd.data[1] = 0x14;			//Active to idle time
		ret = brl_send_cmd(chip_info, &cmd);
		//ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GAME_MODE, 0x01);
		TPD_INFO("%s: GTP_CMD_ENTER_GAME_MODE\n", __func__);
	} else {
		cmd.len = 4;
		cmd.cmd = GTP_CMD_EXIT_GAME_MODE;
		ret = brl_send_cmd(chip_info, &cmd);

		// ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GAME_MODE, 0x00);
		TPD_INFO("%s: GTP_CMD_EXIT_GAME_MODE\n", __func__);
	}

	return ret;
}

#pragma  pack(1)
struct goodix_config_head {
	union {
		struct {
			u8 panel_name[8];
			u8 fw_pid[8];
			u8 fw_vid[4];
			u8 project_name[8];
			u8 file_ver[2];
			u32 cfg_id;
			u8 cfg_ver;
			u8 cfg_time[8];
			u8 reserved[15];
			u8 flag;
			u16 cfg_len;
			u8 cfg_num;
			u16 checksum;
		};
		u8 buf[64];
	};
};
#pragma pack()

#define CONFIG_CND_LEN 			4
#define CONFIG_CMD_START 		0x04
#define CONFIG_CMD_WRITE 		0x05
#define CONFIG_CMD_EXIT  		0x06
#define CONFIG_CMD_READ_START  		0x07
#define CONFIG_CMD_READ_EXIT   		0x08

#define CONFIG_CMD_STATUS_PASS 		0x80
#define CONFIG_CMD_WAIT_RETRY           20

static int wait_cmd_status(struct chip_data_brl *chip_info, u8 target_status, int retry)
{
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;
	int i, ret;

	for (i = 0; i < retry; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client,
				misc->cmd_addr, sizeof(cmd_ack),  cmd_ack.buf);
		if (!ret && cmd_ack.state == target_status) {
			TPD_DEBUG("status check pass\n");
			return 0;
		}

		TPD_INFO("%s: cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n",__func__,
				i, cmd_ack.ack, cmd_ack.state, ret);
		TPD_INFO("cmd buf %*ph\n", (int)sizeof(cmd_ack), cmd_ack.buf);
		msleep(15);
	}
	return -EINVAL;
}

static int send_cfg_cmd(struct chip_data_brl *chip_info,
		struct goodix_ts_cmd *cfg_cmd)
{
	int ret;

	ret = brl_send_cmd(chip_info, cfg_cmd);
	if (ret) {
		TPD_INFO("%s: failed write cfg prepare cmd %d\n",__func__, ret);
		return ret;
	}
	ret = wait_cmd_status(chip_info, CONFIG_CMD_STATUS_PASS,
			CONFIG_CMD_WAIT_RETRY);
	if (ret) {
		TPD_INFO("%s: failed wait for fw ready for config, %d\n",
				__func__, ret);
		return ret;
	}
	return 0;
}

static s32 goodix_send_config(struct chip_data_brl *chip_info, u8 *cfg, int len)
{
	int ret;
	u8 *tmp_buf;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (len > misc->fw_buffer_max_len) {
		TPD_INFO("%s: config len exceed limit %d > %d\n", __func__,
				len , misc->fw_buffer_max_len);
		return -EINVAL;
	}
	if (cfg == NULL){
		TPD_INFO ("%s,the cfg value is null\n",__func__);
		return -EINVAL;
	}

	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (!tmp_buf) {
		TPD_INFO("%s: failed alloc malloc\n", __func__);
		return -ENOMEM;
	}

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_START;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("%s: failed write cfg prepare cmd %d\n",
				__func__, ret);
		goto exit;
	}

	TPD_DEBUG("try send config to 0x%x, len %d\n",
			misc->fw_buffer_addr, len);
	ret = touch_i2c_write_block_u32(chip_info->client,
			misc->fw_buffer_addr, len, cfg);
	if (ret) {
		TPD_INFO("%s: failed write config data, %d\n", __func__, ret);
		goto exit;
	}
	ret = touch_i2c_read_block_u32(chip_info->client,
			misc->fw_buffer_addr, len, tmp_buf);
	if (ret) {
		TPD_INFO("%s: failed read back config data\n", __func__);
		goto exit;
	}

	if (memcmp(cfg, tmp_buf, len)) {
		TPD_INFO("%s: config data read back compare file\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	/* notify fw for recive config */
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_WRITE;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret)
		TPD_INFO("%s: failed send config data ready cmd %d\n", __func__, ret);

exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("%s: failed send config write end command\n", __func__);
		ret = -EINVAL;
	}

	if (!ret) {
		TPD_INFO("success send config!,config_len is :%d, config_version is :0x %x \n ",len,cfg[34]);
		msleep(100);
	}

	kfree(tmp_buf);
	return ret;
}

static int goodix_reset(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
		udelay(2000);
		gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
		chip_info->halt_status = false; //reset this flag when ic reset
	} else {
		TPD_INFO("reset gpio is invalid\n");
	}

	msleep(300);
	return 0;
}

/*********** End of function that work for touchpanel_operations callbacks***************/


/********* Start of implementation of touchpanel_operations callbacks********************/
static int goodix_ftm_process(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("%s is called!\n", __func__);
	tp_powercontrol_2v8(chip_info->hw_res, false);
	tp_powercontrol_1v8(chip_info->hw_res, false);
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, false);
	}

	return 0;
}

static void  goodix_util_get_vendor(struct hw_resource * hw_res,
		struct panel_info *panel_data)
{
	char manuf_version[MAX_DEVICE_VERSION_LENGTH] = "0202 ";
	char manuf_manufacture[MAX_DEVICE_MANU_LENGTH] = "GD_";
	char gt_fw_name[16] = "GT9897.bin";
	char gt_test_limit_name[MAX_DEVICE_MANU_LENGTH] = "Brl__limit";

	memcpy (panel_data->manufacture_info.version,
			&manuf_version[0], MAX_DEVICE_VERSION_LENGTH);
	memcpy (panel_data->manufacture_info.manufacture,
			&manuf_manufacture[0], MAX_DEVICE_MANU_LENGTH);
	memcpy (panel_data->fw_name , &gt_fw_name [0],16);
	memcpy (panel_data->test_limit_name,
			&gt_test_limit_name[0], MAX_DEVICE_MANU_LENGTH);
}

static int goodix_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	int len = 0;
	char manu_temp[MAX_DEVICE_MANU_LENGTH] = "HD_";
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	len = strlen(panel_data->fw_name);
	chip_info->tp_type = panel_data->tp_type;
	chip_info->p_tp_fw = panel_data->manufacture_info.version;
	strlcat(manu_temp, panel_data->manufacture_info.manufacture, MAX_DEVICE_MANU_LENGTH);
	strncpy(panel_data->manufacture_info.manufacture, manu_temp, MAX_DEVICE_MANU_LENGTH);
	TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->fw_name);

	return 0;
}


#define GOODIX_READ_VERSION_RETRY 	5
#define FW_VERSION_INFO_ADDR        0x1000C    //0x10068
#define GOODIX_NORMAL_PID		"9897"
/*
 * return: 0 for no error.
 * 	   -EINVAL when encounter a bus error
 * 	   -EFAULT version checksum error
 */
static int brl_read_version(struct chip_data_brl *chip_info,
		struct goodix_fw_version *version)
{
	int ret, i;
	u8 buf[sizeof(struct goodix_fw_version)] = {0};

	for (i = 0; i < GOODIX_READ_VERSION_RETRY; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client,
				FW_VERSION_INFO_ADDR, sizeof(buf), buf);
		if (ret) {
			TPD_INFO("read fw version: %d, retry %d\n", ret, i);
			ret = -EINVAL;
			msleep(5);
			continue;
		}

		if (!checksum_cmp(buf, sizeof(buf), CHECKSUM_MODE_U8_LE))
			break;

		TPD_INFO("invalid fw version: checksum error!\n");
		TPD_INFO("fw version:%*ph\n", (int)sizeof(buf), buf);
		ret = -EFAULT;
		msleep(10);
	}
	if (ret) {
		TPD_INFO("%s: failed get valied fw version\n", __func__);
		return ret;
	}
	memcpy(version, buf, sizeof(*version));
	TPD_INFO("rom_pid:%*ph\n",
			(int)sizeof(version->rom_pid), version->rom_pid);
	TPD_INFO("rom_vid:%*ph\n",
			(int)sizeof(version->rom_vid), version->rom_vid);
	TPD_INFO("pid:%*ph\n",
			(int)sizeof(version->patch_pid), version->patch_pid);
	TPD_INFO("vid:%*ph\n",
			(int)sizeof(version->patch_vid), version->patch_vid);
	TPD_INFO("sensor_id:%d\n", version->sensor_id);
	if (memcmp(GOODIX_NORMAL_PID, version->patch_pid,
				strlen(GOODIX_NORMAL_PID))) {
		TPD_INFO("abnormal patch id detected\n");
		//return GOODIX_EVERSION;
	}
	return 0;
}

#define LE16_TO_CPU(x)  (x = le16_to_cpu(x))
#define LE32_TO_CPU(x)  (x = le32_to_cpu(x))
static int convert_ic_info(struct goodix_ic_info *info, const u8 *data)
{
	int i;
	struct goodix_ic_info_version *version = &info->version;
	struct goodix_ic_info_feature *feature = &info->feature;
	struct goodix_ic_info_param *parm = &info->parm;
	struct goodix_ic_info_misc *misc = &info->misc;

	info->length = le16_to_cpup((__le16 *)data);

	data += 2;
	memcpy(version, data, sizeof(*version));
	version->config_id = le32_to_cpu(version->config_id);

	data += sizeof(struct goodix_ic_info_version);
	memcpy(feature, data, sizeof(*feature));
	feature->freqhop_feature = le16_to_cpu(feature->freqhop_feature);
	feature->calibration_feature = le16_to_cpu(feature->calibration_feature);
	feature->gesture_feature = le16_to_cpu(feature->gesture_feature);
	feature->side_touch_feature = le16_to_cpu(feature->side_touch_feature);
	feature->stylus_feature = le16_to_cpu(feature->stylus_feature);

	data += sizeof(struct goodix_ic_info_feature);
	parm->drv_num = *(data++);
	parm->sen_num = *(data++);
	parm->button_num = *(data++);
	parm->force_num = *(data++);
	parm->active_scan_rate_num = *(data++);
	if (parm->active_scan_rate_num > MAX_SCAN_RATE_NUM) {
		TPD_INFO("%s: invalid scan rate num %d > %d\n", __func__,
				parm->active_scan_rate_num, MAX_SCAN_RATE_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->active_scan_rate_num; i++)
		parm->active_scan_rate[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->active_scan_rate_num * 2;
	parm->mutual_freq_num = *(data++);
	if (parm->mutual_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("%s: invalid mntual freq num %d > %d\n", __func__,
				parm->mutual_freq_num, MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->mutual_freq_num; i++)
		parm->mutual_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->mutual_freq_num * 2;
	parm->self_tx_freq_num = *(data++);
	if (parm->self_tx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("%s: invalid tx freq num %d > %d\n", __func__,
				parm->self_tx_freq_num, MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->self_tx_freq_num; i++)
		parm->self_tx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->self_tx_freq_num * 2;
	parm->self_rx_freq_num = *(data++);
	if (parm->self_rx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("%s: invalid rx freq num %d > %d\n", __func__,
				parm->self_rx_freq_num, MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->self_rx_freq_num; i++)
		parm->self_rx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->self_rx_freq_num * 2;
	parm->stylus_freq_num = *(data++);
	if (parm->stylus_freq_num > MAX_FREQ_NUM_STYLUS) {
		TPD_INFO("%s: invalid stylus freq num %d > %d\n", __func__,
				parm->stylus_freq_num, MAX_FREQ_NUM_STYLUS);
		return -EINVAL;
	}
	for (i = 0; i < parm->stylus_freq_num; i++)
		parm->stylus_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->stylus_freq_num * 2;
	memcpy(misc, data, sizeof(*misc));
	misc->cmd_addr = le32_to_cpu(misc->cmd_addr);
	misc->cmd_max_len = le16_to_cpu(misc->cmd_max_len);
	misc->cmd_reply_addr = le32_to_cpu(misc->cmd_reply_addr);
	misc->cmd_reply_len = le16_to_cpu(misc->cmd_reply_len);
	misc->fw_state_addr = le32_to_cpu(misc->fw_state_addr);
	misc->fw_state_len = le16_to_cpu(misc->fw_state_len);
	misc->fw_buffer_addr = le32_to_cpu(misc->fw_buffer_addr);
	misc->fw_buffer_max_len = le16_to_cpu(misc->fw_buffer_max_len);
	misc->frame_data_addr = le32_to_cpu(misc->frame_data_addr);
	misc->frame_data_head_len = le16_to_cpu(misc->frame_data_head_len);

	misc->fw_attr_len = le16_to_cpu(misc->fw_attr_len);
	misc->fw_log_len = le16_to_cpu(misc->fw_log_len);
	misc->stylus_struct_len = le16_to_cpu(misc->stylus_struct_len);
	misc->mutual_struct_len = le16_to_cpu(misc->mutual_struct_len);
	misc->self_struct_len = le16_to_cpu(misc->self_struct_len);
	misc->noise_struct_len = le16_to_cpu(misc->noise_struct_len);
	misc->touch_data_addr = le32_to_cpu(misc->touch_data_addr);
	misc->touch_data_head_len = le16_to_cpu(misc->touch_data_head_len);
	misc->point_struct_len = le16_to_cpu(misc->point_struct_len);
	LE32_TO_CPU(misc->mutual_rawdata_addr);
	LE32_TO_CPU(misc->mutual_diffdata_addr);
	LE32_TO_CPU(misc->mutual_refdata_addr);
	LE32_TO_CPU(misc->self_rawdata_addr);
	LE32_TO_CPU(misc->self_diffdata_addr);
	LE32_TO_CPU(misc->self_refdata_addr);
	LE32_TO_CPU(misc->iq_rawdata_addr);
	LE32_TO_CPU(misc->iq_refdata_addr);
	LE32_TO_CPU(misc->im_rawdata_addr);
	LE16_TO_CPU(misc->im_readata_len);
	LE32_TO_CPU(misc->noise_rawdata_addr);
	LE16_TO_CPU(misc->noise_rawdata_len);
	LE32_TO_CPU(misc->stylus_rawdata_addr);
	LE16_TO_CPU(misc->stylus_rawdata_len);
	LE32_TO_CPU(misc->noise_data_addr);
	LE32_TO_CPU(misc->esd_addr);

	return 0;
}

static void print_ic_info(struct goodix_ic_info *ic_info)
{
	struct goodix_ic_info_version *version = &ic_info->version;
	struct goodix_ic_info_feature *feature = &ic_info->feature;
	struct goodix_ic_info_param *parm = &ic_info->parm;
	struct goodix_ic_info_misc *misc = &ic_info->misc;

	TPD_INFO("ic_info_length:                %d\n", ic_info->length);
	TPD_INFO("info_customer_id:              0x%01X\n", version->info_customer_id);
	TPD_INFO("info_version_id:               0x%01X\n", version->info_version_id);
	TPD_INFO("ic_die_id:                     0x%01X\n", version->ic_die_id);
	TPD_INFO("ic_version_id:                 0x%01X\n", version->ic_version_id);
	TPD_INFO("config_id:                     0x%4X\n", version->config_id);
	TPD_INFO("config_version:                0x%01X\n", version->config_version);
	TPD_INFO("frame_data_customer_id:        0x%01X\n", version->frame_data_customer_id);
	TPD_INFO("frame_data_version_id:         0x%01X\n", version->frame_data_version_id);
	TPD_INFO("touch_data_customer_id:        0x%01X\n", version->touch_data_customer_id);
	TPD_INFO("touch_data_version_id:         0x%01X\n", version->touch_data_version_id);

	TPD_INFO("freqhop_feature:               0x%04X\n", feature->freqhop_feature);
	TPD_INFO("calibration_feature:           0x%04X\n", feature->calibration_feature);
	TPD_INFO("gesture_feature:               0x%04X\n", feature->gesture_feature);
	TPD_INFO("side_touch_feature:            0x%04X\n", feature->side_touch_feature);
	TPD_INFO("stylus_feature:                0x%04X\n", feature->stylus_feature);

	TPD_INFO("Drv*Sen,Button,Force num:      %d x %d, %d, %d\n",
			parm->drv_num, parm->sen_num, parm->button_num, parm->force_num);

	TPD_INFO("Cmd:                           0x%04X, %d\n",
			misc->cmd_addr, misc->cmd_max_len);
	TPD_INFO("Cmd-Reply:                     0x%04X, %d\n",
			misc->cmd_reply_addr, misc->cmd_reply_len);
	TPD_INFO("FW-State:                      0x%04X, %d\n",
			misc->fw_state_addr, misc->fw_state_len);
	TPD_INFO("FW-Buffer:                     0x%04X, %d\n",
			misc->fw_buffer_addr, misc->fw_buffer_max_len);
	TPD_INFO("Touch-Data:                    0x%04X, %d\n",
			misc->touch_data_addr, misc->touch_data_head_len);
	TPD_INFO("point_struct_len:              %d\n", misc->point_struct_len);
	TPD_INFO("mutual_rawdata_addr:           0x%04X\n",
			misc->mutual_rawdata_addr);
	TPD_INFO("mutual_diffdata_addr:          0x%04X\n",
			misc->mutual_diffdata_addr);
	TPD_INFO("self_rawdata_addr:             0x%04X\n",
			misc->self_rawdata_addr);
	TPD_INFO("self_difdata_addr:             0x%04X\n",
			misc->self_diffdata_addr);
	TPD_INFO("stylus_diffdata_addr:           0x%04X, %d\n",
			misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
	TPD_INFO("esd_addr:                      0x%04X\n", misc->esd_addr);
}

#define GOODIX_GET_IC_INFO_RETRY	3
#define GOODIX_IC_INFO_MAX_LEN		1024
#define GOODIX_IC_INFO_ADDR		    0x10068
static u8 GOODIX_IC_INFO_DEFAULT[200] = { 0x9C,0x00,0x03,0x00,0x20,0x00,0x39,0x71,0x5F,0x35,0x02,0x01,0x00,0x01,0x00,0x6E,0x41,0x11,0x02,0x00,
	0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x10,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x01,0x01,
	0x00,0x10,0x00,0x48,0x02,0x01,0x00,0x10,0x00,0xDC,0x02,0x01,0x00,0x5C,0x00,0xB4,0x5E,0x01,0x00,0x74,
	0x0F,0x00,0x00,0x00,0x00,0x10,0x00,0x16,0x00,0x84,0x00,0x03,0x00,0x00,0x00,0x88,0x04,0x72,0x00,0x0A,
	0x00,0x38,0x03,0x01,0x00,0x08,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x10,0x16,0x01,0x00,0xF4,0x1C,0x01,
	0x00,0x2C,0x0F,0x01,0x00,0xAC,0x0E,0x01,0x00,0x2C,0x0E,0x01,0x00,0xAC,0x0D,0x01,0x00,0x74,0x33,0x01,
	0x00,0x54,0x3B,0x01,0x00,0x3C,0x2B,0x01,0x00,0x7E,0x00,0x28,0x09,0x01,0x00,0x54,0x00,0xE8,0x7D,0x01,
	0x00,0xC6,0x02,0x78,0x08,0x01,0x00,0x68,0x01,0x01,0x00,0x00,0x00,0x00,0x9A,0x11,0xCE,0x92,0x1C,0x0C,
	0x9B,0x2F,0x48,0x08,0xBB,0xAF,0x49,0xF3,0xBF,0x0A,0x2F,0x36,0x58,0x5E,0x22,0x44,0xD2,0xFA,0x3F,0x85,
	0x44,0x9D,0xA0,0x18,0x5A,0x87,0x38,0x80,0xBF,0xA3,0x21,0x2F,0x0F,0xB1,0x61,0x74,0x86,0x9E,0x9D,0x9B};

static int brl_get_ic_info(struct chip_data_brl *chip_info,
		struct goodix_ic_info *ic_info)
{
	int ret, i;
	u16 length = 0;
	u8 afe_data[GOODIX_IC_INFO_MAX_LEN] = {0};
	//u8 flag = 0;

	for (i = 0; i < GOODIX_GET_IC_INFO_RETRY; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client,
				GOODIX_IC_INFO_ADDR,
				sizeof(length), (u8 *)&length);
		if (ret) {
			TPD_INFO("failed get ic info length, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		length = le16_to_cpu(length);
		if (length >= GOODIX_IC_INFO_MAX_LEN) {
			TPD_INFO("invalid ic info length %d, retry %d\n", length, i);
			continue;
		}

		ret = touch_i2c_read_block_u32(chip_info->client,
				GOODIX_IC_INFO_ADDR, length, afe_data);
		if (ret) {
			TPD_INFO("failed get ic info data, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		/* judge whether the data is valid */
		/*if (is_risk_data((const uint8_t *)afe_data, length)) {
		  TPD_INFO("fw info data invalid\n");
		  usleep_range(5000, 5100);
		  continue;
		  }*/
		if (checksum_cmp((const uint8_t *)afe_data,
					length, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("fw info checksum error!\n");
			usleep_range(5000, 5100);
			continue;
		}
		break;
	}
	if (i == GOODIX_GET_IC_INFO_RETRY) {
		TPD_INFO("%s: failed get ic info\n", __func__);
		/* set ic_info length =0 to indicate this is invalid */
		ic_info->length = 0;
		//	return -EINVAL;
		//flag = 1;
	}

	//if(flag == 1) {
		memset(afe_data,0,sizeof(afe_data));
		memcpy(afe_data, GOODIX_IC_INFO_DEFAULT, 190);
		TPD_INFO("get ic info from default\n");
		//flag = 0;
	//} 

	ret = convert_ic_info(ic_info, afe_data);
	if (ret) {
		TPD_INFO("%s: convert ic info encounter error\n", __func__);
		ic_info->length = 0;
		return ret;
	}
	print_ic_info(ic_info);
	/* check some key info */
	if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
			!ic_info->misc.touch_data_addr) {
		TPD_INFO("%s: cmd_addr fw_buf_addr and touch_data_addr is null\n", __func__);
		ic_info->length = 0;
		//return -EINVAL;
		goto Error_icinfo;
	}
	TPD_INFO("success get ic info %d\n", ic_info->length);
	return 0;

Error_icinfo:
	TPD_INFO("failed get ic info \n");
	return 0;
}

static int goodix_get_chip_info(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	return brl_get_ic_info(chip_info, &chip_info->ic_info);
}

static int goodix_power_control(void *chip_data, bool enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (true == enable) {
		ret = tp_powercontrol_1v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		usleep_range(3000, 3100);
		ret = tp_powercontrol_2v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		msleep(10);
		if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
			gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
		}
		msleep(100);
	} else {
		ret = tp_powercontrol_1v8(chip_info->hw_res, false);
		if (ret)
			return -1;
		ret = tp_powercontrol_2v8(chip_info->hw_res, false);
		if (ret)
			return -1;
		if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
			gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
		}
	}

	return ret;
}


static int goodix_read_config(struct chip_data_brl *chip_info, u8 *cfg, int size);

static fw_check_state goodix_fw_check(void *chip_data,
		struct resolution_info *resolution_info,
		struct panel_info *panel_data)
{
	int ret = 0;
	u32 fw_ver_num = 0;
	u8 cfg_ver = 0;
	//char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version *fw_ver;
	//	struct goodix_ts_config temp_cfg; 

	// TODO need confirm fw state check method
	fw_ver = &chip_info->ver_info;
	ret = brl_read_version(chip_data, fw_ver);
	if (ret < 0) {
		TPD_INFO("%s: goodix read version failed\n", __func__);
		return FW_ABNORMAL;
	}
	if (!isdigit(fw_ver->patch_pid[0]) || !isdigit(fw_ver->patch_pid[1]) ||
			!isdigit(fw_ver->patch_pid[2]) || !isdigit(fw_ver->patch_pid[3])) {
		TPD_INFO("%s: goodix abnormal patch id found: %s\n", __func__,
				fw_ver->patch_pid);
		return FW_ABNORMAL;
	}
	fw_ver_num = be32_to_cpup((__be32 *)&fw_ver->patch_vid[0]);

    cfg_ver = chip_info->normal_cfg.data[34];

	if (panel_data->manufacture_info.version) {
		//panel_data->TP_FW = cfg_ver | (fw_ver_num & 0xFF) << 8;
		//sprintf(dev_version, "%04x", panel_data->TP_FW);
		//strlcpy(&(panel_data->manufacture_info.version[7]), dev_version, 5);
		panel_data->TP_FW = cfg_ver | (fw_ver_num & 0xFF) << 8;;
		sprintf(panel_data->manufacture_info.version, "0x%x", panel_data->TP_FW);
	}

	TPD_INFO("%s: panel_data->TP_FW = 0x%x \n", __func__, panel_data->TP_FW);
	return FW_NORMAL;
}

/*****************start of GT9886's update function********************/

#define FW_HEADER_SIZE				256
#define FW_SUBSYS_INFO_SIZE			10
#define FW_SUBSYS_INFO_OFFSET       36
#define FW_SUBSYS_MAX_NUM			28

#define ISP_MAX_BUFFERSIZE			(1024 * 4)

#define NO_NEED_UPDATE              99

#define FW_PID_LEN		    		8
#define FW_VID_LEN       			4
#define FLASH_CMD_LEN 				11

#define FW_FILE_CHECKSUM_OFFSET 	8
#define CONFIG_ID_OFFSET 			30
#define CONFIG_DATA_TYPE 			4

#define ISP_RAM_ADDR				0x18400
#define HW_REG_CPU_RUN_FROM			0x10000
#define FLASH_CMD_REG				0x10400
#define HW_REG_ISP_BUFFER			0x10410
#define CONFIG_DATA_ADDR			0x3E000

#define HOLD_CPU_REG_W 				0x0002
#define HOLD_CPU_REG_R 				0x2000
#define MISCTL_REG				    0xD807
#define ESD_KEY_REG     			0xCC58
#define WATCH_DOG_REG				0xCC54


#define FLASH_CMD_TYPE_READ  			    0xAA
#define FLASH_CMD_TYPE_WRITE 			    0xBB
#define FLASH_CMD_ACK_CHK_PASS	    		0xEE
#define FLASH_CMD_ACK_CHK_ERROR     		0x33
#define FLASH_CMD_ACK_IDLE      		    0x11
#define FLASH_CMD_W_STATUS_CHK_PASS 		0x22
#define FLASH_CMD_W_STATUS_CHK_FAIL 		0x33
#define FLASH_CMD_W_STATUS_ADDR_ERR 		0x44
#define FLASH_CMD_W_STATUS_WRITE_ERR 		0x55
#define FLASH_CMD_W_STATUS_WRITE_OK 		0xEE


/**
 * fw_subsys_info - subsytem firmware infomation
 * @type: sybsystem type
 * @size: firmware size
 * @flash_addr: flash address
 * @data: firmware data
 */
struct fw_subsys_info {
	u8 type;
	u32 size;
	u32 flash_addr;
	const u8 *data;
};

/**
 *  firmware_summary
 * @size: fw total length
 * @checksum: checksum of fw
 * @hw_pid: mask pid string
 * @hw_pid: mask vid code
 * @fw_pid: fw pid string
 * @fw_vid: fw vid code
 * @subsys_num: number of fw subsystem
 * @chip_type: chip type
 * @protocol_ver: firmware packing
 *   protocol version
 * @bus_type: 0 represent I2C, 1 for SPI
 * @subsys: sybsystem info
 */
#pragma pack(1)
struct  firmware_summary {
	u32 size;
	u32 checksum;
	u8 hw_pid[6];
	u8 hw_vid[3];
	u8 fw_pid[FW_PID_LEN];
	u8 fw_vid[FW_VID_LEN];
	u8 subsys_num;
	u8 chip_type;
	u8 protocol_ver;
	u8 bus_type;
	u8 flash_protect;
	u8 reserved[2];
	struct fw_subsys_info subsys[FW_SUBSYS_MAX_NUM];
};
#pragma pack()

/**
 * firmware_data - firmware data structure
 * @fw_summary: firmware infomation
 * @firmware: firmware data structure
 */
struct firmware_data {
	struct  firmware_summary fw_summary;
	const struct firmware *firmware;
};

struct config_data {
	u8 *data;
	int size;
};

#pragma pack(1)
struct goodix_flash_cmd {
	union {
		struct {
			u8 status;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 fw_type;
			u16 fw_len;
			u32 fw_addr;
			//u16 checksum;
		};
		u8 buf[16];
	};
};
#pragma pack()

struct fw_update_ctrl {
	int mode;
	struct goodix_ts_config *ic_config;
	struct chip_data_brl *chip_info;
	struct firmware_data fw_data;
};

/**
 * goodix_parse_firmware - parse firmware header infomation
 *	and subsystem infomation from firmware data buffer
 *
 * @fw_data: firmware struct, contains firmware header info
 *	and firmware data.
 * return: 0 - OK, < 0 - error
 */
/* sizeof(length) + sizeof(checksum) */

static int goodix_parse_firmware(struct firmware_data *fw_data)
{
	const struct firmware *firmware;
	struct  firmware_summary *fw_summary;
	unsigned int i, fw_offset, info_offset;
	u32 checksum;
	int r = 0;

	fw_summary = &fw_data->fw_summary;

	/* copy firmware head info */
	firmware = fw_data->firmware;
	if (firmware->size < FW_SUBSYS_INFO_OFFSET) {
		TPD_INFO("%s: Invalid firmware size:%zu\n",
				__func__, firmware->size);
		r = -EINVAL;
		goto err_size;
	}
	memcpy(fw_summary, firmware->data, sizeof(*fw_summary));

	/* check firmware size */
	fw_summary->size = le32_to_cpu(fw_summary->size);
	if (firmware->size != fw_summary->size + FW_FILE_CHECKSUM_OFFSET) {
		TPD_INFO("%s: Bad firmware, size not match, %zu != %d\n",
				__func__, firmware->size, fw_summary->size + 6);
		r = -EINVAL;
		goto err_size;
	}

	for (i = FW_FILE_CHECKSUM_OFFSET, checksum = 0;
			i < firmware->size; i+=2)
		checksum += firmware->data[i] + (firmware->data[i+1] << 8);

	/* byte order change, and check */
	fw_summary->checksum = le32_to_cpu(fw_summary->checksum);
	if (checksum != fw_summary->checksum) {
		TPD_INFO("%s: Bad firmware, cheksum error\n", __func__);
		r = -EINVAL;
		goto err_size;
	}

	if (fw_summary->subsys_num > FW_SUBSYS_MAX_NUM) {
		TPD_INFO("%s: Bad firmware, invalid subsys num: %d\n",
				__func__, fw_summary->subsys_num);
		r = -EINVAL;
		goto err_size;
	}

	/* parse subsystem info */
	fw_offset = FW_HEADER_SIZE;
	for (i = 0; i < fw_summary->subsys_num; i++) {
		info_offset = FW_SUBSYS_INFO_OFFSET +
			i * FW_SUBSYS_INFO_SIZE;

		fw_summary->subsys[i].type = firmware->data[info_offset];
		fw_summary->subsys[i].size =
			le32_to_cpup((__le32 *)&firmware->data[info_offset + 1]);

		fw_summary->subsys[i].flash_addr =
			le32_to_cpup((__le32 *)&firmware->data[info_offset + 5]);
		if (fw_offset > firmware->size) {
			TPD_INFO("%s: Sybsys offset exceed Firmware size\n",
					__func__);
			goto err_size;
		}

		fw_summary->subsys[i].data = firmware->data + fw_offset;
		fw_offset += fw_summary->subsys[i].size;
	}

	TPD_INFO("Firmware package protocol: V%u\n", fw_summary->protocol_ver);
	TPD_INFO("Fimware PID:GT%s\n", fw_summary->fw_pid);
	TPD_INFO("Fimware VID:%*ph\n", 4, fw_summary->fw_vid);
	TPD_INFO("Firmware chip type:%02X\n", fw_summary->chip_type);
	TPD_INFO("Firmware bus type:%d\n", fw_summary->bus_type);
	TPD_INFO("Firmware size:%u\n", fw_summary->size);
	TPD_INFO("Firmware subsystem num:%u\n", fw_summary->subsys_num);

	for (i = 0; i < fw_summary->subsys_num; i++) {
		TPD_DEBUG("------------------------------------------\n");
		TPD_DEBUG("Index:%d\n", i);
		TPD_DEBUG("Subsystem type:%02X\n", fw_summary->subsys[i].type);
		TPD_DEBUG("Subsystem size:%u\n", fw_summary->subsys[i].size);
		TPD_DEBUG("Subsystem flash_addr:%08X\n",
				fw_summary->subsys[i].flash_addr);
		TPD_DEBUG("Subsystem Ptr:%p\n", fw_summary->subsys[i].data);
	}

err_size:
	return r;
}

static int goodix_reg_write(struct chip_data_brl *chip_info, unsigned int addr,
		unsigned char *data, unsigned int len)
{

	return touch_i2c_write_block_u32(chip_info->client, addr, len, data);
}

static int goodix_reg_read(struct chip_data_brl *chip_info, unsigned int addr,
		unsigned char *data, unsigned int len)
{
	return touch_i2c_read_block_u32(chip_info->client, addr, len, data);
}

/**
 * goodix_reg_write_confirm - write register and confirm the value
 *  in the register.
 * @client: pointer to touch device client
 * @addr: register address
 * @data: pointer to data buffer
 * @len: data length
 * return: 0 write success and confirm ok
 *           < 0 failed
 */
static int goodix_reg_write_confirm(struct i2c_client *client,
		unsigned int addr, unsigned char *data,
		unsigned int len)
{
	u8 *cfm, cfm_buf[32];
	int r, i;

	if (len > sizeof(cfm_buf)) {
		cfm = kzalloc(len, GFP_KERNEL);
		if (!cfm) {
			TPD_INFO("Mem alloc failed\n");
			return -ENOMEM;
		}
	} else {
		cfm = &cfm_buf[0];
	}

	for (i = 0; i < GOODIX_BUS_RETRY_TIMES; i++) {
		r = touch_i2c_write_block_u32(client, addr, len, data);
		if (r < 0)
			goto exit;
		r = touch_i2c_read_block_u32(client, addr, len, cfm);
		if (r < 0)
			goto exit;

		if (memcmp(data, cfm, len)) {
			r = -EMEMCMP;
			continue;
		} else {
			r = 0;
			break;
		}
	}

exit:
	if (cfm != &cfm_buf[0])
		kfree(cfm);
	return r;
}

static int goodix_fw_update_reset(struct fw_update_ctrl *fwu_ctrl, int delay_ms)
{
	struct chip_data_brl *chip_info = fwu_ctrl->chip_info;

	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
		udelay(2000);
		gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
		if (delay_ms < 20)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 100);
		else
			msleep(delay_ms);
	}
	return 0;
}

/**
 * goodix_fw_version_compare - compare the active version with
 * firmware file version.
 * @fwu_ctrl: firmware infomation to be compared
 * return: 0 equal, < 0 unequal
 */
static int goodix_fw_version_compare(struct fw_update_ctrl *fwu_ctrl)
{
	int ret = 0;
	struct goodix_fw_version fw_version = {{0}};
	struct firmware_summary *fw_summary = &fwu_ctrl->fw_data.fw_summary;

	ret = brl_read_version(fwu_ctrl->chip_info, &fw_version);
	if (ret) {
		TPD_INFO("failed get active fw version\n");
		return -EINVAL;
	}

	if (memcmp(fw_version.patch_pid, fw_summary->fw_pid, FW_PID_LEN)) {
		TPD_INFO("%s: Product ID mismatch:%s:%s\n", __func__,
				fw_version.patch_pid, fw_summary->fw_pid);
		return -EINVAL;
	}

	ret = memcmp(fw_version.patch_vid, fw_summary->fw_vid, FW_VID_LEN);
	if (ret) {
		TPD_INFO("active firmware version:%*ph\n", FW_VID_LEN,
				fw_version.patch_vid);
		TPD_INFO("firmware file version: %*ph\n", FW_VID_LEN,
				fw_summary->fw_vid);
		return -EINVAL;
	}

	TPD_INFO("Firmware version equal\n");
	return 0;
	//return -EINVAL;
}

/**
 * goodix_load_isp - load ISP program to deivce ram
 * @dev: pointer to touch device
 * @fw_data: firmware data
 * return 0 ok, <0 error
 */
static int goodix_load_isp(struct fw_update_ctrl *fwu_ctrl)
{
	struct firmware_data *fw_data = &fwu_ctrl->fw_data;
	struct goodix_fw_version isp_fw_version = {{0}};
	struct fw_subsys_info *fw_isp;
	u8 reg_val[8] = {0x00};
	int r;

	fw_isp = &fw_data->fw_summary.subsys[0];

	TPD_INFO("Loading ISP start\n");
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info->client, ISP_RAM_ADDR,
			(u8 *)fw_isp->data, fw_isp->size);
	if (r < 0) {
		TPD_INFO("%s: Loading ISP error\n", __func__);
		return r;
	}

	TPD_INFO("Success send ISP data\n");

	/* SET BOOT OPTION TO 0X55 */
	memset(reg_val, 0x55, 8);
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info->client,
			HW_REG_CPU_RUN_FROM, reg_val, 8);
	if (r < 0) {
		TPD_INFO("%s: Failed set REG_CPU_RUN_FROM flag\n", __func__);
		return r;
	}
	TPD_INFO("Success write [8]0x55 to 0x%x\n", HW_REG_CPU_RUN_FROM);

	if (goodix_fw_update_reset(fwu_ctrl, 100))
		TPD_INFO("%s: reset abnormal\n", __func__);
	/*check isp state */
	if (brl_read_version(fwu_ctrl->chip_info, &isp_fw_version)) {
		TPD_INFO("%s: failed read isp version\n", __func__);
		return -2;
	}
	if (memcmp(&isp_fw_version.patch_pid[3], "ISP", 3)) {
		TPD_INFO("%s: patch id error %c%c%c != %s\n", __func__,
				isp_fw_version.patch_pid[3], isp_fw_version.patch_pid[4],
				isp_fw_version.patch_pid[5], "ISP");
		return -3;
	}
	TPD_INFO("ISP running successfully\n");
	return 0;
}

/**
 * goodix_update_prepare - update prepare, loading ISP program
 *  and make sure the ISP is running.
 * @fwu_ctrl: pointer to fimrware control structure
 * return: 0 ok, <0 error
 */
static int goodix_update_prepare(struct fw_update_ctrl *fwu_ctrl)
{
	u8 reg_val[4] = {0};
	u8 temp_buf[64] = {0};
	int retry = 20;
	int r;

	/*reset IC*/
	TPD_INFO("firmware update, reset\n");
	if (goodix_fw_update_reset(fwu_ctrl, 5))
		TPD_INFO("%s: reset abnormal\n", __func__);

	retry = 100;
	/* Hold cpu*/
	do {
		reg_val[0] = 0x01;
		reg_val[1] = 0x00;
		r = goodix_reg_write(fwu_ctrl->chip_info,
				HOLD_CPU_REG_W, reg_val, 2);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				HOLD_CPU_REG_R, &temp_buf[0], 4);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				HOLD_CPU_REG_R, &temp_buf[4], 4);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				HOLD_CPU_REG_R, &temp_buf[8], 4);
		if (!r && !memcmp(&temp_buf[0], &temp_buf[4], 4) &&
				!memcmp(&temp_buf[4], &temp_buf[8], 4) &&
				!memcmp(&temp_buf[0], &temp_buf[8], 4)) {
			break;
		}
		usleep_range(1000, 1100);
		TPD_INFO("retry hold cpu %d\n", retry);
		TPD_DEBUG("data:%*ph\n", 12, temp_buf);
	} while (--retry);
	if (!retry) {
		TPD_INFO("%s: Failed to hold CPU, return =%d\n", __func__, r);
		return -1;
	}
	TPD_INFO("Success hold CPU\n");

	/* enable misctl clock */
	reg_val[0] = 0x08;
	r = goodix_reg_write(fwu_ctrl->chip_info, MISCTL_REG, reg_val, 1);
	TPD_INFO("enbale misctl clock\n");

	/* open ESD_KEY */
	retry = 20;
	do {
		reg_val[0] = 0x95;
		r = goodix_reg_write(fwu_ctrl->chip_info,
				ESD_KEY_REG, reg_val, 1);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				ESD_KEY_REG, temp_buf, 1);
		if (!r && temp_buf[0] == 0x01)
			break;
		usleep_range(1000, 1100);
		TPD_INFO("retry %d enable esd key, 0x%x\n", retry, temp_buf[0]);
	} while (--retry);
	if (!retry) {
		TPD_INFO("%s: Failed to enable esd key, return =%d\n", __func__, r);
		return -2;
	}
	TPD_INFO("success enable esd key\n");

	/* disable watch dog */
	reg_val[0] = 0x00;
	r = goodix_reg_write(fwu_ctrl->chip_info, WATCH_DOG_REG, reg_val, 1);
	TPD_INFO("disable watch dog\n");

	/* load ISP code and run form isp */
	r = goodix_load_isp(fwu_ctrl);
	if (r < 0)
		TPD_INFO("%s: Failed lode and run isp\n", __func__);

	return r;
}

/* goodix_send_flash_cmd: send command to read or write flash data
 * @flash_cmd: command need to send.
 * */
static int goodix_send_flash_cmd(struct fw_update_ctrl *fwu_ctrl,
		struct goodix_flash_cmd *flash_cmd)
{
	int i, ret, retry;
	struct goodix_flash_cmd tmp_cmd;

	TPD_INFO("try send flash cmd:%*ph\n", (int)sizeof(flash_cmd->buf),
			flash_cmd->buf);
	memset(tmp_cmd.buf, 0, sizeof(tmp_cmd));
	ret = goodix_reg_write(fwu_ctrl->chip_info, FLASH_CMD_REG,
			flash_cmd->buf, sizeof(flash_cmd->buf));
	if (ret) {
		TPD_INFO("%s: failed send flash cmd %d\n", __func__, ret);
		return ret;
	}

	retry = 5;
	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(fwu_ctrl->chip_info, FLASH_CMD_REG,
				tmp_cmd.buf, sizeof(tmp_cmd.buf));
		if (!ret && tmp_cmd.ack == FLASH_CMD_ACK_CHK_PASS)
			break;
		usleep_range(5000, 5100);
		TPD_INFO("flash cmd ack error retry %d, ack 0x%x, ret %d\n",
				i, tmp_cmd.ack, ret);
	}
	if (tmp_cmd.ack != FLASH_CMD_ACK_CHK_PASS) {
		TPD_INFO("%s: flash cmd ack error, ack 0x%x, ret %d\n",
				__func__, tmp_cmd.ack, ret);
		TPD_INFO("%s: data:%*ph\n", __func__,
				(int)sizeof(tmp_cmd.buf), tmp_cmd.buf);
		return -EINVAL;
	}
	TPD_INFO("flash cmd ack check pass\n");

	msleep(80);
	retry = 20;
	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(fwu_ctrl->chip_info, FLASH_CMD_REG,
				tmp_cmd.buf, sizeof(tmp_cmd.buf));
		if (!ret && tmp_cmd.ack == FLASH_CMD_ACK_CHK_PASS &&
				tmp_cmd.status == FLASH_CMD_W_STATUS_WRITE_OK) {
			TPD_INFO("flash status check pass\n");
			return 0;
		}

		TPD_INFO("flash cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n",
				i, tmp_cmd.ack, tmp_cmd.status, ret);
		msleep(15);
	}

	TPD_INFO("%s: flash cmd status error %d, ack 0x%x, status 0x%x, ret %d\n", __func__,
			i, tmp_cmd.ack, tmp_cmd.status, ret);
	if (ret) {
		TPD_INFO("reason: bus or paltform error\n");
		return -EINVAL;
	}

	switch (tmp_cmd.status) {
		case FLASH_CMD_W_STATUS_CHK_PASS:
			TPD_INFO("%s: data check pass, but failed get follow-up results\n", __func__);
			return -EFAULT;
		case FLASH_CMD_W_STATUS_CHK_FAIL:
			TPD_INFO("%s: data check failed, please retry\n", __func__);
			return -EAGAIN;
		case FLASH_CMD_W_STATUS_ADDR_ERR:
			TPD_INFO("%s: flash target addr error, please check\n", __func__);
			return -EFAULT;
		case FLASH_CMD_W_STATUS_WRITE_ERR:
			TPD_INFO("%s: flash data write err, please retry\n", __func__);
			return -EAGAIN;
		default:
			TPD_INFO("%s: unknown status\n", __func__);
			return -EFAULT;
	}
}

static int goodix_flash_package(struct fw_update_ctrl *fwu_ctrl,
		u8 subsys_type, u8 *pkg, u32 flash_addr, u16 pkg_len)
{
	int ret, retry;
	struct goodix_flash_cmd flash_cmd;

	retry = 2;
	do {
		ret = goodix_reg_write_confirm(fwu_ctrl->chip_info->client,
				HW_REG_ISP_BUFFER, pkg, pkg_len);
		if (ret < 0) {
			TPD_INFO("%s: Failed to write firmware packet\n", __func__);
			return ret;
		}

		flash_cmd.status = 0;
		flash_cmd.ack = 0;
		flash_cmd.len = FLASH_CMD_LEN;
		flash_cmd.cmd = FLASH_CMD_TYPE_WRITE;
		flash_cmd.fw_type = subsys_type;
		flash_cmd.fw_len = cpu_to_le16(pkg_len);
		flash_cmd.fw_addr = cpu_to_le32(flash_addr);

		goodix_append_checksum(&(flash_cmd.buf[2]),
				9, CHECKSUM_MODE_U8_LE);

		ret = goodix_send_flash_cmd(fwu_ctrl, &flash_cmd);
		if (!ret) {
			TPD_INFO("success write package to 0x%x, len %d\n",
					flash_addr, pkg_len - 4);
			return 0;
		}
	} while (ret == -EAGAIN && --retry);

	return ret;
}

/**
 * goodix_flash_subsystem - flash subsystem firmware,
 *  Main flow of flashing firmware.
 *	Each firmware subsystem is divided into several
 *	packets, the max size of packet is limited to
 *	@{ISP_MAX_BUFFERSIZE}
 * @dev: pointer to touch device
 * @subsys: subsystem infomation
 * return: 0 ok, < 0 error
 */
static int goodix_flash_subsystem(struct fw_update_ctrl *fwu_ctrl,
		struct fw_subsys_info *subsys)
{
	u16 data_size, offset;
	u32 total_size;
	//TODO: confirm flash addr ,<< 8??
	u32 subsys_base_addr = subsys->flash_addr;
	u8 *fw_packet = NULL;
	int r = 0;

	/*
	 * if bus(i2c/spi) error occued, then exit, we will do
	 * hardware reset and re-prepare ISP and then retry
	 * flashing
	 */
	total_size = subsys->size;
	fw_packet = kzalloc(ISP_MAX_BUFFERSIZE + 4, GFP_KERNEL);
	if (!fw_packet) {
		TPD_INFO("%s: Failed alloc memory\n", __func__);
		return -EINVAL;
	}

	offset = 0;
	while (total_size > 0) {
		data_size = total_size > ISP_MAX_BUFFERSIZE ?
			ISP_MAX_BUFFERSIZE : total_size;
		TPD_INFO("Flash firmware to %08x,size:%u bytes\n",
				subsys_base_addr + offset, data_size);

		memcpy(fw_packet, &subsys->data[offset], data_size);
		/* set checksum for package data */
		goodix_append_checksum(fw_packet,
				data_size, CHECKSUM_MODE_U16_LE);

		r = goodix_flash_package(fwu_ctrl, subsys->type, fw_packet,
				subsys_base_addr + offset, data_size + 4);
		if (r) {
			TPD_INFO("%s: failed flash to %08x,size:%u bytes\n",
					__func__, subsys_base_addr + offset, data_size);
			break;
		}
		offset += data_size;
		total_size -= data_size;
	} /* end while */

	kfree(fw_packet);
	return r;
}

/**
 * goodix_flash_firmware - flash firmware
 * @dev: pointer to touch device
 * @fw_data: firmware data
 * return: 0 ok, < 0 error
 */
static int goodix_flash_firmware(struct fw_update_ctrl *fw_ctrl)
{
	struct firmware_data *fw_data = &fw_ctrl->fw_data;
	struct  firmware_summary  *fw_summary;
	struct fw_subsys_info *fw_x;
	struct fw_subsys_info subsys_cfg = {0};
	int retry = GOODIX_BUS_RETRY_TIMES;
	int i, r = 0, fw_num;

	/* start from subsystem 1,
	 * subsystem 0 is the ISP program */

	fw_summary = &fw_data->fw_summary;
	fw_num = fw_summary->subsys_num;

	/* flash config data first if we have */
	if (fw_ctrl->ic_config && fw_ctrl->ic_config->length) {
		subsys_cfg.data = fw_ctrl->ic_config->data;
		subsys_cfg.size = fw_ctrl->ic_config->length;
		subsys_cfg.flash_addr = CONFIG_DATA_ADDR;
		subsys_cfg.type = CONFIG_DATA_TYPE;
		r = goodix_flash_subsystem(fw_ctrl, &subsys_cfg);
		if (r) {
			TPD_INFO("%s: failed flash config with ISP, %d\n",
					__func__, r);
			return r;
		}
		TPD_INFO("success flash config with ISP\n");
	}

	for (i = 1; i < fw_num && retry;) {
		TPD_INFO("--- Start to flash subsystem[%d] ---\n", i);
		fw_x = &fw_summary->subsys[i];
		r = goodix_flash_subsystem(fw_ctrl, fw_x);
		if (r == 0) {
			TPD_INFO("--- End flash subsystem[%d]: OK ---\n", i);
			i++;
		} else if (r == -EAGAIN) {
			retry--;
			TPD_INFO("%s: --- End flash subsystem%d: Fail, errno:%d, retry:%d ---\n", __func__,
					i, r, GOODIX_BUS_RETRY_TIMES - retry);
		} else if (r < 0) { /* bus error */
			TPD_INFO("%s: --- End flash subsystem%d: Fatal error:%d exit ---\n", __func__,
					i, r);
			goto exit_flash;
		}
	}

exit_flash:
	return r;
}

/**
 * goodix_update_finish - update finished, FREE resource
 *  and reset flags---
 * @fwu_ctrl: pointer to fw_update_ctrl structrue
 * return: 0 ok, < 0 error
 */
static int goodix_update_finish(struct fw_update_ctrl *fwu_ctrl)
{
	if (goodix_fw_update_reset(fwu_ctrl, 100))
		TPD_INFO("%s: reset abnormal\n", __func__);
	if (goodix_fw_version_compare(fwu_ctrl))
		return -EINVAL;

	return 0;
}

static u32 getUint(u8 *buffer, int len)
{
	u32 num = 0;
	int i = 0;
	for (i = 0; i < len; i++) {
		num <<= 8;
		num += buffer[i];
	}
	return num;
}

static int goodix_parse_cfg_data(const struct firmware *cfg_bin,
		char *cfg_type, u8 *cfg, int *cfg_len, u8 sid)
{
	int i = 0, config_status = 0, one_cfg_count = 0;
	int cfgPackageLen = 0;

	u8 bin_group_num = 0, bin_cfg_num = 0;
	u16 cfg_checksum = 0, checksum = 0;
	u8 sid_is_exist = GOODIX_NOT_EXIST;
	u16 cfg_offset = 0;

	TPD_DEBUG("%s run,sensor id:%d\n", __func__, sid);

	cfgPackageLen = getU32(cfg_bin->data) + BIN_CFG_START_LOCAL;
	if ( cfgPackageLen > cfg_bin->size) {
		TPD_INFO("%s:Bad firmware!,cfg package len:%d,firmware size:%d\n",
				__func__, cfgPackageLen, (int)cfg_bin->size);
		goto exit;
	}

	/* check firmware's checksum */
	cfg_checksum = getU16(&cfg_bin->data[4]);

	for (i = BIN_CFG_START_LOCAL; i < (cfgPackageLen) ; i++)
		checksum += cfg_bin->data[i];

	if ((checksum) != cfg_checksum) {
		TPD_INFO("%s:Bad firmware!(checksum: 0x%04X, header define: 0x%04X)\n",
				__func__, checksum, cfg_checksum);
		goto exit;
	}
	/* check head end  */

	bin_group_num = cfg_bin->data[MODULE_NUM];
	bin_cfg_num = cfg_bin->data[CFG_NUM];
	TPD_DEBUG("%s:bin_group_num = %d, bin_cfg_num = %d\n",
			__func__, bin_group_num, bin_cfg_num);

	if (!strncmp(cfg_type, GOODIX_TEST_CONFIG, strlen(GOODIX_TEST_CONFIG)))
		config_status = 0;
	else if (!strncmp(cfg_type, GOODIX_NORMAL_CONFIG, strlen(GOODIX_NORMAL_CONFIG)))
		config_status = 1;
	else if (!strncmp(cfg_type, GOODIX_NORMAL_NOISE_CONFIG, strlen(GOODIX_NORMAL_NOISE_CONFIG)))
		config_status = 2;
	else if (!strncmp(cfg_type, GOODIX_GLOVE_CONFIG, strlen(GOODIX_GLOVE_CONFIG)))
		config_status = 3;
	else if (!strncmp(cfg_type, GOODIX_GLOVE_NOISE_CONFIG, strlen(GOODIX_GLOVE_NOISE_CONFIG)))
		config_status = 4;
	else if (!strncmp(cfg_type, GOODIX_HOLSTER_CONFIG, strlen(GOODIX_HOLSTER_CONFIG)))
		config_status = 5;
	else if (!strncmp(cfg_type, GOODIX_HOLSTER_NOISE_CONFIG, strlen(GOODIX_HOLSTER_NOISE_CONFIG)))
		config_status = 6;
	else if (!strncmp(cfg_type, GOODIX_NOISE_TEST_CONFIG, strlen(GOODIX_NOISE_TEST_CONFIG)))
		config_status = 7;
	else {
		TPD_INFO("%s: invalid config text field\n", __func__);
		goto exit;
	}

	cfg_offset = CFG_HEAD_BYTES + bin_group_num * bin_cfg_num * CFG_INFO_BLOCK_BYTES ;
	for (i = 0 ; i < bin_group_num * bin_cfg_num; i++) {
		/* find cfg's sid in cfg.bin */
		one_cfg_count = getU16(&cfg_bin->data[CFG_HEAD_BYTES + 2 + i * CFG_INFO_BLOCK_BYTES]);
		if (sid == (cfg_bin->data[CFG_HEAD_BYTES + i * CFG_INFO_BLOCK_BYTES])) {
			sid_is_exist = GOODIX_EXIST;
			if (config_status == (cfg_bin->data[CFG_HEAD_BYTES + 1 + i * CFG_INFO_BLOCK_BYTES])) {
				memcpy(cfg, &cfg_bin->data[cfg_offset], one_cfg_count);
				*cfg_len = one_cfg_count;
				TPD_DEBUG("%s:one_cfg_count = %d, cfg_data1 = 0x%02x, cfg_data2 = 0x%02x\n",
						__func__, one_cfg_count, cfg[0], cfg[1]);
				break;
			}
		}
		cfg_offset += one_cfg_count;
	}

	if (i >= bin_group_num * bin_cfg_num) {
		TPD_INFO("%s:(not find config ,config_status: %d)\n", __func__, config_status);
		goto exit;
	}

	TPD_DEBUG("%s exit\n", __func__);
	return NO_ERR;
exit:
	return RESULT_ERR;
}

static int goodix_get_cfg_data(void *chip_data_info, const struct firmware *cfg_bin,
		char *config_name, struct goodix_ts_config *config)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	u8 *cfg_data = NULL;
	int cfg_len = 0;
	int ret = NO_ERR;

	TPD_DEBUG("%s run\n", __func__);

	cfg_data = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_data == NULL) {
		TPD_INFO("Memory allco err\n");
		goto exit;
	}

	config->length = 0;
	chip_info->ver_info.sensor_id = 0x01;
	TPD_INFO("use default id is %d\n", chip_info->ver_info.sensor_id);
	/* parse config data */
	ret = goodix_parse_cfg_data(cfg_bin, config_name, cfg_data,
			&cfg_len, chip_info->ver_info.sensor_id);
	if (ret < 0) {
		TPD_INFO("%s: parse %s data failed\n", __func__, config_name);
		ret = -EINVAL;
		goto exit;
	}

	TPD_DEBUG("%s: %s  version:%d , size:%d\n", __func__,
			config_name, cfg_data[0], cfg_len);
	memcpy(config->data, cfg_data, cfg_len);
	config->length = cfg_len;

	strncpy(config->name, config_name, MAX_STR_LEN);

exit:
	if (cfg_data) {
		kfree(cfg_data);
		cfg_data = NULL;
	}
	TPD_DEBUG("%s exit\n", __func__);
	return ret;
}


static int goodix_get_cfg_parms(void *chip_data_info,
		const struct firmware *firmware)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	int ret = 0;

	TPD_DEBUG("%s run\n", __func__);
	if(firmware == NULL) {
		TPD_INFO("%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("%s:Bad firmware!(config firmware data is null: )\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_INFO("%s: cfg_bin_size:%d\n", __func__, (int)firmware->size);
	if (firmware->size > 56) {
		TPD_INFO("cfg_bin head info:%*ph\n", 32, firmware->data);
		TPD_INFO("cfg_bin head info:%*ph\n", 24, firmware->data + 32);
	}

	/* parse normal config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
			GOODIX_NORMAL_CONFIG, &chip_info->normal_cfg);
	if (ret < 0) {
		TPD_INFO("%s: Failed to parse normal_config data:%d\n", __func__, ret);
	} else {
		TPD_DEBUG("%s: parse normal_config data success\n", __func__);
	}

	ret = goodix_get_cfg_data(chip_info, firmware,
			GOODIX_TEST_CONFIG, &chip_info->test_cfg);
	if (ret < 0) {
		TPD_INFO("%s: Failed to parse test_config data:%d\n", __func__, ret);
	} else {
		TPD_DEBUG("%s: parse test_config data success\n", __func__);
	}

	/* parse normal noise config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
			GOODIX_NORMAL_NOISE_CONFIG, &chip_info->normal_noise_cfg);
	if (ret < 0) {
		TPD_INFO("%s: Failed to parse normal_noise_config data\n", __func__);
	} else {
		TPD_DEBUG("%s: parse normal_noise_config data success\n", __func__);
	}

	/* parse noise test config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
			GOODIX_NOISE_TEST_CONFIG, &chip_info->noise_test_cfg);
	if (ret < 0) {
		memcpy(&chip_info->noise_test_cfg, &chip_info->normal_cfg,
				sizeof(chip_info->noise_test_cfg));
		TPD_INFO("%s: Failed to parse noise_test_config data,use normal_config data\n", __func__);
	} else {
		TPD_DEBUG("%s: parse noise_test_config data success\n", __func__);
	}
exit:
	TPD_DEBUG("%s exit:%d\n", __func__, ret);
	return ret;
}

//get fw firmware from firmware
//return value:
//      0: operate success
//      other: failed
static int goodix_get_fw_parms(void *chip_data_info,
		const struct firmware *firmware, struct firmware *fw_firmware)
{
	//struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	int ret = 0;
	int cfgPackageLen = 0;
	int fwPackageLen = 0;

	TPD_DEBUG("%s run\n", __func__);
	if(firmware == NULL) {
		TPD_INFO("%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("%s:Bad firmware!(config firmware data si null)\n", __func__);
		ret = -1;
		goto exit;
	}

	if(fw_firmware == NULL) {
		TPD_INFO("%s:fw_firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_DEBUG("clear fw_firmware\n");
	memset(fw_firmware, 0, sizeof(struct firmware));

	cfgPackageLen = getU32(firmware->data) + BIN_CFG_START_LOCAL;
	TPD_DEBUG("%s cfg package len:%d\n", __func__, cfgPackageLen);

	if(firmware->size <= (cfgPackageLen + 16)) {
		TPD_INFO("%s current firmware does not contain goodix fw\n", __func__);
		TPD_INFO("%s cfg package len:%d,firmware size:%d\n", __func__,
				cfgPackageLen, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	if(!(firmware->data[cfgPackageLen + 0] == 'G' && firmware->data[cfgPackageLen + 1] == 'X' &&
				firmware->data[cfgPackageLen + 2] == 'F' && firmware->data[cfgPackageLen + 3] == 'W')) {
		TPD_INFO("%s can't find fw package\n", __func__);
		TPD_INFO("Data type:%c %c %c %c,dest type is:GXFW\n", firmware->data[cfgPackageLen + 0],
				firmware->data[cfgPackageLen + 1], firmware->data[cfgPackageLen + 2],
				firmware->data[cfgPackageLen + 3]);
		ret = -1;
		goto exit;
	}

	if(firmware->data[cfgPackageLen + 4] != 1) {
		TPD_INFO("%s can't support this ver:%d\n", __func__,
				firmware->data[cfgPackageLen + 4]);
		ret = -1;
		goto exit;
	}

	fwPackageLen =  getU32(firmware->data + cfgPackageLen + 8);

	TPD_DEBUG("%s fw package len:%d\n", __func__, fwPackageLen);
	if((fwPackageLen + 16 + cfgPackageLen) > firmware->size ) {
		TPD_INFO("%s bad firmware,need len:%d,actual firmware size:%d\n",
				__func__, fwPackageLen + 16 + cfgPackageLen, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	fw_firmware->size = fwPackageLen;
	fw_firmware->data = firmware->data + cfgPackageLen + 16;

	TPD_DEBUG("success get fw,len:%d\n", fwPackageLen);
	TPD_DEBUG("fw head info:%*ph\n", 32, fw_firmware->data);
	TPD_DEBUG("fw tail info:%*ph\n", 4, &fw_firmware->data[fwPackageLen - 4 - 1]);
	ret = 0;

exit:
	TPD_DEBUG("%s exit:%d\n", __func__, ret);
	return ret;
}

static fw_update_state goodix_fw_update(void *chip_data,
		const struct firmware *cfg_fw_firmware, bool force)
{

#define FW_UPDATE_RETRY   2
	int retry0 = FW_UPDATE_RETRY;
	int retry1 = FW_UPDATE_RETRY;
	int r, ret;
	struct chip_data_brl *chip_info;
	struct fw_update_ctrl *fwu_ctrl = NULL;
	struct firmware fw_firmware;

	fwu_ctrl = kzalloc(sizeof(struct fw_update_ctrl), GFP_KERNEL);
	if (!fwu_ctrl) {
		TPD_INFO("Failed to alloc memory for fwu_ctrl\n");
		return -ENOMEM;
	}
	chip_info = (struct chip_data_brl *)chip_data;
	fwu_ctrl->chip_info = chip_info;

	r = goodix_get_cfg_parms(chip_data, cfg_fw_firmware);
	if(r < 0) {
		TPD_INFO("%s Failed get cfg from firmware\n", __func__);
	} else {
		TPD_INFO("%s success get ic cfg from firmware\n", __func__);
	}

	r = goodix_get_fw_parms(chip_data, cfg_fw_firmware, &fw_firmware);
	if(r < 0) {
		TPD_INFO("%s Failed get ic fw from firmware\n", __func__);
		goto err_parse_fw;
	} else {
		TPD_INFO("%s success get ic fw from firmware\n", __func__);
	}

	fwu_ctrl->fw_data.firmware = &fw_firmware;
	fwu_ctrl->ic_config = &fwu_ctrl->chip_info->normal_cfg;
	r = goodix_parse_firmware(&fwu_ctrl->fw_data);
	if (r < 0) {
		goto err_parse_fw;
	}

	/* TODO: set force update flag*/
	if (force == false) {
		r = goodix_fw_version_compare(fwu_ctrl);
		if (!r) {
			TPD_INFO("firmware upgraded\n");
			r = FW_NO_NEED_UPDATE;
			goto err_check_update;
		}
	}

start_update:
	do {
		ret = goodix_update_prepare(fwu_ctrl);
		if (ret) {
			TPD_INFO("%s: failed prepare ISP, retry %d\n", __func__,
					FW_UPDATE_RETRY - retry0);
		}
	} while (ret && --retry0 > 0);
	if (ret) {
		TPD_INFO("%s: Failed to prepare ISP, exit update:%d\n",
				__func__, ret);
		goto err_fw_prepare;
	}

	/* progress: 20%~100% */
	ret = goodix_flash_firmware(fwu_ctrl);
	if (ret == -EAGAIN && --retry1 > 0) {
		TPD_INFO("%s: Bus error, retry firmware update:%d\n", __func__,
				FW_UPDATE_RETRY - retry1);
		goto start_update;
	}
	if (ret)
		TPD_INFO("flash fw data enter error\n");
	else
		TPD_INFO("flash fw data success, need check version\n");

err_fw_prepare:
	ret = goodix_update_finish(fwu_ctrl);
	if (!ret) {
		TPD_INFO("Firmware update successfully\n");
		brl_get_ic_info(chip_info, &chip_info->ic_info);
		r = FW_UPDATE_SUCCESS;
	} else {
		TPD_INFO("%s: Firmware update failed\n", __func__);
		r = FW_UPDATE_ERROR;
	}
err_check_update:
err_parse_fw:
	ret = goodix_send_config(chip_info, chip_info->normal_cfg.data, chip_info->normal_cfg.length);

	if(ret < 0) {
		TPD_INFO("%s: send normal cfg failed:%d\n", __func__, ret);
	} else {
		TPD_INFO("%s: send normal cfg success\n", __func__);
	}

	if (fwu_ctrl) {
		kfree(fwu_ctrl);
		fwu_ctrl = NULL;
	}

	return r;
}
/*****************end of GT9886's update function********************/

static u32 goodix_u32_trigger_reason(void *chip_data,
		int gesture_enable, int is_suspended)
{
	int ret = -1;
	u8 touch_num = 0;
	u32 result_event = 0;
	int pre_read_len;
	u8 event_status;
	struct goodix_ic_info_misc *misc;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	memset(chip_info->touch_data, 0, MAX_GT_IRQ_DATA_LENGTH);
	if (chip_info->kernel_grip_support) {
		memset(chip_info->edge_data, 0, MAX_GT_EDGE_DATA_LENGTH);
	}

	misc = &chip_info->ic_info.misc;
	if (!misc->touch_data_addr) {
		TPD_INFO("%s: invalied touch data address\n", __func__);
		return IRQ_IGNORE;
	}
	pre_read_len = IRQ_EVENT_HEAD_LEN +
		BYTES_PER_POINT + COOR_DATA_CHECKSUM_SIZE;

	ret = touch_i2c_read_block_u32(chip_info->client, misc->touch_data_addr,
			pre_read_len, chip_info->touch_data);
	if (ret < 0) {
		TPD_INFO("%s: i2c transfer error!\n", __func__);
		goto IGNORE_CLEAR_IRQ;
	}

	if (checksum_cmp(chip_info->touch_data,
				IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("%s: touch head checksum err\n", __func__);
		TPD_INFO("%s: touch_head %*ph\n", __func__, IRQ_EVENT_HEAD_LEN,
				chip_info->touch_data);
		goto IGNORE_CLEAR_IRQ;
	}

	event_status = chip_info->touch_data[IRQ_EVENT_TYPE_OFFSET];

	if (event_status & GOODIX_TOUCH_EVENT) {
		touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
		touch_num = touch_num < MAX_POINT_NUM ? touch_num : MAX_POINT_NUM;
		if (chip_info->kernel_grip_support) {
			if (touch_num > 0) {
				// TODO add EDGE data info read function
				// ret = touch_i2c_read_block_u32(chip_info->client, edge_data_addr, BYTES_PER_EDGE * touch_num, chip_info->edge_data);
				// if (ret < 0) {
				//     TPD_INFO("%s: i2c transfer error!\n", __func__);
				//     goto IGNORE_CLEAR_IRQ;
				// }
			}
		}
		if (touch_num > 1) {
			ret = touch_i2c_read_block_u32(chip_info->client,
					misc->touch_data_addr + pre_read_len,
					(touch_num - 1) * BYTES_PER_POINT,
					&chip_info->touch_data[pre_read_len]); //read out point data
			if (ret < 0) {
				TPD_INFO("read touch point data from coor_addr failed!\n");
				goto IGNORE_CLEAR_IRQ;
			}
		}

		if (touch_num && checksum_cmp(&chip_info->touch_data[IRQ_EVENT_HEAD_LEN],
					touch_num * BYTES_PER_POINT + 2, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("%s: touch data checksum error\n", __func__);
			TPD_INFO("%s: data:%*ph", __func__, touch_num * BYTES_PER_POINT + 2,&chip_info->touch_data[IRQ_EVENT_HEAD_LEN]);
			goto IGNORE_CLEAR_IRQ;
		}
	} else if (!(event_status & (GOODIX_GESTURE_EVENT | GOODIX_FINGER_IDLE_EVENT))) {
		//TODO check this event
	}

	if (gesture_enable && is_suspended && (event_status & GOODIX_GESTURE_EVENT)) {   //check whether the gesture trigger
		return IRQ_GESTURE;
	} else if (is_suspended) {
		goto IGNORE_CLEAR_IRQ;
	}
	if (event_status & GOODIX_FINGER_IDLE_EVENT) {
		goto IGNORE_IDLETOACTIVE_IRQ;
	}
	if (event_status & GOODIX_REQUEST_EVENT) {     //int request
		return IRQ_FW_CONFIG;
	}

	if (event_status & GOODIX_FINGER_STATUS_EVENT) {
		SET_BIT(result_event, IRQ_FW_HEALTH);
		//goto IGNORE_CLEAR_IRQ;

	}

	if (event_status & GOODIX_TOUCH_EVENT) {
		SET_BIT(result_event, IRQ_TOUCH);
		if ((event_status & GOODIX_FINGER_PRINT_EVENT) &&
				!is_suspended && (chip_info->fp_down_flag == false)) {
			chip_info->fp_down_flag = true;
			SET_BIT(result_event, IRQ_FINGERPRINT);
		} else if (!is_suspended && (event_status & GOODIX_FINGER_PRINT_EVENT) &&
				(chip_info->fp_down_flag == true) ) {
			chip_info->fp_down_flag = false;
			SET_BIT(result_event, IRQ_FINGERPRINT);
		}
	}
	return  result_event;

IGNORE_CLEAR_IRQ:
	TPD_DEBUG("error coor data :%*ph\n", pre_read_len, chip_info->touch_data);
	ret = goodix_clear_irq(chip_info);
	return IRQ_IGNORE;

IGNORE_IDLETOACTIVE_IRQ:
	TPD_DEBUG("idle to active :%*ph\n", pre_read_len, chip_info->touch_data);
	return IRQ_IGNORE;
}

static int goodix_get_touch_points(void *chip_data,
		struct point_info *points, int max_num)
{
	int ret, i;
	int touch_map = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 touch_num = 0;

	u8 finger_processed = 0;
	u8 *coor_data = NULL;
	u8 *ew_data = NULL;
	s32 id = 0;

	touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
	if (touch_num == 0) { //Up event
		goto END_TOUCH;
	}

	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	ew_data = &chip_info->edge_data[0];
	id = (coor_data[0] >> 4) & 0x0F;
	for (i = 0; i < max_num; i++) {
		if (i == id) {
			//finger_processed++;
			//if(finger_processed > touch_num)
			//goto END_TOUCH;

			points[i].x = le16_to_cpup((__le16 *)(coor_data + 2));
			points[i].y = le16_to_cpup((__le16 *)(coor_data + 4));
			points[i].z = coor_data[6];
			points[i].width_major = 30; // any value
			points[i].status = 1;
			points[i].touch_major = coor_data[7];
			points[i].tx_press = ew_data[0];
			points[i].rx_press = ew_data[1];

			if (coor_data[1] & 0x01) {
				// TODO need confirm
				// chip_info->fp_coor_report.fp_x_coor = points[i].x;
				// chip_info->fp_coor_report.fp_y_coor = points[i].y;
				// chip_info->fp_coor_report.fp_area = coor_data[7];
			}

			touch_map |= 0x01 << i;
			coor_data += BYTES_PER_POINT;
			ew_data += BYTES_PER_EDGE;

            if (++finger_processed < touch_num) {
                id = (coor_data[0] >> 4) & 0x0F;
            } else {
                break;
            }
        }
    }
END_TOUCH:
	ret = goodix_clear_irq(chip_info);
	return touch_map;
}


//gesture define
//#define GSX_KEY_DATA_LEN                   42
//#define GSX_GESTURE_TYPE_LEN            32
/**************************************/
/*************************************/
static int goodix_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	int ret = -1;
	uint8_t doze_buf[GSX_KEY_DATA_LEN] = {0};
	uint8_t point_data[MAX_GESTURE_POINT_NUM * 4 + 1];
	uint8_t point_num = 0;
	uint8_t gesture_id = 0;
	struct Coordinate limitPoint[4];
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	ret = touch_i2c_read_block_u32(chip_info->client, chip_info->ic_info.misc.touch_data_addr, sizeof(doze_buf), doze_buf);
	if (ret < 0) {
		TPD_INFO("%s: read gesture info i2c faild\n", __func__);
		return -1;
	}

	gesture_id = doze_buf[4];
	if (gesture_id == 0 || ((doze_buf[0] & GOODIX_GESTURE_EVENT)  == 0)) { //no gesture type, no need handle
		TPD_INFO("Read gesture data faild. doze_buf[0]=0x%x\n", doze_buf[0]);
		goto END_GESTURE;
	}

	if (checksum_cmp(doze_buf, IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("%s: gesture head checksum err\n", __func__);
		TPD_INFO("%s: touch_head %*ph\n", __func__, IRQ_EVENT_HEAD_LEN,
				&doze_buf[0]);
		goto END_GESTURE;
	}

	if (checksum_cmp(&doze_buf[IRQ_EVENT_HEAD_LEN], 34, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("%s: gesture coor data  checksum err\n", __func__);
		TPD_INFO("%s: gesture coor data %*ph\n", __func__, 34,
				&doze_buf[IRQ_EVENT_HEAD_LEN]);
		goto END_GESTURE;
	}

	/*For FP_DOWN_DETECT FP_UP_DETECT no need to read GTP_REG_GESTURE_COOR data*/
	gesture->clockwise = 2;
	switch (gesture_id) {   //judge gesture type //Jarvis: need check with FW.protocol 3
		case FP_DOWN_DETECT:
			gesture->gesture_type = FingerprintDown;
			chip_info->fp_coor_report.fp_x_coor = (doze_buf[8] & 0xFF) | (doze_buf[9] & 0x0F) << 8;
			chip_info->fp_coor_report.fp_y_coor = (doze_buf[10] & 0xFF) | (doze_buf[11] & 0x0F) << 8;
			chip_info->fp_coor_report.fp_area = (doze_buf[16] & 0xFF) | (doze_buf[17] & 0x0F) << 8;

			gesture->Point_start.x = chip_info->fp_coor_report.fp_x_coor;
			gesture->Point_start.y = chip_info->fp_coor_report.fp_y_coor;
			gesture->Point_end.x   = chip_info->fp_coor_report.fp_area;
			chip_info->fp_down_flag = true;
			goto REPORT_GESTURE;
			break;

		case FP_UP_DETECT:
			gesture->gesture_type = FingerprintUp;
			chip_info->fp_coor_report.fp_x_coor = (doze_buf[8] & 0xFF) | (doze_buf[9] & 0x0F) << 8;
			chip_info->fp_coor_report.fp_y_coor = (doze_buf[10] & 0xFF) | (doze_buf[11] & 0x0F) << 8;
			chip_info->fp_coor_report.fp_area = (doze_buf[16] & 0xFF) | (doze_buf[17] & 0x0F) << 8;

			gesture->Point_start.x = chip_info->fp_coor_report.fp_x_coor;
			gesture->Point_start.y = chip_info->fp_coor_report.fp_y_coor;
			gesture->Point_end.x   = chip_info->fp_coor_report.fp_area;

			chip_info->fp_down_flag = false;
			goto REPORT_GESTURE;
			break;
	}

	memset(point_data, 0, sizeof(point_data));
	point_num = doze_buf[3];
	point_num = point_num < MAX_GESTURE_POINT_NUM ? point_num : MAX_GESTURE_POINT_NUM;

	//ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_GESTURE_COOR, point_num * 4 +1, point_data); //havn't add checksum here.
	ret = touch_i2c_read_block_u32(chip_info->client, chip_info->ic_info.misc.touch_data_addr + 42, point_num * 4 +1, point_data);
	if (ret < 0) {
		TPD_INFO("%s: read gesture data i2c faild\n", __func__);
		goto END_GESTURE;
	}

	switch (gesture_id) {   //judge gesture type //Jarvis: need check with FW.protocol 3
		case RIGHT_SLIDE_DETECT :
			gesture->gesture_type  = Left2RightSwip;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case LEFT_SLIDE_DETECT :
			gesture->gesture_type  = Right2LeftSwip;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case DOWN_SLIDE_DETECT  :
			gesture->gesture_type  = Up2DownSwip;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case UP_SLIDE_DETECT :
			gesture->gesture_type  = Down2UpSwip;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case DTAP_DETECT:
			gesture->gesture_type  = DouTap;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			break;

		case STAP_DETECT:
			gesture->gesture_type  = SingleTap;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			break;

		case UP_VEE_DETECT :
			gesture->gesture_type  = UpVee;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case DOWN_VEE_DETECT :
			gesture->gesture_type  = DownVee;
			getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case LEFT_VEE_DETECT:
			gesture->gesture_type = LeftVee;
			getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case RIGHT_VEE_DETECT ://this gesture is C
		case RIGHT_VEE_DETECT2://this gesture is <
			gesture->gesture_type  = RightVee;
			getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			break;

		case CIRCLE_DETECT  :
			gesture->gesture_type = Circle;
			gesture->clockwise = clockWise(&point_data[0], point_num);
			getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
			gesture->Point_1st = limitPoint[0]; //ymin
			gesture->Point_2nd = limitPoint[1]; //xmin
			gesture->Point_3rd = limitPoint[2]; //ymax
			gesture->Point_4th = limitPoint[3]; //xmax
			break;

		case DOUSWIP_DETECT  :
			gesture->gesture_type  = DouSwip;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
			gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
			break;

		case M_DETECT  :
			gesture->gesture_type  = Mgestrue;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
			gesture->Point_end.y = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
			gesture->Point_1st.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			gesture->Point_2nd.x = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_2nd.y = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_3rd.x = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
			gesture->Point_3rd.y = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
			break;

		case W_DETECT :
			gesture->gesture_type  = Wgestrue;
			gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
			gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
			gesture->Point_end.x = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
			gesture->Point_end.y = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
			gesture->Point_1st.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
			gesture->Point_1st.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
			gesture->Point_2nd.x = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
			gesture->Point_2nd.y = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
			gesture->Point_3rd.x = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
			gesture->Point_3rd.y = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
			break;

		default:
			gesture->gesture_type = UnkownGesture;
			break;
	}

REPORT_GESTURE:
	TPD_INFO("%s: gesture_id = 0x%x, gesture_type = %d, clockWise = %d, point:(%d %d)(%d %d)(%d %d)(%d %d)(%d %d)(%d %d)\n",
			__func__, gesture_id, gesture->gesture_type, gesture->clockwise,
			gesture->Point_start.x, gesture->Point_start.y, gesture->Point_end.x, gesture->Point_end.y,
			gesture->Point_1st.x, gesture->Point_1st.y, gesture->Point_2nd.x, gesture->Point_2nd.y,
			gesture->Point_3rd.x, gesture->Point_3rd.y, gesture->Point_4th.x, gesture->Point_4th.y);

END_GESTURE:
	ret = goodix_clear_irq(chip_info);  //clear int
	return 0;

	/************************************************************/
	// TODO need add gesture process code
	gesture->gesture_type = UnkownGesture;
	ret = goodix_clear_irq(chip_info);  //clear int
	return 0;
}

//#define GTP_REG_DEBUG  0x102DC

static void goodix_get_health_info(void *chip_data, struct monitor_data *mon_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_health_info *health_info;
	struct goodix_health_info *health_local = &chip_info->health_info;
	u8 log[50];
	int ret = 0;
	u8 clear_flag = 0;

	ret = touch_i2c_read_block_u32(chip_info->client, chip_info->ic_info.misc.fw_state_addr,50,log);
	if (ret < 0) {
		TPD_INFO("%s: read debug log data i2c faild\n", __func__);
		goto END_HEALTH;
	}
	TPD_INFO("GTP_REG_DEBUG:%*ph\n", 50, log);
	if (checksum_cmp(log, 50, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("%s: read debug log data  checksum erro\n", __func__);
	}

	health_info = (struct goodix_health_info *)log;

	if (health_info->shield_water) {
		mon_data->shield_water++;
		if (tp_debug != 0) {
			TPD_INFO("%s: enter water mode\n", __func__);
		}
	}
	if (health_info->baseline_refresh) {
		switch(health_info->baseline_refresh_type) {
			case BASE_DC_COMPONENT:
				mon_data->reserve1++;
				break;
			case BASE_SYS_UPDATE:
				mon_data->reserve2++;
				break;
			case BASE_NEGATIVE_FINGER:
				mon_data->reserve3++;
				break;
			case BASE_MONITOR_UPDATE:
				mon_data->reserve4++;
				break;
			case BASE_CONSISTENCE:
				mon_data->reserve4++;
				break;
			case BASE_FORCE_UPDATE:
				mon_data->baseline_err++;
				break;
			default:
				break;
		}
		if (tp_debug != 0) {
			TPD_INFO("%s: baseline refresh type: %d \n", __func__, health_info->baseline_refresh_type);
		}
	}
	if (health_info->shield_freq != 0) {
		mon_data->noise_count++;
		if (tp_debug != 0) {
			TPD_INFO("%s: freq before: %d HZ, freq after: %d HZ\n", __func__,health_info->freq_before_l | health_info->freq_before_h<<8,health_info->freq_after_h<< 8 | health_info->freq_after_l);

		}
	}
	if (health_info->fw_rst != 0) {
		switch(health_info->reset_reason) {
			case RST_MAIN_REG:
				mon_data->hard_rst++;
				break;
			case RST_OVERLAY_ERROR:
				mon_data->inst_rst++;
				break;
			case RST_LOAD_OVERLAY:
				mon_data->parity_rst++;
				break;
			case RST_CHECK_PID:
				mon_data->wd_rst++;
				break;
			case RST_CHECK_RAM:
				mon_data->other_rst++;
				break;
			case RST_CHECK_RAWDATA:
				mon_data->other_rst++;
				break;
			default:
				break;
		}

		if (tp_debug != 0) {
			TPD_INFO("%s: fw reset type : %d\n", __func__, health_info->reset_reason);
		}
	}
	if (health_info->shield_palm != 0) {
		mon_data->shield_palm++;
		TPD_DEBUG("%s: enter palm mode\n", __func__);
	}

	if (health_info->baseline_status!= 0) {
		if (tp_debug != 0) {
			TPD_DEBUG("%s: baseline_type is :0x%x \n", __func__,health_info->baseline_study_status);
		}
	}

	if (health_info->ub_freqhop_status != 0) {
		if (tp_debug != 0) {
			TPD_DEBUG("%s: ub_freqhop_status is :0x%x \n", __func__,health_info->ub_freqhop_type);
		}
	}

	if (health_info->noise_stutus != 0) {
		if (tp_debug != 0) {
			TPD_DEBUG("%s: last_frame_noise_value is :0x%x , current_frame_noise_value is :0x%x \n", __func__,health_info->last_noise_value_h << 8 | health_info->last_noise_value_l,health_info->current_noise_value_h << 8 | health_info->current_noise_value_l);
		}
	}

	if (tp_debug != 0) {
		TPD_INFO("%s: abs_max:,abs_avg:      %d    %d \n",__func__,
				(int) (health_info->abs_max_h <<8 | health_info->abs_max_l),(int) ( health_info->abs_avg_h << 8 | health_info->abs_avg_l));

		TPD_INFO("%s: mutual_diff_max, mutual_diff_min:      %d    %d \n",__func__,
				health_info->mutualdif_max_h << 8 | health_info->mutualdif_max_l, health_info->mutualdif_min_h << 8 | health_info->mutualdif_min_l);

		TPD_INFO("%s: self_tx_diff_max:,self_txdiff_min:      %d    %d \n",__func__,
				health_info->selftx_diff_max_h << 8 | health_info->selftx_diff_max_l, health_info->selftx_diff_min_h<<8 | health_info->selftx_diff_min_l);

		TPD_INFO("%s: self_rx_diff_max:,self_rxdiff_min:      %d    %d \n",__func__,
				health_info->selfrx_diff_max_h<<8 | health_info->selfrx_diff_max_l, health_info->selfrx_diff_min_h<<8 | health_info->selfrx_diff_min_l);

		TPD_INFO("%s: touch_num:,rect_num:      %d    %d \n",__func__,health_info->touch_num, health_info->rectnum);
	}

	mon_data->shield_esd = chip_info->esd_err_count;
	mon_data->reserve5 =  chip_info->send_cmd_err_count;
	memcpy(health_local, health_info, sizeof(struct goodix_health_info));

	ret = touch_i2c_write_block_u32(chip_info->client,chip_info->ic_info.misc.fw_state_addr,1,&clear_flag);
	if (ret < 0) {
		TPD_INFO("%s: clear debug log data i2c faild\n", __func__);
	}

END_HEALTH:

	// TOO  add health info
	ret = goodix_clear_irq(chip_info);  //clear int
	return;
}

static int goodix_mode_switch(void *chip_data, work_mode mode, bool flag)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (!chip_info->ic_info.length) {
		TPD_INFO("%s: goodix ic info invalid\n", __func__);
		return ret;
	}
	if (chip_info->halt_status && (mode != MODE_NORMAL)) {
		goodix_reset(chip_info);
	}

	switch(mode) {
		case MODE_NORMAL:
			ret = 0;    //after reset, it's already normal
			break;

		case MODE_SLEEP:
			ret = goodix_enter_sleep(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: goodix enter sleep failed\n", __func__);
			}
			break;

		case MODE_GESTURE:
			ret = goodix_enable_gesture(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: goodix enable:(%d) gesture failed.\n", __func__, flag);
				return ret;
			}
			break;

		case MODE_LIMIT_SWITCH:
			ret = goodix_enable_edge_limit(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: goodix enable:(%d) edge limit failed.\n", __func__, flag);
				return ret;
			}
			break;

		case MODE_CHARGE:
			ret = goodix_enable_charge_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
			}
			break;

		case MODE_GAME:
			ret = goodix_enable_game_mode(chip_info, flag);
			if (ret < 0) {
				TPD_INFO("%s: enable game mode : %d failed\n", __func__, flag);
			}
			break;

		default:
			TPD_INFO("%s: mode %d not support.\n", __func__, mode);
	}

	return ret;
}

static int goodix_esd_handle(void *chip_data) //Jarvis:have not finished
{
	s32 ret = -1;
	u8 esd_buf = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (!chip_info->esd_check_enabled || !misc->esd_addr) {
		TPD_DEBUG("%s: close\n", __func__);
		return 0;
	}

	ret = touch_i2c_read_block_u32(chip_info->client, misc->esd_addr, 1, &esd_buf);
	if ((ret < 0) || esd_buf == 0xAA) {
		TPD_INFO("%s: esd dynamic esd occur, ret = %d, esd_buf = %d.\n",
				__func__, ret, esd_buf);
		TPD_INFO("%s: IC works abnormally! Process esd reset.\n", __func__);
		disable_irq_nosync(chip_info->client->irq);

		goodix_power_control(chip_info, false);
		msleep(30);
		goodix_power_control(chip_info, true);
		msleep(10);

		goodix_reset(chip_data); // reset function have rewrite the esd reg no need to do it again

		tp_touch_btnkey_release();

		enable_irq(chip_info->client->irq);
		TPD_INFO("%s: Goodix esd reset over.\n", __func__);
		chip_info->esd_err_count ++;
		return -1;
	} else {
		esd_buf = 0xAA;
		ret = touch_i2c_write_block_u32(chip_info->client,
				chip_info->ic_info.misc.esd_addr, 1, &esd_buf);
		if (ret < 0) {
			TPD_INFO("%s: Failed to reset esd reg.\n", __func__);
		}
	}

	return 0;
}

static void goodix_enable_fingerprint_underscreen(void *chip_data, uint32_t enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("%s, enable = %d\n", __func__, enable);
	if (enable) {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_FOD_FINGER_PRINT, 0);
	} else {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_FOD_FINGER_PRINT, 1);
	}

	return;
}

static void goodix_enable_gesture_mask(void *chip_data, uint32_t enable)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("%s, enable = %d\n", __func__, enable);
	if (enable) {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GESTURE_MASK, 1);/* enable all gesture */
	} else {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_GESTURE_MASK, 0);/* mask all gesture */
	}

	return;
}

static void goodix_screenon_fingerprint_info(void *chip_data,
		struct fp_underscreen_info *fp_tpinfo)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (chip_info->fp_down_flag) {
		fp_tpinfo->x = chip_info->fp_coor_report.fp_x_coor;
		fp_tpinfo->y = chip_info->fp_coor_report.fp_y_coor;
		fp_tpinfo->area_rate = chip_info->fp_coor_report.fp_area;
		fp_tpinfo->touch_state = FINGERPRINT_DOWN_DETECT;
	} else {
		fp_tpinfo->x = chip_info->fp_coor_report.fp_x_coor;
		fp_tpinfo->y = chip_info->fp_coor_report.fp_y_coor;
		fp_tpinfo->area_rate = chip_info->fp_coor_report.fp_area;
		fp_tpinfo->touch_state = FINGERPRINT_UP_DETECT;
	}
}

static int goodix_request_event_handler(struct chip_data_brl *chip_info)
{
	int ret = -1;
	u8 rqst_code = 0;

	rqst_code = chip_info->touch_data[REQUEST_EVENT_TYPE_OFFSET];
	TPD_INFO("%s: request state:0x%02x.\n", __func__, rqst_code);

	switch (rqst_code) {
		case GTP_RQST_CONFIG:
			TPD_INFO("HW request config.\n");
			ret = goodix_send_config(chip_info, chip_info->normal_cfg.data,
					chip_info->normal_cfg.length);
			if (ret) {
				TPD_INFO("request config, send config faild.\n");
			}
			break;
		case GTP_RQST_RESET:
			TPD_INFO("%s: HW requset reset.\n", __func__);
			goodix_reset(chip_info);
			break;
		default:
			TPD_INFO("%s: Unknown hw request:%d.\n", __func__, rqst_code);
			break;
	}

	goodix_clear_irq(chip_info);
	return 0;
}

static int goodix_fw_handle(void *chip_data)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	ret = goodix_request_event_handler(chip_info);
	ret |= goodix_clear_irq(chip_info);

	return ret;
}

static void goodix_register_info_read(void *chip_data,
		uint16_t register_addr, uint8_t *result, uint8_t length)
{
	//struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	// TODO need change framework to support u32 address
	//touch_i2c_read_block_u32(chip_info->client, register_addr, length, result);         /*read data*/
}

static void goodix_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	chip_info->touch_direction = dir;
}

static uint8_t goodix_get_touch_direction(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	return chip_info->touch_direction;
}

static void goodix_specific_resume_operate(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	TPD_DEBUG("%s call\n", __func__);
	goodix_esd_check_enable(chip_info, true);
}

static int goodix_enable_single_tap(void *chip_data, bool enable)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	chip_info->single_tap_flag = enable;
	return 0;
}

struct touchpanel_operations brl_goodix_ops = {
	.ftm_process                 = goodix_ftm_process,
	.get_vendor                  = goodix_get_vendor,
	.get_util_vendor             = goodix_util_get_vendor,
	.get_chip_info               = goodix_get_chip_info,
	.reset                       = goodix_reset,
	.power_control               = goodix_power_control,
	.fw_check                    = goodix_fw_check,
	.fw_update                   = goodix_fw_update,
	.u32_trigger_reason          = goodix_u32_trigger_reason,
	.get_touch_points            = goodix_get_touch_points,
	.get_gesture_info            = goodix_get_gesture_info,
	.mode_switch                 = goodix_mode_switch,
	.esd_handle                  = goodix_esd_handle,
	.fw_handle                   = goodix_fw_handle,
	.register_info_read          = goodix_register_info_read,
	.get_usb_state               = goodix_get_usb_state,
	.enable_fingerprint          = goodix_enable_fingerprint_underscreen,
	.enable_gesture_mask         = goodix_enable_gesture_mask,
	.screenon_fingerprint_info   = goodix_screenon_fingerprint_info,
	.set_touch_direction         = goodix_set_touch_direction,
	.get_touch_direction         = goodix_get_touch_direction,
	.specific_resume_operate     = goodix_specific_resume_operate,
	.gt_health_report               = goodix_get_health_info,
	.enable_single_tap           = goodix_enable_single_tap,
};
/********* End of implementation of touchpanel_operations callbacks**********************/

/******** Start of implementation of debug_info_proc_operations callbacks*********************/
static void goodix_debug_info_read(struct seq_file *s,
		void *chip_data, debug_type debug_type)
{
	int ret = -1, i = 0, j = 0;
	u8 *kernel_buf = NULL;
	int addr = 0;
	u8 clear_state = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info *ic_info;
	int TX_NUM = 0;
	int RX_NUM = 0;
	s16 data = 0;

	// /*keep in active mode*/
	// goodix_send_cmd_simple(chip_info, GTP_CMD_ENTER_DOZE_TIME, 0xFF);

	ic_info = &chip_info->ic_info;
	RX_NUM = ic_info->parm.sen_num;
	TX_NUM = ic_info->parm.drv_num;
	if (!TX_NUM || !RX_NUM) {
		TPD_INFO("%s: error invalid tx %d or rx %d num\n",
				__func__, TX_NUM, RX_NUM);
		return;
	}

	kernel_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if(kernel_buf == NULL) {
		TPD_INFO("%s kmalloc error\n", __func__);
		return;
	}
	switch (debug_type) {
		case GTP_RAWDATA:
			addr = ic_info->misc.mutual_rawdata_addr;
			break;
		case GTP_DIFFDATA:
			addr = ic_info->misc.mutual_diffdata_addr;
			break;
		default:
			addr = ic_info->misc.mutual_refdata_addr;
			break;
	}
	gt8x_rawdiff_mode = 1;
	goodix_send_cmd_simple(chip_info, GTP_CMD_RAWDATA, 0);
	msleep(20);
	touch_i2c_write_block_u32(chip_info->client, ic_info->misc.touch_data_addr, 1, &clear_state);

	//wait for data ready
	while(i++ < 10) {
		ret = touch_i2c_read_block_u32(chip_info->client, ic_info->misc.touch_data_addr, 1, kernel_buf);
		TPD_INFO("ret = %d  kernel_buf = %d\n", ret, kernel_buf[0]);
		if(!ret && (kernel_buf[0] & 0x80)) {
			TPD_INFO("Data ready OK\n");
			break;
		}
		msleep(20);
	}
	if(i >= 10) {
		TPD_INFO("data not ready, quit!\n");
		goto read_data_exit;
	}

	ret = touch_i2c_read_block_u32(chip_info->client, addr, TX_NUM * RX_NUM * 2, kernel_buf);
	msleep(5);

	for(i = 0; i < RX_NUM; i++) {
		seq_printf(s, "[%2d] ", i);
		for(j = 0; j < TX_NUM; j++) {
			data = kernel_buf[j * RX_NUM * 2 + i * 2] + (kernel_buf[j * RX_NUM * 2 + i * 2 + 1] << 8);
			seq_printf(s, "%4d ", data);
		}
		seq_printf(s, "\n");
	}
read_data_exit:
	goodix_send_cmd_simple(chip_info, GTP_CMD_NORMAL, 0);
	gt8x_rawdiff_mode = 0;
	touch_i2c_write_block_u32(chip_info->client, ic_info->misc.touch_data_addr, 1, &clear_state);
	// /*be normal in idle*/
	// goodix_send_cmd_simple(chip_info, GTP_CMD_DEFULT_DOZE_TIME, 0x00);

	kfree(kernel_buf);
	return;
}

static void goodix_down_diff_info_read(struct seq_file *s, void *chip_data)
{
	//TODO need add down diff addr info to ic_info struct
	int ret = -1, i = 0, j = 0;
	u8 *kernel_buf = NULL;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info *ic_info;
	int TX_NUM = 0;
	int RX_NUM = 0;
	s16 diff_data = 0;

	ic_info = &chip_info->ic_info;

	kernel_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if(kernel_buf == NULL) {
		TPD_INFO("%s kmalloc error\n", __func__);
		return;
	}

	RX_NUM = ic_info->parm.sen_num;
	TX_NUM = ic_info->parm.drv_num;

	if (TX_NUM > 50 || RX_NUM > 50) {
		TPD_INFO("%s first finger down diff data is invalid.\n", __func__);
		goto exit;
	}

	if ((TX_NUM * RX_NUM * 2+2) > PAGE_SIZE) {
		TPD_INFO("%s: data invalid, tx:%d,rx :%d\n", __func__, TX_NUM, RX_NUM);
		goto exit;
	}

	//ret = touch_i2c_read_block_u32(chip_info->client, ic_info->misc.touch_data_addr, TX_NUM * RX_NUM * 2 + 2, kernel_buf);
	ret = touch_i2c_read_block_u32(chip_info->client, 0x17DE8, TX_NUM * RX_NUM * 2 + 2 , kernel_buf);
	if (ret < 0) {
		TPD_INFO("%s failed to read down diff data.\n", __func__);
		goto exit;
	}

	seq_printf(s, "\nfinger first down diff data:\n");

	for(i = 0; i < RX_NUM; i++) {
		seq_printf(s, "[%2d] ", i);
		for(j = 0; j < TX_NUM; j++) {
			diff_data = (kernel_buf[j * RX_NUM * 2 + i * 2 + 2]) + (kernel_buf[j * RX_NUM * 2 + i * 2 + 1 + 2] <<8);
			seq_printf(s, "%4d ", diff_data);
		}
		seq_printf(s, "\n");
	}

exit:

	if (kernel_buf) {
		kfree(kernel_buf);
	}

	return;
}

//proc/touchpanel/debug_info/delta
static void goodix_delta_read(struct seq_file *s, void *chip_data)
{
	//struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	//int ret =0;
	goodix_debug_info_read(s, chip_data, GTP_DIFFDATA);
	if (tp_debug) {
		goodix_down_diff_info_read(s, chip_data);
	}
}

//proc/touchpanel/debug_info/baseline
static void goodix_baseline_read(struct seq_file *s, void *chip_data)
{
	goodix_debug_info_read(s, chip_data, GTP_RAWDATA);
}

//proc/touchpanel/debug_info/main_register
static void goodix_main_register_read(struct seq_file *s, void *chip_data)//Jarvis:need change to GT9886's register
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info_misc *ic_info_misc;
	int ret = 0;
	u8 touch_data[IRQ_EVENT_HEAD_LEN + BYTES_PER_POINT + COOR_DATA_CHECKSUM_SIZE] = {0};

	ic_info_misc = &chip_info->ic_info.misc;
	seq_printf(s, "====================================================\n");
	if(chip_info->p_tp_fw) {
		seq_printf(s, "tp fw = 0x%s\n", chip_info->p_tp_fw);
	}
	ret = touch_i2c_read_block_u32(chip_info->client, ic_info_misc->touch_data_addr,
			sizeof(touch_data), touch_data);
	if (ret < 0) {
		TPD_INFO("%s: i2c transfer error!\n", __func__);
		goto out;
	}
	seq_printf(s, "cached touch_data: %*ph\n", (int)sizeof(touch_data), chip_info->touch_data);
	TPD_INFO("%s: cached touch_data: %*ph\n", __func__, (int)sizeof(touch_data), chip_info->touch_data);
	// TODO need confirm the data, address and length
	seq_printf(s, "current touch_data: %*ph\n", (int)sizeof(touch_data), touch_data);
	TPD_INFO("%s: current touch_data: %*ph\n", __func__, (int)sizeof(touch_data), touch_data);
	seq_printf(s, "====================================================\n");
out:
	return;
}

static struct debug_info_proc_operations debug_info_proc_ops = {
	.limit_read         = goodix_limit_read,
	.delta_read         = goodix_delta_read,
	.baseline_read      = goodix_baseline_read,
	.main_register_read = goodix_main_register_read,
};
/********* End of implementation of debug_info_proc_operations callbacks**********************/

/************** Start of callback of proc/Goodix/config_version node**************************/

/* success return config_len, <= 0 failed */
static int goodix_read_config(struct chip_data_brl *chip_info, u8 *cfg, int size)
{
	int ret;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;
	struct goodix_config_head cfg_head;

	if (!cfg)
		return -EINVAL;

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_START;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("%s: failed send config read prepare command\n", __func__);
		return ret;
	}

	ret = touch_i2c_read_block_u32(chip_info->client, misc->fw_buffer_addr,
			sizeof(cfg_head), cfg_head.buf);
	if (ret) {
		TPD_INFO("%s: failed read config head %d\n", __func__, ret);
		goto exit;
	}

	if (checksum_cmp(cfg_head.buf, sizeof(cfg_head), CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("%s: config head checksum error\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	cfg_head.cfg_len = le16_to_cpu(cfg_head.cfg_len);
	if (cfg_head.cfg_len > misc->fw_buffer_max_len ||
			cfg_head.cfg_len > size) {
		TPD_INFO("%s: cfg len exceed buffer size %d > %d\n", __func__, cfg_head.cfg_len,
				misc->fw_buffer_max_len);
		ret = -EINVAL;
		goto exit;
	}

	memcpy(cfg, cfg_head.buf, sizeof(cfg_head));
	ret = touch_i2c_read_block_u32(chip_info->client, misc->fw_buffer_addr + sizeof(cfg_head),
			cfg_head.cfg_len, cfg + sizeof(cfg_head));
	if (ret) {
		TPD_INFO("%s: failed read cfg pack, %d\n", __func__, ret);
		goto exit;
	}

	TPD_INFO("config len %d\n", cfg_head.cfg_len);
	if (checksum_cmp(cfg + sizeof(cfg_head),
				cfg_head.cfg_len, CHECKSUM_MODE_U16_LE)) {
		TPD_INFO("%s: config body checksum error\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	TPD_INFO("success read config data: len %zu,config_version is : %d\n",
			cfg_head.cfg_len + sizeof(cfg_head),cfg[34]);
exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("%s: failed send config read finish command\n", __func__);
		ret = -EINVAL;
	}
	if (ret)
		return -EINVAL;
	return cfg_head.cfg_len + sizeof(cfg_head);
}

static void goodix_config_info_read(struct seq_file *s, void *chip_data)
{
	int ret = 0, i = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version *fw_version = &chip_info->ver_info;
	struct goodix_ic_info *ic_info = &chip_info->ic_info;

	seq_printf(s, "==== Goodix default config setting in driver====\n");
	for(i = 0; i < TS_CFG_MAX_LEN && i < chip_info->normal_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->normal_cfg.data[i]);
		if(i % 10 == 9)
			seq_printf(s, "\n");
	}
	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix test cfg in driver====\n");
	for(i = 0; i < TS_CFG_MAX_LEN && i < chip_info->test_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->test_cfg.data[i]);
		if(i % 10 == 9)
			seq_printf(s, "\n");
	}
	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix noise test cfg in driver====\n");
	for(i = 0; i < TS_CFG_MAX_LEN && i < chip_info->noise_test_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->noise_test_cfg.data[i]);
		if(i % 10 == 9)
			seq_printf(s, "\n");
	}
	seq_printf(s, "\n");

	seq_printf(s, "==== Goodix config read from chip====\n");

	ret = brl_get_ic_info(chip_info, ic_info);
	if (ret) {
		TPD_INFO("%s: failed get ic info, ret %d\n", __func__, ret);
		goto exit;
	}
	ret = brl_read_version(chip_info, fw_version);
	if(ret) {
		TPD_INFO("goodix_config_info_read goodix_read_config error:%d\n", ret);
		goto exit;
	}

	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix Version Info ====\n");
	seq_printf(s, "ConfigVer: %02X\n", ic_info->version.config_version);
	seq_printf(s, "ProductID: GT%s\n", fw_version->patch_pid);
	seq_printf(s, "PatchID: %*ph\n", (int)sizeof(fw_version->patch_pid), fw_version->patch_pid);
	seq_printf(s, "MaskID: %*ph\n", (int)sizeof(fw_version->rom_vid), fw_version->rom_vid);
	seq_printf(s, "SensorID: %02X\n", fw_version->sensor_id);

exit:
	return;
}
/*************** End of callback of proc/Goodix/config_version node***************************/

/************** Start of atuo test func**************************/
static int init_test_config(struct goodix_ts_test *ts_test)
{
	int ret = 0;

	return ret;
}

static void goodix_print_testdata(struct goodix_ts_test *ts_test)
{
	struct ts_test_params *test_params = NULL;
	int i = 0;
	test_params = &ts_test->test_params;

	if (tp_debug < 2)
		return;
	TPD_DEBUG("rawdata:\n");
	for(i = 0; i < test_params->drv_num * test_params->sen_num; i++) {
		TPD_DEBUG("%d,", ts_test->rawdata.data[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("noisedata:\n");
	for(i = 0; i < test_params->drv_num * test_params->sen_num; i++) {
		TPD_DEBUG("%d,", ts_test->noisedata.data[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("self_rawdata:\n");
	for(i = 0; i < test_params->drv_num + test_params->sen_num; i++) {
		TPD_DEBUG("%d,", ts_test->self_rawdata.data[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("self_noisedata:\n");
	for(i = 0; i < test_params->drv_num + test_params->sen_num; i++) {
		TPD_DEBUG("%d,", ts_test->self_noisedata.data[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}
}

static void goodix_print_test_params(struct goodix_ts_test *ts_test)
{
	struct ts_test_params *test_params = NULL;
	int i = 0;
	test_params = &ts_test->test_params;

	if (tp_debug < 2)
		return;
	TPD_DEBUG("sen_num:%d,drv_num:%d\n", test_params->sen_num, test_params->drv_num);
	TPD_DEBUG("rawdata_addr:%d,noisedata_addr:%d\n", test_params->rawdata_addr, test_params->noisedata_addr);
	TPD_DEBUG("self_rawdata_addr:%d,self_noisedata_addr:%d\n", test_params->self_rawdata_addr, test_params->self_noisedata_addr);
	TPD_DEBUG("basedata_addr:%d\n", test_params->basedata_addr);

	TPD_DEBUG("max_limits:\n");
	for(i = 0; i < test_params->drv_num * test_params->sen_num; i++) {
		TPD_DEBUG("%d,", test_params->max_limits[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("min_limits:\n");
	for(i = 0; i < test_params->drv_num * test_params->sen_num; i++) {
		TPD_DEBUG("%d,", test_params->min_limits[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("deviation_limits:\n");
	for(i = 0; i < test_params->drv_num * test_params->sen_num; i++) {
		TPD_DEBUG("%d,", test_params->deviation_limits[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("self_max_limits:\n");
	for(i = 0; i < test_params->drv_num + test_params->sen_num; i++) {
		TPD_DEBUG("%d,", test_params->self_max_limits[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}

	TPD_DEBUG("self_min_limits:\n");
	for(i = 0; i < test_params->drv_num + test_params->sen_num; i++) {
		TPD_DEBUG("%d,", test_params->self_min_limits[i]);
		if (!((i + 1) % test_params->sen_num) && (i != 0))
			TPD_DEBUG("\n");
	}
}

static int goodix_init_testlimits(struct goodix_ts_test *ts_test)
{
	struct goodix_testdata *p_testdata = ts_test->p_testdata;
	struct test_item_info *p_test_item_info = NULL;
	/*for item parameter data*/
	uint32_t *para_data32 = NULL;
	uint32_t para_num = 0;
	int ret = 0;
	struct ts_test_params *test_params = NULL;

	test_params = &ts_test->test_params;
	test_params->drv_num = p_testdata->TX_NUM;
	test_params->sen_num = p_testdata->RX_NUM;
	TPD_INFO("sen_num:%d,drv_num:%d\n", test_params->sen_num, test_params->drv_num);

	para_data32 = getpara_for_item(p_testdata->fw, TYPE_NOISE_DATA_LIMIT, &para_num);
	if(para_data32) {
		ts_test->is_item_support[TYPE_NOISE_DATA_LIMIT] = para_data32[0];
		if (para_num >= 2) {
			test_params->noise_threshold = para_data32[1];
		}
	} else {
		TPD_INFO("%s: Failed get %s\n", __func__, CSV_TP_NOISE_LIMIT);
		ret = -1;
		goto INIT_LIMIT_END;
	}

	para_data32 = getpara_for_item(p_testdata->fw, TYPE_SHORT_THRESHOLD, &para_num);
	if(para_data32) {
		ts_test->is_item_support[TYPE_SHORT_THRESHOLD] = para_data32[0];
		TPD_INFO("get TYPE_SHORT_THRESHOLD data,len:%d\n", para_num);
		if (para_num >= 8) {
			// store data to test_parms 
			test_params->short_threshold = para_data32[1];
			test_params->r_drv_drv_threshold = para_data32[2];
			test_params->r_drv_sen_threshold = para_data32[3];
			test_params->r_sen_sen_threshold = para_data32[4];
			test_params->r_drv_gnd_threshold = para_data32[5];
			test_params->r_sen_gnd_threshold = para_data32[6];
			test_params->avdd_value = para_data32[7];
		}
	} else {
		TPD_INFO("%s: Failed get %s\n", __func__, CSV_TP_SHORT_THRESHOLD);
		ret = -1;
		goto INIT_LIMIT_END;
	}

	p_test_item_info = get_test_item_info(p_testdata->fw, TYPE_SPECIAL_RAW_MAX_MIN);
	if(p_test_item_info) {
		if (p_test_item_info->para_num) {
			if (p_test_item_info->p_buffer[0]) {
				TPD_INFO("get TYPE_SPECIAL_RAW_MAX_MIN data,len:%d\n", para_num);
				ts_test->is_item_support[TYPE_SPECIAL_RAW_MAX_MIN] = p_test_item_info->p_buffer[0];
				if (p_test_item_info->item_limit_type == LIMIT_TYPE_MAX_MIN_DATA) {
					test_params->max_limits = (uint32_t *)(p_testdata->fw->data + p_test_item_info->top_limit_offset);
					test_params->min_limits = (uint32_t *)(p_testdata->fw->data + p_test_item_info->floor_limit_offset);
				} else {
					TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_RAW_MAX_MIN);
				}
			}
		}
	} else {
		ret = -1;
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_RAW_MAX_MIN);
	}

	p_test_item_info = get_test_item_info(p_testdata->fw, TYPE_SPECIAL_RAW_DELTA);
	if (p_test_item_info) {
		if (p_test_item_info->para_num) {
			if (p_test_item_info->p_buffer[0]) {
				TPD_INFO("get TYPE_SPECIAL_RAW_DELTA data,len:%d\n", para_num);
				ts_test->is_item_support[TYPE_SPECIAL_RAW_DELTA] = p_test_item_info->p_buffer[0];
				if (p_test_item_info->item_limit_type == LIMIT_TYPE_MAX_MIN_DATA) {
					test_params->deviation_limits = (uint32_t *)(p_testdata->fw->data + p_test_item_info->top_limit_offset);
				} else {
					TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_RAW_DELTA);
				}
			}
		}
	} else {
		ret = -1;
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_RAW_DELTA);
	}

	para_data32 = getpara_for_item(p_testdata->fw, TYPE_NOISE_SLEFDATA_LIMIT, &para_num);
	if(para_data32) {
		ts_test->is_item_support[TYPE_NOISE_SLEFDATA_LIMIT] = para_data32[0];
		if (para_num >= 2) {
			// store data to test_parms 
			test_params->self_noise_threshold = para_data32[1];
		}
	} else {
		TPD_INFO("%s: Failed get %s\n", __func__, CSV_TP_SELFNOISE_LIMIT);
		ret = -1;
		goto INIT_LIMIT_END;
	}

	p_test_item_info = get_test_item_info(p_testdata->fw, TYPE_SPECIAL_SELFRAW_MAX_MIN);
	if(p_test_item_info) {
		if (p_test_item_info->para_num) {
			if (p_test_item_info->p_buffer[0]) {
				ts_test->is_item_support[TYPE_SPECIAL_SELFRAW_MAX_MIN] = p_test_item_info->p_buffer[0];
				if (p_test_item_info->item_limit_type == IMIT_TYPE_SLEFRAW_DATA) {
					TPD_INFO("get IMIT_TYPE_SLEFRAW_DATA data\n");
					test_params->self_max_limits = (int32_t *)(p_testdata->fw->data + p_test_item_info->top_limit_offset);
					test_params->self_min_limits = (int32_t *)(p_testdata->fw->data + p_test_item_info->floor_limit_offset);
				} else {
					TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_SELFRAW_MAX_MIN);
				}
			} else {
				TPD_INFO("TYPE_SPECIAL_SELFRAW_MAX_MIN para_num is invalid\n");
			}
		}
	} else {
		ret = -1;
		TPD_INFO("item: %d get_test_item_info fail\n", TYPE_SPECIAL_SELFRAW_MAX_MIN);
	}

	ret |= init_test_config(ts_test);

INIT_LIMIT_END:
	if (p_test_item_info) {
		kfree(p_test_item_info);
	}
	return ret;
}

static int goodix_init_params(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	struct ts_test_params *test_params = &ts_test->test_params;

	TPD_INFO("%s run\n", __func__);

	test_params->rawdata_addr = chip_info->ic_info.misc.mutual_rawdata_addr;
	test_params->noisedata_addr = chip_info->ic_info.misc.mutual_diffdata_addr;
	test_params->self_rawdata_addr =  chip_info->ic_info.misc.self_rawdata_addr;
	test_params->self_noisedata_addr = chip_info->ic_info.misc.self_diffdata_addr;
	test_params->basedata_addr = chip_info->ic_info.misc.mutual_refdata_addr;
	test_params->max_drv_num = MAX_DRV_NUM;
	test_params->max_sen_num = MAX_SEN_NUM;
	test_params->drv_map = gt9897_drv_map;
	test_params->sen_map = gt9897_sen_map;
	TPD_INFO("%s exit:%d\n", __func__, ret);
	return ret;
}


/* goodix_read_origconfig
 *
 * read original config data
 */
static int goodix_cache_origconfig(struct goodix_ts_test *ts_test)
{
	int ret = -ENODEV;

	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("goodix_cache_origconfig run\n");

	ret = goodix_read_config(chip_info, &ts_test->orig_config.data[0], TS_CFG_MAX_LEN);
	if (ret < 0) {
		TPD_INFO("Failed to read original config data\n");
		return ret;
	}
	TPD_INFO("goodix_cache_origconfig read original config success\n");
	/*
	   ts_test->orig_config.data[1] |= GOODIX_CONFIG_REFRESH_DATA;
	   checksum = checksum_u8(&ts_test->orig_config.data[0], 3);
	   ts_test->orig_config.data[3] = (u8)(0 - checksum);
	 */
	ts_test->orig_config.length = ret;
	strcpy(ts_test->orig_config.name, "original_config");

	TPD_INFO("goodix_cache_origconfig exit\n");
	return NO_ERR;
}

/* goodix_tptest_prepare
 *
 * preparation before tp test
 */
static int goodix_tptest_prepare(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("TP test preparation\n");
	ret = goodix_cache_origconfig(ts_test);
	if (ret) {
		TPD_INFO("Failed cache origin config\n");
		return ret;
	}

	/* init reg addr and short cal map */
	ret = goodix_init_params(ts_test);
	if (ret) {
		TPD_INFO("Failed init register address\n");
		return ret;
	}
	/* parse test limits from csv */
	ret = goodix_init_testlimits(ts_test);
	if (ret < 0) {
		TPD_INFO("Failed to init testlimits from csv:%d\n", ret);
		ts_test->error_count ++;
		seq_printf(ts_test->p_seq_file, "Failed to init testlimits from csv\n");
	}
	ret = goodix_send_config(chip_info, chip_info->noise_test_cfg.data, chip_info->noise_test_cfg.length);
	if (ret < 0) {
		TPD_INFO("Failed to send noise test config config:%d,noise_config len:%d\n", ret, chip_info->noise_test_cfg.length);
	}
	TPD_INFO("send noise test config success :%d,noise_config len:%d\n", ret, chip_info->noise_test_cfg.length);
	msleep(200);

	return ret;
}

/* goodix_tptest_finish
 *
 * finish test
 */
static int goodix_tptest_finish(struct goodix_ts_test *ts_test)
{
	int ret = RESULT_ERR;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("TP test finish\n");
	goodix_reset(chip_info);

	//    if (goodix_set_i2c_doze_mode(core_data->ts_dev,true))
	//        TPD_INFO("WRNING: doze may enabled after reset\n");

	ret = goodix_send_config(chip_info, ts_test->orig_config.data, ts_test->orig_config.length);   
	// ret = goodix_send_config(chip_info, chip_info->noise_test_cfg.data, chip_info->noise_test_cfg.length);
	if(ret < 0) {
		TPD_INFO("goodix_tptest_finish:Failed to send normal config:%d\n", ret);
	} else {
		TPD_INFO("goodix_tptest_finish send normal config succesful\n");
	}

	return ret;
}

static int normandy_sync_read_rawdata(struct i2c_client *client,
		unsigned int sync_addr, unsigned char sync_mask,
		unsigned int reg_addr, unsigned char *data,
		unsigned int len)
{
	int ret = 0;
	int retry_times = 0;
	unsigned char sync_data[0];

	TPD_INFO("%s run,reg_addr:0x%04x,len:%d,sync_addr:0x%04x,sync_mask:0x%x\n",
			__func__, reg_addr, len, sync_addr, sync_mask);

	while(retry_times ++ < 200) {
		sync_data[0] = 0x00;
		msleep(10);
		ret = touch_i2c_read_block_u32(client, sync_addr, 1, sync_data);
		if(ret < 0 || ((sync_data[0] & sync_mask) == 0)) {
			TPD_INFO("%s sync invalid,ret:%d,sync data:0x%x\n",
					__func__, ret, sync_data[0]);
			continue;
		}

		TPD_INFO("%s sync is valid:0x%x\n", __func__, sync_data[0]);

		ret = touch_i2c_read_block_u32(client, reg_addr, len, data);
		if(ret < 0) {
			TPD_INFO("sync read rawdata failed:%d\n", ret);
		} else {
			break;
		}
	}

	if(retry_times >= 200) {
		TPD_INFO("sync read rawdata timeout\n");
		ret = -1;
	} else {
		TPD_INFO("sync read rawdata,get rawdata success\n");
		sync_data[0] = 0x00;
		ret = touch_i2c_write_block_u32(client, sync_addr, 1, sync_data);
		TPD_INFO("sync read rawdata,clear sync\n");
		ret = 0;
	}

	TPD_INFO("%s exit:%d\n", __func__, ret);
	return ret;
}

/**
 * goodix_cache_rawdata - cache rawdata
 */
static int goodix_cache_rawdata(struct goodix_ts_test *ts_test)
{
	int j = 0;
	int i = 0;
	int ret = -EINVAL;
	u32 rawdata_size = 0;
	u32 rawdata_addr = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	TPD_INFO("Cache rawdata\n");
	ts_test->rawdata.size = 0;
	rawdata_size = ts_test->test_params.sen_num * ts_test->test_params.drv_num;

	if (rawdata_size > MAX_DRV_NUM * MAX_SEN_NUM || rawdata_size <= 0) {
		TPD_INFO("Invalid rawdata size(%u)\n", rawdata_size);
		return ret;
	}

	rawdata_addr = ts_test->test_params.rawdata_addr;
	TPD_INFO("Rawdata address=0x%x\n", rawdata_addr);

	for (j = 0; j < GOODIX_RETRY_NUM_3; j++) {
		/* read rawdata */
		ret = normandy_sync_read_rawdata(chip_info->client,
				chip_info->ic_info.misc.touch_data_addr, 0x80, rawdata_addr,
				(u8 *)&ts_test->rawdata.data[0], rawdata_size * sizeof(u16));
		if (ret < 0) {
			if (j == GOODIX_RETRY_NUM_3 - 1) {
				TPD_INFO("Failed to read rawdata:%d,at:%d\n", ret, j);
				goto cache_exit;
			} else {
				continue;
			}
		}
		for (i = 0; i < rawdata_size; i++)
			ts_test->rawdata.data[i] = le16_to_cpu(ts_test->rawdata.data[i]);
		ts_test->rawdata.size = rawdata_size;
		TPD_INFO("Rawdata ready\n");
		break;
	}

cache_exit:
	return ret;
}

/**
 * goodix_cache_selfrawdata - cache selfrawdata
 */
static int goodix_cache_self_rawdata(struct goodix_ts_test *ts_test)
{
	int i = 0, j = 0;
	int ret = -EINVAL;
	u16 self_rawdata_size = 0;
	u32 self_rawdata_addr = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	TPD_INFO("Cache selfrawdata\n");
	ts_test->self_rawdata.size = 0;
	self_rawdata_size = ts_test->test_params.sen_num + ts_test->test_params.drv_num;

	if (self_rawdata_size > MAX_DRV_NUM + MAX_SEN_NUM || self_rawdata_size <= 0) {
		TPD_INFO("Invalid selfrawdata size(%u)\n", self_rawdata_size);
		return ret;
	}

	self_rawdata_addr = ts_test->test_params.self_rawdata_addr;
	TPD_INFO("Selfraw address=0x%x\n", self_rawdata_addr);

	for (j = 0; j < GOODIX_RETRY_NUM_3; j++) {
		/* read selfrawdata */
		ret = normandy_sync_read_rawdata(chip_info->client,
				chip_info->ic_info.misc.touch_data_addr, 0x80,
				self_rawdata_addr, (u8 *)&ts_test->self_rawdata.data[0],
				self_rawdata_size * sizeof(u16));
		if (ret < 0) {
			if (j == GOODIX_RETRY_NUM_3 - 1) {
				TPD_INFO("Failed to read self_rawdata:%d\n", ret);
				goto cache_exit;
			} else {
				continue;
			}
		}
		for (i = 0; i < self_rawdata_size; i++)
			ts_test->self_rawdata.data[i] = le16_to_cpu(ts_test->self_rawdata.data[i]);
		ts_test->self_rawdata.size = self_rawdata_size;
		TPD_INFO("self_Rawdata ready\n");
		break;
	}

cache_exit:
	TPD_INFO("%s exit:%d\n", __func__, ret);
	return ret;
}

/**
 * goodix_noisetest_prepare- noisetest prepare
 */
static int goodix_noisetest_prepare(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	u32 noise_data_size = 0;
	u32 self_noise_data_size = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("%s run\n", __func__);
	noise_data_size = ts_test->test_params.sen_num * ts_test->test_params.drv_num;
	self_noise_data_size = ts_test->test_params.sen_num + ts_test->test_params.drv_num;

	if (noise_data_size <= 0 || noise_data_size > MAX_DRV_NUM * MAX_SEN_NUM) {
		TPD_INFO("%s: Bad noise_data_size[%d]\n", __func__, noise_data_size);
		ts_test->test_result[GTP_NOISE_TEST] = SYS_SOFTWARE_REASON;
		return -EINVAL;
	}

	if (self_noise_data_size <= 0 || self_noise_data_size > MAX_DRV_NUM + MAX_SEN_NUM) {
		TPD_INFO("%s: Bad self_noise_data_size[%d]\n", __func__, self_noise_data_size);
		ts_test->test_result[GTP_SELFNOISE_TEST] = SYS_SOFTWARE_REASON;
		return -EINVAL;
	}

	ts_test->noisedata.size = noise_data_size;
	ts_test->self_noisedata.size = self_noise_data_size;

	/* change to rawdata mode */
	ret = goodix_send_cmd_simple(chip_info, GTP_CMD_RAWDATA, 0);
	if (ret < 0) {
		TPD_INFO("%s: Failed send rawdata command:ret:%d\n", __func__, ret);
		ts_test->test_result[GTP_NOISE_TEST] = SYS_SOFTWARE_REASON;
		ts_test->test_result[GTP_SELFNOISE_TEST] = SYS_SOFTWARE_REASON;
		return ret;
	}
	msleep(50);

	TPD_INFO("%s: Enter rawdata mode\n", __func__);

	return ret;
}

/**
 * goodix_cache_noisedata- cache noisedata
 */
static int goodix_cache_noisedata(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	int ret1 = 0;
	u32 noisedata_addr = 0;
	u32 self_noisedata_addr = 0;
	u32 noise_data_size = 0;
	u32 self_noise_data_size = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	noisedata_addr = ts_test->test_params.noisedata_addr;
	self_noisedata_addr = ts_test->test_params.self_noisedata_addr;
	noise_data_size = ts_test->noisedata.size;
	self_noise_data_size = ts_test->self_noisedata.size;

	/* read noise data */
	ret = normandy_sync_read_rawdata(chip_info->client,
			chip_info->ic_info.misc.touch_data_addr, 0x80,
			noisedata_addr, (u8  *)&ts_test->noisedata.data[0],
			noise_data_size * sizeof(u16));
	if (ret < 0) {
		TPD_INFO("%s: Failed read noise data\n", __func__);
		ts_test->noisedata.size = 0;
		ts_test->test_result[GTP_NOISE_TEST] = SYS_SOFTWARE_REASON;
	}

	/* read self noise data */
	ret1 = normandy_sync_read_rawdata(chip_info->client,
			chip_info->ic_info.misc.touch_data_addr, 0x80,
			self_noisedata_addr, (u8 *)&ts_test->self_noisedata.data[0],
			self_noise_data_size * sizeof(u16));
	if (ret1 < 0) {
		TPD_INFO("%s: Failed read self noise data\n", __func__);
		ts_test->self_noisedata.size = 0;
		ts_test->test_result[GTP_SELFNOISE_TEST] = SYS_SOFTWARE_REASON;
	}

	TPD_INFO("%s exit:%d\n", __func__, ret | ret1);
	return ret | ret1;
}

/**
 * goodix_analyse_noisedata- analyse noisedata
 */
static void goodix_analyse_noisedata(struct goodix_ts_test *ts_test)
{
	int i = 0;
	u32 find_bad_node = 0;
	s16 noise_value = 0;

	for (i = 0; i < ts_test->noisedata.size; i++) {
		noise_value = (s16)le16_to_cpu(ts_test->noisedata.data[i]);
		ts_test->noisedata.data[i] = abs(noise_value);

		if (ts_test->noisedata.data[i] > ts_test->test_params.noise_threshold) {
			find_bad_node++;
			TPD_INFO("noise check failed: niose[%d][%d]:%u, > %u\n",
					(u32)div_s64(i, ts_test->test_params.sen_num),
					i % ts_test->test_params.sen_num,
					ts_test->noisedata.data[i],
					ts_test->test_params.noise_threshold);
		}
	}
	if (find_bad_node) {
		TPD_INFO("%s:noise test find bad node\n", __func__);
		ts_test->test_result[GTP_NOISE_TEST] = GTP_PANEL_REASON;
	} else {
		ts_test->test_result[GTP_NOISE_TEST] = GTP_TEST_PASS;
		TPD_INFO("noise test check pass\n");
	}

	return;
}

/**
 * goodix_analyse_self_noisedata- analyse self noisedata
 */
static void goodix_analyse_self_noisedata(struct goodix_ts_test *ts_test)
{
	int i = 0;
	u32 self_find_bad_node = 0;
	s16 self_noise_value = 0;
	TPD_INFO("%s run\n", __func__);
	for (i = 0; i < ts_test->self_noisedata.size; i++) {
		self_noise_value = (s16)be16_to_cpu(ts_test->self_noisedata.data[i]);
		ts_test->self_noisedata.data[i] = abs(self_noise_value);

		if (ts_test->self_noisedata.data[i] > ts_test->test_params.self_noise_threshold) {
			self_find_bad_node++;
			TPD_INFO("self noise check failed: self_noise[%d][%d]:%u, > %u\n",
					(u32)div_s64(i, ts_test->test_params.drv_num),
					i % ts_test->test_params.drv_num,
					ts_test->self_noisedata.data[i],
					ts_test->test_params.self_noise_threshold);
		}
	}

	if (self_find_bad_node) {
		TPD_INFO("%s:self_noise test find bad node\n", __func__);
		ts_test->test_result[GTP_SELFNOISE_TEST] = GTP_PANEL_REASON;
	} else {
		ts_test->test_result[GTP_SELFNOISE_TEST] = GTP_TEST_PASS;
		TPD_INFO("self noise test check passed\n");
	}
	TPD_INFO("%s exit\n", __func__);
	return;
}


/* test noise data */
static void goodix_test_noisedata(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	int test_cnt = 0;
	u8 buf[1];
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("%s run\n", __func__);
	ret = goodix_noisetest_prepare(ts_test);
	if (ret < 0) {
		TPD_INFO("%s :Noisetest prepare failed\n", __func__);
		goto soft_err_out;
	}

	/* read noisedata and self_noisedata,calculate result */
	for (test_cnt = 0; test_cnt < RAWDATA_TEST_TIMES; test_cnt++) {
		ret = goodix_cache_noisedata(ts_test);
		if (ret) {
			if (test_cnt == RAWDATA_TEST_TIMES - 1) {
				TPD_INFO("%s: Cache noisedata failed\n", __func__);
				goto soft_err_out;
			} else {
				continue;
			}
		}
		goodix_analyse_noisedata(ts_test);

		goodix_analyse_self_noisedata(ts_test);
		break;
	}

	TPD_INFO("%s: Noisedata and Self_noisedata test end\n", __func__);
	goto noise_test_exit;

soft_err_out:
	ts_test->noisedata.size = 0;
	ts_test->test_result[GTP_NOISE_TEST] = SYS_SOFTWARE_REASON;
	ts_test->self_noisedata.size = 0;
	ts_test->test_result[GTP_SELFNOISE_TEST] = SYS_SOFTWARE_REASON;
noise_test_exit:
	ret = goodix_send_cmd_simple(chip_info, GTP_CMD_NORMAL, 0);

	buf[0] = 0x00;
	ret = touch_i2c_write_block_u32(chip_info->client, chip_info->ic_info.misc.touch_data_addr, 1, buf);
	return;
}

static int goodix_self_rawcapacitance_test(struct ts_test_self_rawdata *rawdata,
		struct ts_test_params *test_params)
{
	int i = 0;
	int ret = NO_ERR;

	for (i = 0; i < rawdata->size; i++) {
		if (rawdata->data[i] > test_params->self_max_limits[i]) {
			TPD_INFO("self_rawdata[%d][%d]:%u >self_max_limit:%u, NG\n",
					(u32)div_s64(i, test_params->drv_num), i % test_params->drv_num,
					rawdata->data[i], test_params->self_max_limits[i]);
			ret = RESULT_ERR;
		}

		if (rawdata->data[i] < test_params->self_min_limits[i]) {
			TPD_INFO("self_rawdata[%d][%d]:%u < min_limit:%u, NG\n",
					(u32)div_s64(i, test_params->drv_num), i % test_params->drv_num,
					rawdata->data[i], test_params->self_min_limits[i]);
			ret = RESULT_ERR;
		}
	}

	return ret;
}

/* goodix_captest_prepare
 *
 * parse test peremeters from dt
 */
static int goodix_captest_prepare(struct goodix_ts_test *ts_test)
{
	int ret = -EINVAL;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	TPD_INFO("%s run\n", __func__);

	ret = goodix_send_config(chip_info, chip_info->test_cfg.data, chip_info->test_cfg.length);
	if (ret)
		TPD_INFO("Failed to send test config:%d\n", ret);
	else
		TPD_INFO("Success send test config\n");

	TPD_INFO("%s exit:%d\n", __func__, ret);
	return ret;
}

static int goodix_rawcapacitance_test(struct ts_test_rawdata *rawdata,
		struct ts_test_params *test_params)
{
	int i = 0;
	int ret = NO_ERR;

	for (i = 0; i < rawdata->size; i++) {
		if (rawdata->data[i] > test_params->max_limits[i]) {
			TPD_INFO("rawdata[%d][%d]:%u > max_limit:%u, NG\n",
					(u32)div_s64(i, test_params->sen_num), i % test_params->sen_num,
					rawdata->data[i], test_params->max_limits[i]);
			ret = RESULT_ERR;
		}

		if (rawdata->data[i] < test_params->min_limits[i]) {
			TPD_INFO("rawdata[%d][%d]:%u < min_limit:%u, NG\n",
					(u32)div_s64(i, test_params->sen_num), i % test_params->sen_num,
					rawdata->data[i], test_params->min_limits[i]);
			ret = RESULT_ERR;
		}
	}

	return ret;
}

static int goodix_deltacapacitance_test(struct ts_test_rawdata *rawdata,
		struct ts_test_params *test_params)
{
	int i = 0;
	int ret = NO_ERR;
	int cols = 0;
	u32 max_val = 0;
	u32 rawdata_val = 0;
	u32 sc_data_num = 0;
	u32 up = 0, down = 0, left = 0, right = 0;

	cols = test_params->sen_num;
	sc_data_num = test_params->drv_num * test_params->sen_num;
	if (cols <= 0) {
		TPD_INFO("%s: parmas invalid\n", __func__);
		return RESULT_ERR;
	}

	for (i = 0; i < sc_data_num; i++) {
		rawdata_val = rawdata->data[i];
		max_val = 0;
		/* calculate deltacpacitance with above node */
		if (i - cols >= 0) {
			up = rawdata->data[i - cols];
			up = abs(rawdata_val - up);
			if (up > max_val)
				max_val = up;
		}

		/* calculate deltacpacitance with bellow node */
		if (i + cols < sc_data_num) {
			down = rawdata->data[i + cols];
			down = abs(rawdata_val - down);
			if (down > max_val)
				max_val = down;
		}

		/* calculate deltacpacitance with left node */
		if (i % cols) {
			left = rawdata->data[i - 1];
			left = abs(rawdata_val - left);
			if (left > max_val)
				max_val = left;
		}

		/* calculate deltacpacitance with right node */
		if ((i + 1) % cols) {
			right = rawdata->data[i + 1];
			right = abs(rawdata_val - right);
			if (right > max_val)
				max_val = right;
		}

		/* float to integer */
		if (rawdata_val) {
			max_val *= FLOAT_AMPLIFIER;
			max_val = (u32)div_s64(max_val, rawdata_val);
			if (max_val > test_params->deviation_limits[i]) {
				TPD_INFO("deviation[%d][%d]:%u > delta_limit:%u, NG\n",
						(u32)div_s64(i, cols), i % cols, max_val,
						test_params->deviation_limits[i]);
				ret = RESULT_ERR;
			}
		} else {
			TPD_INFO("Find rawdata=0 when calculate deltacapacitance:[%d][%d]\n",
					(u32)div_s64(i, cols), i % cols);
			ret = RESULT_ERR;
		}
	}

	return ret;
}

/* goodix_rawdata_test
 * test rawdata with one frame
 */
static void goodix_rawdata_test(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	/* read rawdata and calculate result,  statistics fail times */
	ret = goodix_cache_rawdata(ts_test);
	if (ret < 0) {
		/* Failed read rawdata */
		TPD_INFO("Read rawdata failed\n");
		ts_test->test_result[GTP_CAP_TEST] = SYS_SOFTWARE_REASON;
		ts_test->test_result[GTP_DELTA_TEST] = SYS_SOFTWARE_REASON;
	} else {
		ret = goodix_rawcapacitance_test(&ts_test->rawdata, &ts_test->test_params);
		if (!ret) {
			ts_test->test_result[GTP_CAP_TEST] = GTP_TEST_PASS;
			TPD_INFO("Rawdata test pass\n");
		} else {
			ts_test->test_result[GTP_CAP_TEST] = GTP_PANEL_REASON;
			TPD_INFO("RawCap test failed\n");
		}

		ret = goodix_deltacapacitance_test(&ts_test->rawdata, &ts_test->test_params);
		if (!ret)  {
			ts_test->test_result[GTP_DELTA_TEST] = GTP_TEST_PASS;
			TPD_INFO("DeltaCap test pass\n");
		} else {
			ts_test->test_result[GTP_DELTA_TEST] = GTP_PANEL_REASON;
			TPD_INFO("DeltaCap test failed\n");
		}
	}
}

static void goodix_capacitance_test(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	u8 buf[1];
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("%s run\n", __func__);
	ret = goodix_captest_prepare(ts_test);
	if (ret) {
		TPD_INFO("Captest prepare failed\n");
		ts_test->test_result[GTP_CAP_TEST] = SYS_SOFTWARE_REASON;
		ts_test->test_result[GTP_DELTA_TEST] = SYS_SOFTWARE_REASON;
		ts_test->test_result[GTP_SELFCAP_TEST] = SYS_SOFTWARE_REASON;
		return;
	}
	/* read rawdata and calculate result,  statistics fail times */
	ret = goodix_send_cmd_simple(chip_info, GTP_CMD_RAWDATA, 0);
	if (ret < 0) {
		TPD_INFO("%s:Failed send rawdata cmd:ret%d\n", __func__, ret);
		goto capac_test_exit;
	} else {
		TPD_INFO("%s: Success change to rawdata mode\n", __func__);
	}
	msleep(50);

	goodix_rawdata_test(ts_test);

	/* read selfrawdata and calculate result,  statistics fail times */
	ret = goodix_cache_self_rawdata(ts_test);
	if (ret < 0) {
		/* Failed read selfrawdata */
		TPD_INFO("Read selfrawdata failed\n");
		ts_test->test_result[GTP_SELFCAP_TEST] = SYS_SOFTWARE_REASON;
		goto capac_test_exit;
	} else {
		ret = goodix_self_rawcapacitance_test(&ts_test->self_rawdata, &ts_test->test_params);
		if (!ret) {
			ts_test->test_result[GTP_SELFCAP_TEST] = GTP_TEST_PASS;
			TPD_INFO("selfrawdata test pass\n");
		} else {
			ts_test->test_result[GTP_SELFCAP_TEST] = GTP_PANEL_REASON;
			TPD_INFO("selfrawCap test failed\n");
		}
	}
capac_test_exit:
	goodix_send_cmd_simple(chip_info, GTP_CMD_NORMAL, 0);

	buf[0] = 0x00;
	ret = touch_i2c_write_block_u32(chip_info->client, chip_info->ic_info.misc.touch_data_addr, 1, buf);
	return;
}

static void goodix_intgpio_test(struct goodix_ts_test *p_ts_test)
{
	/*for interrupt pin test*/
	int eint_status = 0, eint_count = 0, read_gpio_num = 0;//not test int pin

	TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);

	while (read_gpio_num--) {
		msleep(5);
		eint_status = gpio_get_value(p_ts_test->p_testdata->irq_gpio);
		if (eint_status == 1) {
			eint_count--;
		} else {
			eint_count++;
		}
		TPD_INFO("%s eint_count = %d  eint_status = %d\n", __func__, eint_count, eint_status);
	}
	TPD_INFO("TP EINT PIN %s direct short! eint_count = %d\n", eint_count == 10 ? "is" : "not", eint_count);
	if (eint_count == 10) {
		TPD_INFO("error :  TP EINT PIN direct short!\n");
		seq_printf(p_ts_test->p_seq_file, "eint_status is low, TP EINT direct stort\n");
		eint_count = 0;
		p_ts_test->error_count ++;
		p_ts_test->test_result[GTP_INTPIN_TEST] = GTP_PANEL_REASON;
	}
	p_ts_test->test_result[GTP_INTPIN_TEST] = GTP_TEST_PASS;
}

/* Error code of AFE Inspection */
typedef enum {
	THP_AFE_INSPECT_OK = 0,            /* OK */
	THP_AFE_INSPECT_ESPI = (1 << 0),   // 0:0x0001, SPI communication error
	THP_AFE_INSPECT_ERAW = (1 << 1),   // 1:0x0002, Raw data error
	THP_AFE_INSPECT_ENOISE = (1 << 2), // 2:0x0004, Noise error
	THP_AFE_INSPECT_EOPEN = (1 << 3),  // 3:0x0008, Sensor open error
	THP_AFE_INSPECT_ESHORT = (1 << 4), // 4:0x0010, Sensor short error
	THP_AFE_INSPECT_ERC = (1 << 5),    // 5:0x0020, Sensor RC error
	THP_AFE_INSPECT_EPIN = (1 << 6),   // 6:0x0040,
	/* Errors of TSVD/TSHD/TRCST/TRCRQ and other PINs
	   when Report Rate Switching between 60 Hz and 120 Hz */
	THP_AFE_INSPECT_EOTHER = (1 << 7)  // 7:0x0080, All other errors
} THP_AFE_INSPECT_ERR_ENUM;


static u32 gt9897_short_parms[] = {10, 150, 150, 150, 100, 100, 30};

/* max pin num */
#define MAX_DRV_PIN_NUM	21
#define MAX_SEN_PIN_NUM	42
#define DRV_CHANNEL_FLAG	0x80

/**************************************************/

#define INSPECT_FW_SWITCH_CMD 			0x85
#define INSPECT_PARAM_CMD					0xAA

#define INSPECT_CMD_ACK_DONE    			0xEE

#define INSPECT_CMD_STATUS_RUNNING  		0x00
#define INSPECT_CMD_STATUS_DRV_DONE		0x08
#define INSPECT_CMD_STATUS_SEN_DONE 		0x80
#define INSPECT_CMD_STATUS_FINISH  		0x88

#define SHORT_FW_CMD_REG					0x10400
#define SHORT_TEST_TIME_REG				0x11FF2
#define GTX8_RETRY_NUM(x)					(x)
#define TEST_FW_PID  			"OST"  /* PID3-5 */
#define  MAX_TEST_TIME_MS            5000
#define DEFAULT_TEST_TIME_MS	 1000
#define MIN_TEST_TIME_MS            800



#define DFT_SHORT_THRESHOLD  			16
#define DFT_DIFFCODE_SHORT_THRESHOLD	16
#define DFT_ADC_DUMP_NUM				1396

#define DRV_TO_DRV_R(selfcode, V2)  (((selfcode)/(V2) - 1) * 63)
#define SEN_TO_SEN_R(selfcode, V2)  (((selfcode)/(V2) - 1) * 63)
#define SEN_TO_DRV_R(selfcode, V2)  (((selfcode)/(V2) - 1) * 63)



//#define DRV_TO_AVDD_R(diffcode, avdd) ((1.25 * 1024 * ((avdd) - 12.5) * 40) / ((diffcode) * 10) - 40)
#define DRV_TO_AVDD_R(diffcode, avdd)  ((51200*avdd)-(125*5120))/(((diffcode) * 10) - 40) 

//((1.25 * 1024 * ((avdd) - 12.5) * 40) / ((diffcode) * 10) - 40)

#define SEN_TO_AVDD_R(diffcode, avdd)  ((51200*avdd)-(125*5120))/(((diffcode) * 10) - 40)  
//((1.25 * 1024 * ((avdd) - 12.5) * 40) / ((diffcode) * 10) - 40)
//#define SEN_TO_AVDD_R(diffcode, avdd) ((1.25 * 1024 * ((avdd) - 12.5) * 40) / ((diffcode) * 10) - 40)
#define DRV_TO_GND_R(diffcode)	((64148 / (diffcode)) - 40)
#define SEN_TO_GND_R(diffcode)	((64148 / (diffcode)) - 40)

#define SHORT_CELL_SIZE(a)			(4 + (a) * 2 + 2)

static u8 map_die2pin(struct ts_test_params *test_params, u8 chn_num)
{
	int i = 0;
	u8 res = 255;

	for (i = 0; i < MAX_SEN_PIN_NUM; i++) {
		if (test_params->sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}

	/* res != 0xFF mean found the corresponding channel num */
	if (res != 0xFF)
		return res;

	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < MAX_DRV_PIN_NUM; i++) {
		if (test_params->drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (res >= MAX_DRV_PIN_NUM) {
		TPD_INFO("Faild found corrresponding channel num:%d\n", chn_num);
	} else {
		res |= DRV_CHANNEL_FLAG;
	}
	return res;
}


#define SHORT_TYPE_FLAG  ((uint16_t)1 << 15)

static int gdix_check_resistance_to_gnd(struct ts_test_params *test_params,
		u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;

	avdd_value = test_params->avdd_value;
	if (adc_signal == 0 || adc_signal == 0x8000)
		adc_signal |= 1;

	if ((adc_signal & 0x8000) == 0) {	/* short to GND */
		if (pos < MAX_DRV_NUM)
			r = DRV_TO_GND_R(adc_signal);
		else
			r = SEN_TO_GND_R(adc_signal);
	} else {	/* short to VDD */
		//adc_signal = adc_signal & ~0x8000;
		if (pos < MAX_DRV_NUM)
			r = DRV_TO_AVDD_R((adc_signal & ~0x8000), avdd_value);
		else
			r = SEN_TO_AVDD_R((adc_signal & ~0x8000), avdd_value);
	}

	if (pos < MAX_DRV_NUM)
		r_th = test_params->r_drv_gnd_threshold;
	else
		r_th = test_params->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < MAX_DRV_NUM)
		chn_id_tmp += MAX_SEN_NUM;
	else
		chn_id_tmp -= MAX_DRV_NUM;

	if (r < r_th) {
		pin_num = map_die2pin(test_params, chn_id_tmp);
		TPD_INFO("%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK\n",
				(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
				(pin_num & ~DRV_CHANNEL_FLAG),
				(adc_signal & 0x8000) ? "VDD" : "GND",
				r, r_th);

		return THP_AFE_INSPECT_ESHORT;
	}
	return THP_AFE_INSPECT_OK;
}



#define DRV_DRV_SELFCODE_REG  0x1045E
static int gdix_check_tx_tx_shortcircut(struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	uint32_t data_reg;
	//struct short_record temp_short_info;
	struct ts_test_params *test_params = &ts_test->test_params;
	u16 self_capdata, short_die_num = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *) ts_test->ts;


	size = SHORT_CELL_SIZE(MAX_DRV_NUM);	/* 4 + MAX_DRV_NUM * 2 + 2; */
	data_buf =  kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		TPD_INFO("Failed to alloc memory\n");
		return THP_AFE_INSPECT_EOTHER;
	}

	/* drv&drv shortcircut check */
	data_reg = DRV_DRV_SELFCODE_REG;
	for (i = 0; i < short_ch_num; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client,data_reg, size, data_buf);
		if (ret) {
			TPD_INFO("Failed read Drv-to-Drv short rawdata\n");
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("Drv-to-Drv adc data checksum error\n");
			//NLOGI_ARRAY(data_buf, size);
			TPD_INFO("%s: data_buf:%*ph " ,__func__,40,data_buf);
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		r_threshold = test_params->r_drv_drv_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);     //info->length = le16_to_cpup((__le16 *)data);
		short_die_num -= MAX_SEN_NUM;
		if (short_die_num >= MAX_DRV_NUM) {
			TPD_INFO("invalid short pad num:%d\n",
					short_die_num + MAX_SEN_NUM);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < MAX_DRV_NUM; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);
			if (adc_signal < test_params->short_threshold)
				continue;
			short_r = DRV_TO_DRV_R(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num + MAX_SEN_NUM);
				slave_pin_num = map_die2pin(test_params, j + MAX_SEN_NUM);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("WARNNING invalid pin");
					continue;
				}
				TPD_INFO("short circut:R=%dK,R_Threshold=%dK\n",
						short_r, r_threshold);
				TPD_INFO("%s%d--%s%d shortcircut\n",
						(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(master_pin_num & ~DRV_CHANNEL_FLAG),
						(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err |= THP_AFE_INSPECT_ESHORT;
			}
		}
		data_reg += size;
	}

	if (data_buf != NULL) {
		kfree(data_buf);
		data_buf = NULL;
	}
	return err;
}

#define SEN_SEN_SELFCODE_REG 0x1084E
static int gdix_check_rx_rx_shortcircut(struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	uint32_t data_reg;
	//	struct short_record temp_short_info;
	struct ts_test_params *test_params = &ts_test->test_params;
	u16 self_capdata, short_die_num = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *) ts_test->ts;


	size = SHORT_CELL_SIZE(MAX_SEN_NUM);	/* 4 + MAX_SEN_NUM * 2 + 2; */
	data_buf =  kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		TPD_INFO("Failed to alloc memory\n");
		return THP_AFE_INSPECT_EOTHER;
	}

	/* drv&drv shortcircut check */
	data_reg = SEN_SEN_SELFCODE_REG;
	for (i = 0; i < short_ch_num; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client ,data_reg, size, data_buf);
		if (ret) {
			TPD_INFO("Failed read Sen-to-Sen short rawdata\n");
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("Sen-to-Sen adc data checksum error\n");
			//NLOGI_ARRAY(data_buf, size);
			TPD_INFO("%s: data_buf:%*ph " ,__func__,size,data_buf);
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		r_threshold = test_params->r_sen_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= MAX_SEN_NUM) {
			TPD_INFO("invalid short pad num:%d\n",	short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < MAX_SEN_NUM; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);
			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = SEN_TO_SEN_R(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("WARNNING invalid pin");
					continue;
				}
				TPD_INFO("short circut:R=%dK,R_Threshold=%dK\n",
						short_r, r_threshold);
				TPD_INFO("%s%d--%s%d shortcircut\n",
						(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(master_pin_num & ~DRV_CHANNEL_FLAG),
						(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err |= THP_AFE_INSPECT_ESHORT;
			}
		}
		data_reg += size;
	}

	if (data_buf != NULL) {
		kfree(data_buf);
		data_buf = NULL;
	}
	return err;
}

#define DRV_SEN_SELFCODE_REG 0x11712
static int gdix_check_tx_rx_shortcircut(struct goodix_ts_test *ts_test, u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	uint32_t data_reg;
	//	struct short_record temp_short_info;
	struct ts_test_params *test_params = &ts_test->test_params;
	u16 self_capdata, short_die_num = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *) ts_test->ts;

	size = SHORT_CELL_SIZE(MAX_DRV_NUM);	/* 4 + MAX_SEN_NUM * 2 + 2; */
	data_buf =  kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		TPD_INFO("Failed to alloc memory\n");
		return THP_AFE_INSPECT_EOTHER;
	}

	/* drv&sen shortcircut check */
	data_reg = DRV_SEN_SELFCODE_REG;
	for (i = 0; i < short_ch_num; i++) {
		ret = touch_i2c_read_block_u32(chip_info->client   ,data_reg, size, data_buf);
		if (ret) {
			TPD_INFO("Failed read Drv-to-Sen short rawdata\n");
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("Drv-to-Sen adc data checksum error\n");
			//NLOGI_ARRAY(data_buf, size);
			TPD_INFO("%s: data_buf:%*ph " ,__func__,size,data_buf);
			err |= THP_AFE_INSPECT_ESPI;
			break;
		}

		r_threshold = test_params->r_drv_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= MAX_SEN_NUM) {
			TPD_INFO("invalid short pad num:%d\n",	short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = 0; j < MAX_DRV_NUM; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = SEN_TO_DRV_R(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j + MAX_SEN_NUM);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("WARNNING invalid pin");
					continue;
				}
				TPD_INFO("short circut:R=%dK,R_Threshold=%dK\n",
						short_r, r_threshold);
				TPD_INFO("%s%d--%s%d shortcircut\n",
						(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(master_pin_num & ~DRV_CHANNEL_FLAG),
						(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
						(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err |= THP_AFE_INSPECT_ESHORT;
			}
		}
		data_reg += size;
	}

	if (data_buf != NULL) {
		kfree(data_buf);
		data_buf = NULL;
	}
	return err;
}


#define DIFF_CODE_DATA_REG 0x11F72
static int gdix_check_gndvdd_shortcircut(struct goodix_ts_test *ts_test)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u8 *data_buf = NULL;
	struct chip_data_brl *chip_info = (struct chip_data_brl *) ts_test->ts;


	size = (MAX_DRV_NUM + MAX_SEN_NUM) * 2 + 2;
	data_buf =  kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		TPD_INFO("Failed to alloc memory\n");
		return THP_AFE_INSPECT_EOTHER;
	}
	/* read diff code, diff code will be used to calculate
	 * resistance between channel and GND */
	ret = touch_i2c_read_block_u32(chip_info->client ,DIFF_CODE_DATA_REG, size, data_buf); /* DIFF_CODE_REG   0xA97A */
	if (ret < 0) {
		TPD_INFO("Failed read to-gnd rawdata\n");
		err |= THP_AFE_INSPECT_ESPI;
		goto err_out;
	}

	if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("diff code checksum error\n");
		//NLOGI_ARRAY(data_buf, size);
		TPD_INFO("%s: data_buf:%*ph " ,__func__,(MAX_DRV_NUM + MAX_SEN_NUM) * 2 + 2,data_buf);
		err |= THP_AFE_INSPECT_ESPI;
		goto err_out;
	}

	for (i = 0; i < MAX_DRV_NUM + MAX_SEN_NUM; i++) {
		adc_signal = le16_to_cpup((__le16 *)&data_buf[i * 2]);
		ret = gdix_check_resistance_to_gnd(&ts_test->test_params,
				adc_signal, i);
		if (ret != THP_AFE_INSPECT_OK) {
			TPD_INFO("Resistance to-gnd/vdd short\n");
			err |= ret;
		}
	}

err_out:
	if (data_buf != NULL) {
		kfree(data_buf);
		data_buf = NULL;
	}
	return err;
}

/***************************************/
typedef struct __attribute__((packed)) {
	uint8_t result;
	uint8_t drv_drv_num;
	uint8_t sen_sen_num;
	uint8_t drv_sen_num;
	uint8_t drv_gnd_avdd_num;
	uint8_t sen_gnd_avdd_num;
	uint16_t checksum;
} test_result_t;

/***************************************/
#define TEST_RESULT_REG 		0x10410
#define TEST_CHANNEL_MAP_REG 	0x10410


static int gdix_shortcircut_analysis(struct goodix_ts_test *ts_test)
{
	int ret = 0, err = 0;
	test_result_t test_result = {0};
	struct chip_data_brl *chip_info = (struct chip_data_brl *) ts_test->ts;


	TPD_INFO(">>>>>start analysis  test result"); 
	ret = touch_i2c_read_block_u32(chip_info->client,TEST_RESULT_REG, sizeof(test_result),(u8 *)&test_result);
	if (ret < 0) {
		TPD_INFO("Read TEST_RESULT_REG falied\n");
		return THP_AFE_INSPECT_ESPI;
	}

	if (checksum_cmp((uint8_t*)&test_result, sizeof(test_result), CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("shrot result checksum err\n");
		//NLOGI_ARRAY((uint8_t*)&test_result, sizeof(test_result));
		TPD_INFO("%s: test_result:%*ph " ,__func__,8,(uint8_t*)&test_result);
		return THP_AFE_INSPECT_EOTHER;
	}

	if (!(test_result.result & 0x0F)) {
		TPD_INFO(">>>>>No shortcircut\n");
		return THP_AFE_INSPECT_OK;
	}

	TPD_INFO("short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d\n", test_result.result, test_result.drv_drv_num, test_result.sen_sen_num,
			test_result.drv_sen_num, test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);

	/* get channel map */
	// ret = thp_spi_read(TEST_CHANNEL_MAP_REG, (uint8_t *)&(ts_test->test_params.channel_map),
	// 	sizeof(ts_test->test_params.channel_map));
	// if (ret) {
	// 	NLOGE("failed get channel map info");
	// 	return THP_AFE_INSPECT_ESPI;
	// }
	// if (checksum_cmp((uint8_t *)&(ts_test->test_params.channel_map), sizeof(ts_test->test_params.channel_map), CHECKSUM_MODE_U8)) {
	// 	NLOGE("channel map checksum error");
	// 	NLOGI_ARRAY((uint8_t *)&(ts_test->test_params.channel_map), sizeof(ts_test->test_params.channel_map));
	// 	return THP_AFE_INSPECT_EOTHER;
	// }

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(ts_test, test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(ts_test, test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(ts_test, test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut(ts_test);

	TPD_INFO(">>>>>short check return 0x%x", err);
	return err;
}



static int goodix_do_short_test(struct goodix_ts_test *ts_test)
{
	int ret;
	int retry;
	uint16_t test_time = 0;
	//fw_info_t *fw_info = ts_test->fw_info;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	struct goodix_ts_cmd  test_parm_cmd ;
	struct goodix_ts_cmd  tmp_cmd ;


	usleep_range(2000, 2100);
	ret = touch_i2c_read_block_u32(chip_info->client,SHORT_TEST_TIME_REG,2, (uint8_t *)&test_time);
	if (ret) {
		TPD_INFO("failed get short test time info, default %dms", DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		test_time = le16_to_cpu(test_time);
		test_time /= 10; /* convert to ms */
		if (test_time > MAX_TEST_TIME_MS) {
			TPD_INFO("test time too long %d > %d",test_time, MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		TPD_INFO("get test time %dms", test_time);
	}

	test_parm_cmd.len = 0x0A;
	test_parm_cmd.cmd = 	    INSPECT_PARAM_CMD;
	test_parm_cmd.data[0] = DFT_SHORT_THRESHOLD & 0xFF;
	test_parm_cmd.data[1] = (DFT_SHORT_THRESHOLD >> 8) & 0xFF;

	test_parm_cmd.data[2] = DFT_DIFFCODE_SHORT_THRESHOLD & 0xFF;
	test_parm_cmd.data[3] = (DFT_DIFFCODE_SHORT_THRESHOLD >> 8) & 0xFF;

	test_parm_cmd.data[4] = DFT_ADC_DUMP_NUM & 0xFF;
	test_parm_cmd.data[5] = (DFT_ADC_DUMP_NUM >> 8) & 0xFF;
	goodix_append_checksum(&(test_parm_cmd.buf[2]), 8, CHECKSUM_MODE_U8_LE);
	TPD_INFO("%s: test_parm_cmd.buf:%*ph " ,__func__,12,test_parm_cmd.buf);

	ret = touch_i2c_write_block_u32(chip_info->client,SHORT_FW_CMD_REG,12,test_parm_cmd.buf);
	if (ret) {
		TPD_INFO("failed send short test param %d", ret);
		return ret;
	}
	retry = GTX8_RETRY_NUM(10);
	do {
		ret = touch_i2c_read_block_u32(chip_info->client ,SHORT_FW_CMD_REG, sizeof(tmp_cmd.buf), tmp_cmd.buf);
		if (!ret && tmp_cmd.ack == INSPECT_CMD_ACK_DONE)
			break;
		TPD_INFO("ack not ready 0x%x", tmp_cmd.ack);
		//NLOGI_ARRAY(tmp_cmd.buf, sizeof(tmp_cmd.buf));
		//TPD_INFO("%s: tmp_cmd.buf:%*ph " ,__func__,sizeof(tmp_cmd.buf),test_parm_cmd.buf);
		usleep_range(1000,1100);
	} while (--retry);

	if (tmp_cmd.ack != INSPECT_CMD_ACK_DONE) {
		TPD_INFO("failed get ack ready flag ack 0x%x != 0x%x",
				tmp_cmd.ack, INSPECT_CMD_ACK_DONE);
		return -EINVAL;
	}
	TPD_INFO(">>>>>success send short test params");
	/* check test result */
	msleep(test_time);
	retry = GTX8_RETRY_NUM(50);
	do {
		ret = touch_i2c_read_block_u32(chip_info->client ,SHORT_FW_CMD_REG, sizeof(tmp_cmd.buf), tmp_cmd.buf);
		if (!ret && tmp_cmd.state== INSPECT_CMD_STATUS_FINISH)
			break;
		TPD_INFO("short test ack 0x%x status 0x%x", tmp_cmd.ack, tmp_cmd.state);
		usleep_range(50000,51000);
	} while (--retry);

	if (tmp_cmd.state== INSPECT_CMD_STATUS_FINISH) {
		TPD_INFO(">>>>>test finished");
		return 0;
	}
	TPD_INFO(">>>>>test status error");
	return -EINVAL;
}


/**************************************************/
/* short test */
static int gdix_short_test_prepare(struct goodix_ts_test *ts_test)
{
	int ret;
	int retry;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	struct goodix_fw_version fw_ver;
	struct goodix_ts_cmd fw_switch_cmd ;

	fw_switch_cmd.len = 0x04;
	fw_switch_cmd.cmd = INSPECT_FW_SWITCH_CMD;
	//goodix_append_checksum(&(fw_switch_cmd.buf[2]), 2, CHECKSUM_MODE_U8_LE);
	//TPD_INFO("%s: fw_switch_cmd.buf:%*ph " ,__func__,6,fw_switch_cmd.buf);
	//TPD_INFO("%s: cmd_addr:0x%4x\n " ,__func__,chip_info->ic_info.misc.cmd_addr);

	retry = GTX8_RETRY_NUM(5);
	do {
		/*
		   ret = touch_i2c_write_block_u32(chip_info->client,chip_info->ic_info.misc.cmd_addr,6,fw_switch_cmd.buf);
		   if (ret) {
		   TPD_INFO("failed send short FW switch cmd %d", ret);
		   return ret;
		   }
		 */
		if ( brl_send_cmd(chip_info, &fw_switch_cmd) != 0)
			TPD_INFO("send short test cmd fail---by benson");
		usleep_range(30000,31000);
		msleep(100);
		/*
		   static int brl_read_version(struct chip_data_brl *chip_info,
		   struct goodix_fw_version *version)
		 */
		ret = brl_read_version(chip_info,&fw_ver);
		if (ret) {
			TPD_INFO("failed get fw version, retry %d", retry);
		} else {
			if (memcmp(&(fw_ver.patch_pid[3]), TEST_FW_PID, strlen(TEST_FW_PID))) {
				TPD_INFO("patch ID dismatch %s != %s", fw_ver.patch_pid,
						TEST_FW_PID);
				ret = -EINVAL;
			}
		}
	} while (ret && --retry);

	if (ret) {
		TPD_INFO("faild switch to short test firmware, %d", ret);
		return ret;
	}

	TPD_INFO(">>>>>success switch to short firmware");
	return goodix_do_short_test(ts_test);
}



/* gdix_test_params_init
 *
 * preparation before tp test
 */
static int gdix_test_params_init(struct goodix_ts_test *ts_test)
{

	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;
	struct ts_test_params *test_params = &ts_test->test_params;

	test_params->max_drv_num = MAX_DRV_NUM;
	test_params->max_sen_num = MAX_SEN_NUM;
	test_params->sen_num =chip_info->ic_info.parm.sen_num;	//ts_test->fw_info->sen_num;
	test_params->drv_num = chip_info->ic_info.parm.drv_num;	//ts_test->fw_info->drv_num;

	test_params->drv_map = gt9897_drv_map;
	test_params->sen_map = gt9897_sen_map;

	test_params->short_threshold = gt9897_short_parms[0];
	test_params->r_drv_drv_threshold = gt9897_short_parms[1];
	test_params->r_drv_sen_threshold = gt9897_short_parms[2];
	test_params->r_sen_sen_threshold = gt9897_short_parms[3];
	test_params->r_drv_gnd_threshold = gt9897_short_parms[4];
	test_params->r_sen_gnd_threshold = gt9897_short_parms[5];
	test_params->avdd_value = gt9897_short_parms[6];

	TPD_INFO("short test sen_num %d, drv_num %d\n", ts_test->test_params.sen_num, ts_test->test_params.drv_num);
	//if (ts_test->test_params.sen_num > ts_test->test_params.max_sen_num ||
	//    ts_test->test_params.drv_num > ts_test->test_params.max_drv_num) {
	if (ts_test->test_params.sen_num > MAX_SEN_PIN_NUM ||
			ts_test->test_params.drv_num > MAX_DRV_PIN_NUM) {
		TPD_INFO("invalied sensor and driver number\n");
		return -1;
	}
	return 0;
}



/****************add by benson**********************************
  int gdix_shortcircut_test(fw_info_t *fw_info)


 **************************************************************/
unsigned long long debug_ret = 0;

static int goodix_shortcircut_test(struct goodix_ts_test *ts_test)
{
	int ret = 0 ;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts_test->ts;

	TPD_INFO("===========short test in==============\n");


	ts_test->test_result[GTP_SHORT_TEST] = GTP_TEST_PASS;
	// ts_test.fw_info = fw_info;

	ret = gdix_test_params_init(ts_test);
	if (ret < 0) {
		TPD_INFO("Failed init test parameters\n");
		ts_test->test_result[GTP_SHORT_TEST] = SYS_SOFTWARE_REASON;
		debug_ret |= 0xFF00000000000000;
		return THP_AFE_INSPECT_EOTHER;
	}

	ret = gdix_short_test_prepare(ts_test);
	if (ret != THP_AFE_INSPECT_OK) {
		TPD_INFO("Failed do short test\n");
		ts_test->test_result[GTP_SHORT_TEST] = SYS_SOFTWARE_REASON;
		goto finish_test;
	}

	ret = gdix_shortcircut_analysis(ts_test);
	if (ret) {
		/*TODO debug return value 0F for Rest Pin test */
		ts_test->test_result[GTP_SHORT_TEST] = GTP_PANEL_REASON;
		debug_ret |= 0x2F0000;

	}
finish_test:
	/*reset IC*/
	TPD_INFO("short test finish, ret 0x%x\n", ret);
	goodix_reset(chip_info);
	return ret;

}


/*************************************************************/


static void goodix_put_test_result(struct goodix_ts_test *ts_test)
{
	uint8_t  data_buf[64];
	struct goodix_testdata *p_testdata;
	int i;
	/*save test fail result*/
	struct timespec now_time;
	struct rtc_time rtc_now_time;
	mm_segment_t old_fs;
	uint8_t file_data_buf[128];
	struct file *filp;

	p_testdata = ts_test->p_testdata;

	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		/* if have tested, show result */
		if (ts_test->test_result[i]) {
			if (GTP_TEST_PASS == ts_test->test_result[i]) {
			} else if(GTP_PANEL_REASON == ts_test->test_result[i]) {
				ts_test->error_count ++;
			} else if(SYS_SOFTWARE_REASON == ts_test->test_result[i]) {
				ts_test->error_count ++;
			}
		}
		TPD_INFO("test_result_info %s[%d]%d\n", test_item_name[i], i, ts_test->test_result[i]);
	}
	//step2: create a file to store test data in /sdcard/Tp_Test
	getnstimeofday(&now_time);
	rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
	//if test fail,save result to path:/sdcard/TpTestReport/screenOn/NG/
	if(ts_test->error_count) {
		snprintf(file_data_buf, 128, "/sdcard/TpTestReport/screenOn/NG/tp_testlimit_%02d%02d%02d-%02d%02d%02d-fail-utc.csv",
				(rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
				rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	} else {
		snprintf(file_data_buf, 128, "/sdcard/TpTestReport/screenOn/OK/tp_testlimit_%02d%02d%02d-%02d%02d%02d-pass-utc.csv",
				(rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
				rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	ksys_mkdir("/sdcard/TpTestReport", 0666);
	ksys_mkdir("/sdcard/TpTestReport/screenOn", 0666);
	ksys_mkdir("/sdcard/TpTestReport/screenOn/NG", 0666);
	ksys_mkdir("/sdcard/TpTestReport/screenOn/OK", 0666);
	filp = filp_open(file_data_buf, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(filp)) {
		TPD_INFO("Open log file '%s' failed. %ld\n", file_data_buf, PTR_ERR(filp));
		set_fs(old_fs);
		/*add for *#899# test for it can not acess sdcard*/
	}
	p_testdata->fp = filp;
	p_testdata->pos = 0;

	memset(data_buf, 0, sizeof(data_buf));
	if (ts_test->rawdata.size) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "%s\n", "[RAW DATA]");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
		for (i = 0; i < ts_test->rawdata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				snprintf(data_buf, 64, "%d,", ts_test->rawdata.data[i]);
				vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
				if (!((i + 1) % p_testdata->RX_NUM) && (i != 0)) {
					snprintf(data_buf, 64, "\n");
					vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
				}
			}
		}
	}
	if (ts_test->noisedata.size) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[NOISE DATA]");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
		for (i = 0; i < ts_test->noisedata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->noisedata.data[i]);
				vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
				if (!((i + 1) % p_testdata->RX_NUM) && (i != 0)) {
					snprintf(data_buf, 64, "\n");
					vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
				}
			}
		}
	}

	if (ts_test->self_noisedata.size) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[SELF NOISE DATA]");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
		for (i = 0; i < ts_test->self_noisedata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->self_noisedata.data[i]);
				vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
			}
		}
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			sprintf(data_buf, "\n");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
	}
	if (ts_test->self_rawdata.size) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[SELF RAW DATA]");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
		for (i = 0; i < ts_test->self_rawdata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->self_rawdata.data[i]);
				vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
			}
		}
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			sprintf(data_buf, "\n");
			vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
		}
	}

	if (!IS_ERR_OR_NULL(p_testdata->fp)) {
		snprintf(data_buf, 64, "TX:%d,RX:%d\n", p_testdata->TX_NUM, p_testdata->RX_NUM);
		vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
	}

	if (!IS_ERR_OR_NULL(p_testdata->fp)) {
		snprintf(data_buf, 64, "Img version:%lld,device version:%lld\n", p_testdata->TP_FW, ts_test->device_tp_fw);
		vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
	}

	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		/* if have tested, show result */
		if (ts_test->test_result[i]) {
			if (GTP_TEST_PASS == ts_test->test_result[i]) {
				if (!IS_ERR_OR_NULL(p_testdata->fp)) {
					snprintf(data_buf, 64, "\n%s %d:%s\n", test_item_name[i], i, "pass");
					vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
				}
			} else if(GTP_PANEL_REASON == ts_test->test_result[i]) {
				if (!IS_ERR_OR_NULL(p_testdata->fp)) {
					snprintf(data_buf, 64, "\n%s %d:%s\n", test_item_name[i], i, "NG:PANEL REASON");
					vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
					seq_printf(ts_test->p_seq_file, "%s", data_buf);
				}
			} else if(SYS_SOFTWARE_REASON == ts_test->test_result[i]) {
				if (!IS_ERR_OR_NULL(p_testdata->fp)) {
					snprintf(data_buf, 64, "\n%s %d:%s\n", test_item_name[i], i, "NG:SOFTWARE_REASON");
					vfs_write(p_testdata->fp, data_buf, strlen(data_buf), &p_testdata->pos);
					seq_printf(ts_test->p_seq_file, "%s", data_buf);
				}
			}
		}
	}

	if (!IS_ERR_OR_NULL(p_testdata->fp)) {
		filp_close(p_testdata->fp, NULL);
		set_fs(old_fs);
	}
	TPD_INFO("%s exit\n", __func__);
	return;
}

static void goodix_auto_test(struct seq_file *s, void *chip_data, struct goodix_testdata *p_testdata)
{
	int ret = 0;
	struct goodix_ts_test *gts_test = NULL;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version *ver_info = &chip_info->ver_info;
	u16 *p_noisedata = NULL;
	u16 *p_rawdata = NULL;
	u16 *p_self_rawdata = NULL;
	u16 *p_self_noisedata = NULL;
	u32 fw_ver = 0;
	u8 cfg_ver = 0;

	TPD_INFO("%s: enter\n", __func__);

	if (!chip_data) {
		TPD_INFO("%s: chip_data is NULL\n", __func__);
		return;
	}
	if (!p_testdata) {
		TPD_INFO("%s: goodix_testdata is null\n", __func__);
		return;
	}

	gts_test = kzalloc(sizeof(struct goodix_ts_test), GFP_KERNEL);
	if (!gts_test) {
		TPD_INFO("%s: goodix_testdata is null\n", __func__);
		seq_printf(s, "1 error(s).\n");
		return;
	}

	p_rawdata = kzalloc(sizeof(u16) * (p_testdata->TX_NUM * p_testdata->RX_NUM), GFP_KERNEL);
	if (!p_rawdata) {
		TPD_INFO("Failed to alloc rawdata info memory\n");
		gts_test->error_count ++;
		ret = -1;
		goto exit_finish;
	}
	p_noisedata = kzalloc(sizeof(u16) * (p_testdata->TX_NUM * p_testdata->RX_NUM), GFP_KERNEL);
	if (!p_noisedata) {
		TPD_INFO("Failed to alloc rawdata info memory\n");
		gts_test->error_count ++;
		ret = -1;
		goto exit_finish;
	}

	p_self_rawdata = kzalloc(sizeof(u16) * (p_testdata->TX_NUM + p_testdata->RX_NUM), GFP_KERNEL);
	if (!p_self_rawdata) {
		TPD_INFO("Failed to alloc rawdata info memory\n");
		gts_test->error_count ++;
		ret = -1;
		goto exit_finish;
	}
	p_self_noisedata  = kzalloc(sizeof(u16) * (p_testdata->TX_NUM + p_testdata->RX_NUM), GFP_KERNEL);
	if (!p_self_noisedata) {
		TPD_INFO("Failed to alloc rawdata info memory\n");
		gts_test->error_count ++;
		ret = -1;
		goto exit_finish;
	}
	TPD_INFO("%s: alloc mem:%ld\n", __func__, sizeof(struct goodix_ts_test) + 4 * (sizeof(u16) * (p_testdata->TX_NUM * p_testdata->RX_NUM)));

	gts_test->p_seq_file = s;
	gts_test->ts = chip_info;
	gts_test->p_testdata = p_testdata;

	gts_test->rawdata.data = p_rawdata;
	gts_test->noisedata.data = p_noisedata;
	gts_test->self_rawdata.data = p_self_rawdata;
	gts_test->self_noisedata.data = p_self_noisedata;

	goodix_intgpio_test(gts_test);
	ret = goodix_tptest_prepare(gts_test);
	if (ret) {
		TPD_INFO("%s: Failed parse test peremeters, exit test\n", __func__);
		gts_test->error_count ++;
		goto exit_finish;
	}
	TPD_INFO("%s: TP test prepare OK\n", __func__);

	goodix_print_test_params(gts_test);

	goodix_test_noisedata(gts_test); /*3F 7F test*/
	goodix_capacitance_test(gts_test); /* 1F 2F 6F test*/

	goodix_shortcircut_test(gts_test); 		/* 5F test */
	goodix_print_testdata(gts_test);
	goodix_tptest_finish(gts_test);

	/*read device fw version*/
	fw_ver = be32_to_cpup((__be32 *)&ver_info->patch_vid[0]);
	cfg_ver = chip_info->ic_info.version.config_version;
	gts_test->device_tp_fw = cfg_ver | (fw_ver & 0xFF) << 8;
	goodix_put_test_result(gts_test);

exit_finish:
	seq_printf(s, "imageid = %lld, deviceid = %lld\n", p_testdata->TP_FW, gts_test->device_tp_fw);
	TPD_INFO("imageid= %lld, deviceid= %lld\n", p_testdata->TP_FW, gts_test->device_tp_fw);
	seq_printf(s, "%d error(s). %s\n", gts_test->error_count, gts_test->error_count ? "" : "All test passed.");
	TPD_INFO(" TP auto test %d error(s). %s\n", gts_test->error_count, gts_test->error_count ? "" : "All test passed.");
	TPD_INFO("%s exit:%d\n", __func__, ret);
	if (p_rawdata) {
		kfree(p_rawdata);
		p_rawdata = NULL;
	}
	if (p_noisedata) {
		kfree(p_noisedata);
		p_noisedata = NULL;
	}
	if (p_self_rawdata) {
		kfree(p_self_rawdata);
		p_self_rawdata = NULL;
	}
	if (p_self_noisedata) {
		kfree(p_self_noisedata);
		p_self_noisedata = NULL;
	}
	if (gts_test) {
		kfree(gts_test);
		gts_test = NULL;
	}
	return;
}
/*************** End of atuo test func***************************/

static int goodix_set_health_info_state (void *chip_data, uint8_t enable)
{
	int ret = 0;

	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;


	TPD_INFO("%s, enable : %d\n", __func__, enable);
	if (enable) {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_DEBUG, 0x55);
		ret |= goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_DOWN_DELTA, 0x55);
		TPD_INFO("%s: enable debug log %s\n", __func__, ret < 0 ? "failed" : "success");
	} else {
		ret = goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_DEBUG, 0x00);
		ret |= goodix_send_cmd_simple_onedata(chip_info, GTP_CMD_DOWN_DELTA, 0x55);
		TPD_INFO("%s: disable debug log %s\n", __func__, ret < 0 ? "failed" : "success");
	}

	return ret;
}

static int goodix_get_health_info_state (void *chip_data)
{
	TPD_INFO("%s enter\n", __func__);
	return 0;
}

struct goodix_proc_operations goodix_brl_proc_ops = {
	.goodix_config_info_read    = goodix_config_info_read,
	.auto_test                  = goodix_auto_test,
	.set_health_info_state      = goodix_set_health_info_state,
	.get_health_info_state      = goodix_get_health_info_state,
};

/*********** Start of I2C Driver and Implementation of it's callbacks*************************/
static int goodix_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct chip_data_brl *chip_info = NULL;
	struct touchpanel_data *ts = NULL;
	int ret = -1;

	TPD_INFO("%s is called\n", __func__);

	if (tp_register_times > 0) {
		TPD_INFO("TP driver have success loaded %d times, exit\n", tp_register_times);
		return -1;
	}

	/* 1. Alloc chip_info */
	chip_info = kzalloc(sizeof(struct chip_data_brl), GFP_KERNEL);
	if (chip_info == NULL) {
		TPD_INFO("chip info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}

	/* 2. Alloc common ts */
	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("ts kzalloc error\n");
		goto ts_malloc_failed;
	}

	/* 3. alloc touch data space */
	chip_info->touch_data = kzalloc(MAX_GT_IRQ_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->touch_data == NULL) {
		TPD_INFO("touch_data kzalloc error\n");
		goto err_register_driver;
	}
	chip_info->edge_data = kzalloc(MAX_GT_EDGE_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->edge_data == NULL) {
		TPD_INFO("edge_data kzalloc error\n");
		goto err_touch_data_alloc;
	}

	/* 4. bind client and dev for easy operate */
	chip_info->client = client;
	chip_info->goodix_ops = &goodix_brl_proc_ops;
	ts->debug_info_ops = &debug_info_proc_ops;
	ts->client = client;
	ts->irq = client->irq;
	ts->dev = &client->dev;
	ts->chip_data = chip_info;
	chip_info->hw_res = &ts->hw_res;
	i2c_set_clientdata(client, ts);

	/* 5. file_operations callbacks binding */
	ts->ts_ops = &brl_goodix_ops;

	/* 6. register common touch device*/
	ret = register_common_touch_device(ts);
	if (ret < 0) {
		goto err_edge_data_alloc;
	}
	chip_info->kernel_grip_support = ts->kernel_grip_support;

	/* 8. create goodix tool node */
	gtx8_init_tool_node(ts);

	/* 9. create goodix debug files */
	Goodix_create_proc(ts, chip_info->goodix_ops);

	goodix_esd_check_enable(chip_info, true);

	TPD_INFO("%s, probe normal end\n", __func__);

	return 0;

err_edge_data_alloc:
	if (chip_info->edge_data) {
		kfree(chip_info->edge_data);
	}
	chip_info->edge_data = NULL;

err_touch_data_alloc:
	if (chip_info->touch_data) {
		kfree(chip_info->touch_data);
	}
	chip_info->touch_data = NULL;

err_register_driver:
	common_touch_data_free(ts);
	ts = NULL;

ts_malloc_failed:
	kfree(chip_info);
	chip_info = NULL;
	ret = -1;

	TPD_INFO("%s, probe error\n", __func__);
	return ret;
}

static int goodix_tp_remove(struct i2c_client *client)
{
	struct touchpanel_data *ts = i2c_get_clientdata(client);

	TPD_INFO("%s is called\n", __func__);
	kfree(ts);

	return 0;
}

static int goodix_i2c_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s: is called\n", __func__);
	tp_i2c_suspend(ts);

	return 0;
}

static int goodix_i2c_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s is called\n", __func__);
	tp_i2c_resume(ts);

	return 0;
}

static const struct i2c_device_id tp_id[] = {
	{TPD_DEVICE, 0},
	{},
};


static struct of_device_id tp_match_table[] = {
	{ .compatible = "Goodix-brl", },
	{ },
	/*
	   static struct of_device_id tp_match_table[] = {
	   { .compatible = TPD_DEVICE, },
	   { },
	 */
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
	.suspend = goodix_i2c_suspend,
	.resume = goodix_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver = {
	.probe = goodix_tp_probe,
	.remove = goodix_tp_remove,
	.id_table = tp_id,
	.driver = {
		.name = TPD_DEVICE,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
		.pm = &tp_pm_ops,
	},
};
/******************* End of I2C Driver and It's dev_pm_ops***********************/

/***********************Start of module init and exit****************************/
static int __init tp_driver_init(void)
{
	TPD_INFO("%s is called\n", __func__);

	/*if (!tp_judge_ic_match(TPD_DEVICE))
	  return -1;*/

	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		TPD_INFO("unable to add i2c driver.\n");
		return -1;
	}

	return 0;
}


static void __exit tp_driver_exit(void)
{
	i2c_del_driver(&tp_i2c_driver);

	return;
}

late_initcall(tp_driver_init);

module_exit(tp_driver_exit);
/***********************End of module init and exit*******************************/

MODULE_DESCRIPTION("GTP Touchpanel Driver");
MODULE_LICENSE("GPL v2");
