/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * VENDOR_EDIT
 * File       : gtx8_tools.h
 * Description: header file for goodix debugging tool code
 * Version   : 1.0
 * Date        : 2019-08-27
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 ****************************************************************/

#ifndef _TOUCHPANEL_TOOL_GTX8_H_
#define _TOUCHPANEL_TOOL_GTX8_H_

#include "../touchpanel_common.h"
#include "goodix_common.h"
#include <linux/miscdevice.h>


/****************************Start of modify declare****************************/
#define NO_ERR                  0
#define EMEMCMP                 1003
#define EBUS                    1000
#define ETIMEOUT                1001
#define ECHKSUM                 1002

/****************************End of modify declare****************************/


struct Goodix_tool_info {
    u8     devicecount;
    bool   esd_handle_support;            /*esd handle support feature*/
    int    *is_suspended;
    void   *chip_data;
    struct hw_resource *hw_res;
    struct i2c_client  *client;
    struct fw_update_info *update_info;
    struct esd_information  *esd_info;

    int  (*reset) (void *chip_data); /*Reset Touchpanel*/
};

int gtx8_init_tool_node(struct touchpanel_data *ts);

#define GTP_ESD_PROTECT 0
#define GTP_DRIVER_VERSION          "V1.4<2015/07/10>"

#define DATA_LENGTH_UINT    512
#define GTP_ADDR_LENGTH     2
#define DATA_LENGTH         (DATA_LENGTH_UINT - GTP_ADDR_LENGTH)

#endif
