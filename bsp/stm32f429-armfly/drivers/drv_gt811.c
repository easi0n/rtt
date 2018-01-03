/*
 * File      : drv_ft5x06.c
 *             ft5x06 touch driver
 * COPYRIGHT (C) 2006 - 2017, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-08     Yang        the first version
 */
 
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/event.h>
#include <rtgui/rtgui_server.h>
#endif

#include "board.h"
#define GT811_READ_XY_REG 	0x721	/* 坐标寄存器 */
#define GT811_CONFIG_REG	0x6A2	/* 配置参数寄存器 */
#define BSP_TOUCH_SAMPLE_HZ     30
#define I2CBUS_NAME             "i2c0"
#define GT811_TS_ADDR          	0x5D //数据手册中定义的地址是0xBA，rtt的i2c驱动不包含读写控制位，在驱动中要将地址左移1位，因此此处将oxBA先右移1位，即除以2
#define TP_MAX_TOUCH_POINT      5
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(...)           rt_kprintf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   
#endif


#define CTRL_NOAUTO_MONITOR    0x00
#define CTRL_AUTO_MONITOR      0x01

#define PMODE_ACTIVE           0x00
#define PMODE_MONITOR          0x01
#define PMODE_STANDBY          0x02
#define PMODE_HIBERNATE        0x03

#define G_MODE_POLLING         0x00
#define G_MODE_TRIGGER         0x01

#define TOUCH_POINT_GET_EVENT(T)    ((touch_event_t)((T).XH >> 6))
#define TOUCH_POINT_GET_ID(T)       ((T).YH >> 4)
#define TOUCH_POINT_GET_X(T)        ((((T).XH & 0x0f) << 8) | (T).XL)
#define TOUCH_POINT_GET_Y(T)        ((((T).YH & 0x0f) << 8) | (T).YL)

typedef enum _touch_event
{
    kTouch_Down = 0,        /*!< The state changed to touched. */
    kTouch_Up = 1,          /*!< The state changed to not touched. */
    kTouch_Contact = 2,     /*!< There is a continuous touch being detected. */
    kTouch_Reserved = 3     /*!< No touch information available. */
} touch_event_t;

typedef struct _touch_point
{
    touch_event_t TOUCH_EVENT;  /*!< Indicates the state or event of the touch point. */
    uint8_t TOUCH_ID;           /*!< Id of the touch point. This numeric value stays constant between down and up event. */
    uint16_t TOUCH_X;           /*!< X coordinate of the touch point */
    uint16_t TOUCH_Y;           /*!< Y coordinate of the touch point */
} touch_point_t;

typedef struct _gt811_touch_point
{
    uint8_t XH;
    uint8_t XL;
    uint8_t YH;
    uint8_t YL;
} gt811_touch_point_t;

typedef struct _gt811_touch_data
{
    uint8_t TouchpointFlag;
    uint8_t Touchkeystate;
    gt811_touch_point_t TOUCH;
} ft5x06_touch_data_t;

static struct rt_i2c_bus_device *_i2c_bus;

static int _gt811_read(uint8_t *buf,size_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = GT811_TS_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &buf[0];
    msgs[0].len   = 2;

    msgs[1].addr  = GT811_TS_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = &buf[2];
    msgs[1].len   = len - 2;

    if (rt_i2c_transfer(_i2c_bus, msgs, 2) == 2)
        return len;
    else
        return -1;
}

static int _gt811_write(void *buf,size_t len)
{
    struct rt_i2c_msg msgs[1];

    msgs[0].addr  = GT811_TS_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = buf;
    msgs[0].len   = len;

    if (rt_i2c_transfer(_i2c_bus, msgs, 1) == 1)
        return len;
    else
        return -1;
}
//#ifdef RT_USING_FINSH
//static int search_ft5x06(void)
//{
//    struct rt_i2c_msg msgs[2];
//    uint8_t cmd = 0xA3;
//    uint8_t buf = 0;

//    msgs[0].flags = RT_I2C_WR;
//    msgs[0].buf   = &cmd;
//    msgs[0].len   = sizeof(cmd);

//    msgs[1].flags = RT_I2C_RD;
//    msgs[1].buf   = &buf;
//    msgs[1].len   = 1;

//    for (int i = 0; i <= 0x7f; i++)
//    {
//        int len;
//        msgs[0].addr  = i;
//        msgs[1].addr  = i;
//        len = rt_i2c_transfer(_i2c_bus, msgs, 2);
//        if (len == 2)
//        {
//            rt_kprintf("add:%02X transfer success, id: %02X\n", i, buf);
//        } 
//    }
//    return 0;
//}
//FINSH_FUNCTION_EXPORT_ALIAS(search_ft5x06, sft, search ft5x06 chip);

//static int ft5x06_dump(void)
//{
//    uint8_t i;
//    uint8_t reg_value;
//    
//    DEBUG_PRINTF("[FTS] Touch Chip\r\n");
//        
//    for (i = 0; i <= 255; i++)
//    {
//        _ft5x06_read(i, &reg_value, 1);
//        
//        if (i % 8 == 7)
//            DEBUG_PRINTF("0x%02X = 0x%02X\r\n", i, reg_value);
//        else
//            DEBUG_PRINTF("0x%02X = 0x%02X ", i, reg_value);
//    }
//    DEBUG_PRINTF("\n");
//    
//    return 0;
//}
//FINSH_FUNCTION_EXPORT_ALIAS(ft5x06_dump, ftdump, ft5x06 dump registers);
//#endif

static int gt811_read_touch(touch_point_t *dp)
{
#if 0
    uint8_t data[33];
    int i;
    
    _ft5x06_read(0, data, sizeof(data));
    
    for (i = 0; i < sizeof(data)/sizeof(data[0]); i++)
    {
        DEBUG_PRINTF("%02X ", data[i]);
    }
    DEBUG_PRINTF("\n");
    
    return -1;
#else
    uint8_t touch_data[8] = {0x07, 0x21, 0};
    
    _gt811_read(touch_data, 8);

//    dp->TOUCH_X = TOUCH_POINT_GET_Y(touch_data.TOUCH);
//    dp->TOUCH_Y = TOUCH_POINT_GET_X(touch_data.TOUCH);

	if(touch_data[2] & 0x1)
	{  
//        input_y = 479-((point_data[4]<<8)|point_data[5]);  
//        input_x = 799-((point_data[6]<<8)|point_data[7]);  
//        input_p = point_data[8];  
		dp->TOUCH_X = ((uint16_t)touch_data[4] << 8) + touch_data[5];
		dp->TOUCH_Y = ((uint16_t)touch_data[6] << 8) + touch_data[7];
//        printk("stat: %d, x: %d, y: %d, p: %d\n", point_data[2], input_x, input_y,  
//            input_p);  
		DEBUG_PRINTF(" ==> status : (%d, %d)\n", dp->TOUCH_X, dp->TOUCH_Y);
		 return 0;
    }    

//    if (touch_data.TD_STATUS != 0)
//        return 0;
//    else
        return -1;
#endif
}

static void _touch_session()
{
    touch_point_t tpd;
#ifdef RT_USING_RTGUI
    struct rtgui_event_mouse emouse;
#endif
    
    gt811_read_touch(&tpd);

#ifdef RT_USING_RTGUI
    emouse.parent.sender = RT_NULL;
    emouse.wid = RT_NULL;

    emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
    emouse.button = RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_DOWN;
    emouse.x = tpd.TOUCH_X;
    emouse.y = tpd.TOUCH_Y;
    emouse.ts = rt_tick_get();
    emouse.id = emouse.ts;
    if (emouse.id == 0) emouse.id = 1;
    rtgui_server_post_event(&emouse.parent, sizeof(emouse));
#endif

    do
    {
        rt_thread_delay(RT_TICK_PER_SECOND / BSP_TOUCH_SAMPLE_HZ);
        if (gt811_read_touch(&tpd) != 0)
            break;

#ifdef RT_USING_RTGUI
        emouse.parent.type = RTGUI_EVENT_MOUSE_MOTION;
        emouse.x = tpd.TOUCH_X;
        emouse.y = tpd.TOUCH_Y;
        emouse.ts = rt_tick_get();
        rtgui_server_post_event(&emouse.parent, sizeof(emouse));
#endif
    }
    while (1);

#ifdef RT_USING_RTGUI
    /* Always send touch up event. */
    emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
    emouse.button = RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP;
    emouse.x = tpd.TOUCH_X;
    emouse.y = tpd.TOUCH_Y;
    emouse.ts = rt_tick_get();
    rtgui_server_post_event(&emouse.parent, sizeof(emouse));
#endif

}
static int gt811_init(void)
{  
    int16_t ret = -1;  
    uint8_t config_info[] = 
		{  
        0x06,0xA2,  
//        0x12,0x10,0x0E,0x0C,0x0A,0x08,0x06,0x04,0x02,0x00,0xE2,0x53,0xD2,0x53,0xC2,0x53,  
//        0xB2,0x53,0xA2,0x53,0x92,0x53,0x82,0x53,0x72,0x53,0x62,0x53,0x52,0x53,0x42,0x53,  
//        0x32,0x53,0x22,0x53,0x12,0x53,0x02,0x53,0xF2,0x53,0x0F,0x13,0x40,0x40,0x40,0x10,  
//        0x10,0x10,0x0F,0x0F,0x0A,0x35,0x25,0x0C,0x03,0x00,0x05,0x20,0x03,0xE0,0x01,0x00,  
//        0x00,0x34,0x2C,0x36,0x2E,0x00,0x00,0x03,0x19,0x03,0x08,0x00,0x00,0x00,0x00,0x00,  
//        0x14,0x10,0xEC,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x40,  
//        0x30,0x3C,0x28,0x00,0x00,0x00,0x00,0xC0,0x12,0x01   
/*
		0x6A2  R/W  Sen_CH0    触摸屏 1 号感应线对应的 IC 感应线 
		0x6A3  R/W  Sen_CH1    触摸屏 2 号感应线对应的 IC 感应线
		0x6A4  R/W  Sen_CH2    触摸屏 3 号感应线对应的 IC 感应线
		0x6A5  R/W  Sen_CH3    触摸屏 4 号感应线对应的 IC 感应线
		0x6A6  R/W  Sen_CH4    触摸屏 5 号感应线对应的 IC 感应线
		0x6A7  R/W  Sen_CH5    触摸屏 6 号感应线对应的 IC 感应线 
		0x6A8  R/W  Sen_CH6    触摸屏 7 号感应线对应的 IC 感应线
		0x6A9  R/W  Sen_CH7    触摸屏 8 号感应线对应的 IC 感应线
		0x6AA  R/W  Sen_CH8    触摸屏 9 号感应线对应的 IC 感应线
		0x6AB  R/W  Sen_CH9    触摸屏 10 号感应线对应的 IC 感应线
	*/
    0x12,0x10,0x0E,0x0C,0x0A,0x08,0x06,0x04,0x02,0x00,

	/*
	0x6AC  R/W  Dr0_Con  CHSELEF0  F1DELAY0
	0x6AD  R/W  Dr0_Con  F2DELAY0  F3DELAY0
	
	0x6AE  R/W  Dr1_Con  CHSELEF1  F1DELAY1
	0x6AF  R/W  Dr1_Con  F2DELAY1  F3DELAY1
	
	0x6B0  R/W  Dr2_Con  CHSELEF2  F1DELAY2
	0x6B1  R/W  Dr2_Con  F2DELAY2  F3DELAY2
	
	0x6B2  R/W  Dr3_Con  CHSELEF3  F1DELAY3
	0x6B3  R/W  Dr3_Con  F2DELAY3  F3DELAY3
	
	0x6B4  R/W  Dr4_Con  CHSELEF4  F1DELAY4
	0x6B5  R/W  Dr4_Con  F2DELAY4  F3DELAY4
	
	0x6B6  R/W  Dr5_Con  CHSELEF5  F1DELAY5
	0x6B7  R/W  Dr5_Con  F2DELAY5  F3DELAY5
	
	0x6B8  R/W  Dr6_Con  CHSELEF6  F1DELAY6
	0x6B9  R/W  Dr6_Con  F2DELAY6  F3DELAY6
	
	0x6BA  R/W  Dr7_Con  CHSELEF7  F1DELAY7
	0x6BB  R/W  Dr7_Con  F2DELAY7  F3DELAY7
	
	0x6BC  R/W  Dr8_Con  CHSELEF8  F1DELAY8
	0x6BD  R/W  Dr8_Con  F2DELAY8  F3DELAY8
	
	0x6BE  R/W  Dr9_Con  CHSELEF9  F1DELAY9
	0x6BF  R/W  Dr9_Con  F2DELAY9  F3DELAY9
	
	0x6C0  R/W  Dr10_Con  CHSELEF10  F1DELAY10
	0x6C1  R/W  Dr10_Con  F2DELAY10  F3DELAY10
	
	0x6C2  R/W  Dr11_Con  CHSELEF11  F1DELAY11
	0x6C3  R/W  Dr11_Con  F2DELAY11  F3DELAY11
	
	0x6C4  R/W  Dr12_Con  CHSELEF12  F1DELAY12
	0x6C5  R/W  Dr12_Con  F2DELAY12  F3DELAY12
	
	0x6C6  R/W  Dr13_Con  CHSELEF13  F1DELAY13
	0x6C7  R/W  Dr13_Con  F2DELAY13  F3DELAY13
	
	0x6C8  R/W  Dr14_Con  CHSELEF14  F1DELAY14
	0x6C9  R/W  Dr14_Con  F2DELAY14  F3DELAY14
	
	0x6CA  R/W  Dr15_Con  CHSELEF15  F1DELAY15
	0x6CB  R/W  Dr15_Con  F2DELAY15  F3DELAY15
	*/
	0x05,0x55,0x15,0x55,0x25,0x55,0x35,0x55,0x45,0x55,0x55,0x55,0x65,0x55,0x75,0x55,
	0x85,0x55,0x95,0x55,0xA5,0x55,0xB5,0x55,0xC5,0x55,0xD5,0x55,0xE5,0x55,0xF5,0x55,	
	
	/*
	0x6CC  R/W  ADCCFG  芯片扫描控制参数
	0x6CD  R/W  SCAN    芯片扫描控制参数
	*/
	0x1B,0x03,
	
	/*
	0x6CE  R/W  F1SET  驱动脉冲 1 频率
	0x6CF  R/W  F2SET  驱动脉冲 2 频率
	0x6D0  R/W  F3SET  驱动脉冲 3 频率
	0x6D1  R/W  F1PNUM  驱动脉冲 1 个数
	0x6D2  R/W  F2PNUM  驱动脉冲 2 个数 
	0x6D3  R/W  F3PNUM  驱动脉冲 3 个数
	*/
	0x00,0x00,0x00,0x13,0x13,0x13,
	
	/* 0x6D4  R/W  TOTALROW  全部使用的驱动通道数(屏的驱动线+按键驱动线) */
	0x0F,
	
	/*
	0x6D5  R/W  TSROW  用在屏上的驱动线
	0x6D6  R/W  TOTALCOL  用在屏上的感应线
	*/
	0x0F,0x0A,
	
	/*
	0x6D7  R/W  Sc_Touch  屏幕按键阈值
	0x6D8  R/W  Sc_Leave  屏幕松键阈值
	*/
	0x50,0x30,
	
	/* 
	0x6D9  R/W  Md_Switch  保留  DD2    R1  R0  INT    SITO    RT    ST 
	0x6DA  R/W  LPower_C  保留  Auto 无按键进低功耗时间，0-63 有效，以 s 为单位
	*/
	0x05,0x03,
	
	/* 0x6DB  R/W  Refresh  触摸刷新速率控制参数（50Hz~100Hz）：0-100 有效 */	
	0x64,
	
	/* 0x6DC  R/W  Touch_N  保留  使能触摸点个数：1-5 有效 */
	0x05,
	
	/* 
	0x6DD  R/W  X_Ou_Max_L X 坐标输出最大值  480
	0x6DE  R/W  X_Ou_Max_H
	
	0x6DF  R/W  Y_Ou_Max_L  Y 坐标输出最大值  800
	0x6E0  R/W  Y_Ou_Max_H
	*/
#if 0
	0x58,0x02,
	0x00,0x04,
#else
	0xe0,0x01,
	0x20,0x03,
#endif

	/*
	0x6E1  R/W  X _Th  X 坐标输出门限：0-255，以 4 个原始坐标点为单位
	0x6E2  R/W  Y_Th  Y 坐标输出门限：0-255，以 4 个原始坐标点为单位
	*/ 
	0x00,  0x00,
	
	/*
	0x6E3  R/W  X_Co_Sm  X 方向平滑控制变量，0-255 可配置，0 表示关
	0x6E4  R/W  Y_Co_Sm  Y 方向平滑控制变量，0-255 可配置，0 表示关
	0x6E5  R/W  X_Sp_Lim  X 方向平滑上限速度：0-255 可配置，0 表示关
	0x6E6  R/W  Y_Sp_ Lim  Y 方向平滑上限速度：0-255 可配置，0 表示关
	*/
	0x32,0x2C,0x34,0x2E,
	
	/*
	0x6E7  R/W  X_Bor_Lim  Reserved  Reserved
	0x6E8  R/W  Y_Bor_Lim  Reserved  Reserved
	*/
	0x00,0x00,
	
	/* 0x6E9  R/W  Filter  丢弃数据帧数  坐标窗口滤波值，以 4 为基数 */
	0x04,
	
	/* 0x6EA  R/W  Large_Tc  0-255 有效：单一触摸区包含结点数大于此数会判为大面积触摸 */
	0x14,
	
	/* 0x6EB  R/W  Shake_Cu  Touch 事件建立去抖  手指个数从多到少去抖 */
	0x22,
	
	/* 0x6EC  R/W  Noise_R  保留  白噪声削减量（低 nibble）有效 */
	0x04,
	
	/* 0x6ED~0x6F1 R/W    保留 */
	0x00,0x00,0x00,0x00,0x00,
	
	
    0x20,0x14,0xEC,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x30,
    0x25,0x28,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x01,			
    };  
    config_info[62] = 480 >> 8;  
    config_info[61] = 480 & 0xff;  
    config_info[64] = 800 >> 8;  
    config_info[63] = 800 & 0xff;  
    ret = _gt811_write(config_info, sizeof(config_info)/sizeof(config_info[0]));  
    if(ret < 0)  {  
        DEBUG_PRINTF("GT811 Send config failed!\n");  
        return ret;   
    }  
		DEBUG_PRINTF("GT811 inited successed!\n"); 
    return 0;  
}  

static void touch_entry(void *p)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __HAL_RCC_GPIOH_CLK_ENABLE();

//    /*Configure GPIO pin : PH7 */
//    GPIO_InitStruct.Pin = GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		
    while(1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND / 60);

//        if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_7) == GPIO_PIN_RESET)
        {
            _touch_session();
        }
//        else
//            continue;
    }
}

uint16_t GT811_ReadVersion(void)
{
	uint8_t buf[4] = {0x7,0x17,0};

	_gt811_read(buf, 4);

	return ((uint16_t)buf[2] << 8) + buf[3];
}

int gt811_hw_init(void)
{
    rt_thread_t tid;
    rt_device_t dev;

    dev = rt_device_find(I2CBUS_NAME);
    if (!dev) 
    {
        DEBUG_PRINTF("search device failed: %s\n", I2CBUS_NAME);
        return -1;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        DEBUG_PRINTF("open device failed: %s\n", I2CBUS_NAME);
        return -1;
    }

    DEBUG_PRINTF("gt811 set i2c bus to %s\n", I2CBUS_NAME);
    _i2c_bus = (struct rt_i2c_bus_device *)dev;
		gt811_init();
	#if 1
	uint16_t ver;
	
	ver = GT811_ReadVersion();
	DEBUG_PRINTF("GT811 Version : %04X\r\n", ver);
#endif
    tid = rt_thread_create("touch", touch_entry, RT_NULL, 2048, 10, 20);
    if (!tid)
    {
        rt_device_close(dev);
        return -1;
    }

    rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(gt811_hw_init);
