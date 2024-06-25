/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2019-07-24     WillianChan    the first version
 * 2020-07-28     WillianChan    add the inclusion of the board.h
 */

#include <stdlib.h>
#include <rtthread.h>
#include "board.h"
#include "drivers/sensor_v2.h"
#include "sensor_dallas_ds18b20.h"
#include "mqtt_sample.h"
#include <rtdevice.h>

/* Modify this pin according to the actual wiring situation */
#define DS18B20_DATA_PIN    GET_PIN(B, 10)

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define REFER_VOLTAGE       3300         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

rt_adc_device_t adc_dev_smt;

int smt_check_init(void)
{
    rt_err_t ret = RT_EOK;
    /* 查找设备 */
    adc_dev_smt = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev_smt == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    return ret;
}

INIT_COMPONENT_EXPORT(smt_check_init);

int smt_adc_get(rt_uint16_t *io_data,rt_uint8_t size)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t value;

    if(adc_dev_smt == NULL)
    {
        rt_kprintf("adc device init fail\r\n");
        memset(io_data,'0',sizeof(rt_uint16_t)*size);
        return -1;
    }

    for (rt_uint8_t i = 0; i < size;i++)
    {
        ret = rt_adc_enable(adc_dev_smt, i);
        value = rt_adc_read(adc_dev_smt, i);
        io_data[i] = value * REFER_VOLTAGE / CONVERT_BITS;
        ret = rt_adc_disable(adc_dev_smt, i);
    }
    return 0;
}

// 定义结构体来存储小时、分钟和秒
struct Time {
    int hours;
    int minutes;
    int seconds;
};

static void read_temp_entry(void *parameter)
{
    struct Time time_experience;
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;
    rt_bool_t mqtt_status = 0;
    char str_send_mqtt[100] = {0};
    rt_uint16_t bat_vol[2] = {0};

    if(mqtt_start_ex())
    {
        rt_kprintf("mqtt start fail\r\n");
    }
    else
    {
        mqtt_status = 1;
        rt_kprintf("mqtt start ok\r\n");
    }
    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_GET_ID, (void *)100);

    smt_check_init();

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            smt_adc_get(&bat_vol,1);            

            // 计算小时、分钟和秒
            sensor_data.timestamp = sensor_data.timestamp / 1000;
            time_experience.hours = sensor_data.timestamp / 3600;
            sensor_data.timestamp = sensor_data.timestamp % 3600;
            time_experience.minutes = sensor_data.timestamp / 60;
            time_experience.seconds = sensor_data.timestamp % 60;

            if (sensor_data.data.temp >= 0)
            {
                // rt_kprintf("temp:%3d.%dC, timestamp:%5d\n",
                //            sensor_data.data.temp / 10,
                //            sensor_data.data.temp % 10,
                //            sensor_data.timestamp);

                rt_sprintf(str_send_mqtt, "temp : %3d.%dC,timestamp : %d:%d:%d, bat_vol : %d", sensor_data.data.temp / 10,
                        sensor_data.data.temp % 10, time_experience.hours, time_experience.minutes,
                        time_experience.seconds,bat_vol[0]);
                }
                else
                {
                    // rt_kprintf("temp:-%2d.%dC, timestamp:%5d\n",
                    //            abs(sensor_data.data.temp / 10),
                    //            abs(sensor_data.data.temp % 10),
                    //            sensor_data.timestamp);

                rt_sprintf(str_send_mqtt, "temp : -%3d.%dC,timestamp : %d:%d:%d, bat_vol : %d", sensor_data.data.temp / 10,
                        sensor_data.data.temp % 10, time_experience.hours, time_experience.minutes,
                        time_experience.seconds,bat_vol[0]);
            }
            
            if(mqtt_status)
            {
                mqtt_publish_ex(str_send_mqtt);
            }
            
        }
        rt_thread_mdelay(1000);
        rt_thread_mdelay(1000);
        rt_thread_mdelay(1000);
        rt_thread_mdelay(1000);
        rt_thread_mdelay(1000);
    }
}

static int ds18b20_read_temp_sample(void)
{
    rt_thread_t ds18b20_thread;

    ds18b20_thread = rt_thread_create("18b20tem",
                                      read_temp_entry,
                                      "temp_ds18b20",
                                      1024,
                                      RT_THREAD_PRIORITY_MAX / 2,
                                      20);
    if (ds18b20_thread != RT_NULL)
    {
        rt_thread_startup(ds18b20_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(ds18b20_read_temp_sample);

static int rt_hw_ds18b20_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.user_data = (void *)DS18B20_DATA_PIN;
    rt_hw_ds18b20_init("ds18b20", &cfg);
    
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_ds18b20_port);
