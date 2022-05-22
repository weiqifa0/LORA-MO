/**
 * @file tuya_device.c
 * @author changmen
 * @brief 宠物喂食器开源demo
 * @version 1.0
 * @copyright Copyright © HANGZHOU 2021 Tuya Inc. All rights reserved.
 *
 */

//头文件
#include "uni_log.h"
#include "tuya_device.h"
#include "tuya_os_adapter.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_led.h"
#include "tuya_uart.h"
#include "tuya_gpio.h"
#include "tuya_key.h"
#include "uni_time_queue.h"
#include "gw_intf.h"
#include "uni_thread.h"
#include "tuya_dp_proc.h"
#include "tuya_hw_set.h"
#include "tuya_user_handle.h"
#include "tuya_hw_handle.h"
#include "gpio_test.h"
#include "tuya_pin.h"
#include "tuya_pwm.h"
#include "tuya_driver.h"

STATIC GW_WIFI_NW_STAT_E wifi_stat; /* wifi的联网状态 */
STATIC TIMER_ID wifi_stat_timer;

typedef UCHAR_T LED_STAT_SET;
#define NET_LINK                0
#define NET_UNLINK              1
#define SENDOR_STUDY            2
#define SENDOR_STUDY_EXIT       3

tuya_pwm_t* demo_pwm_init(tuya_pwm_port_t tuya_pwm, \
                        tuya_pin_name_t pin, \
                        float frequency, \
                        float percent, \
                        tuya_pwm_polarity_t polarity);

#define UART_NONBLOCK_EN    0 // 1: Set the serial port to non-blocking
#define BUF_SIZE    256
tuya_uart_t * uart0;

void uart0_init(void);
void uart_set(tuya_uart_t *uart,
              tuya_uart_baudrate_t baudrate,
              tuya_uart_databits_t data_bits,
              tuya_uart_stopbits_t stop_bits,
              tuya_uart_parity_t parity);


/*************************function define***************************/

/**
 * @brief 应用初始化前置准备工作
 * @return VOID_T
 * @note 在此函数中，应用可以执行一些配置设置、事件关注等和具体功能操作无关的工作，应用必须对其进行实现，如果不需要，则实现空函数。
 */
VOID_T pre_app_init(VOID_T)
{

}

/**
 * @brief mf_user_pre_gpio_test_cb gpio测试前置接口，用于对gpio测试做准备工作，
 * @return VOID_T
 * @note 应用必须对其进行实现，如果不需要，则实现空函数
 */
VOID_T mf_user_pre_gpio_test_cb(VOID_T)
{
}

/**
 * @brief mf_user_enter_callback 配置进入产测回调接口
 * @return VOID_T
 * @note 应用必须对其进行实现，如果不需要，则实现空函数
 */
VOID_T mf_user_enter_callback(VOID_T)
{
    PR_DEBUG("###########cmd ");
    OPERATE_RET op_ret = OPRT_OK;
    op_ret = hw_feed_default_config_init();
    if (op_ret != OPRT_OK)
    {
        PR_ERR("json_config err:%d Using the default configuration!", op_ret);
    }

    return;
}

/**
 * @brief mf_user_product_test_cb 上位机产测回调接口
 * @return VOID_T
 * @note 应用必须对其进行实现，如果不需要，则实现空函数
 */
OPERATE_RET mf_user_product_test_cb(USHORT_T cmd, UCHAR_T *data, UINT_T len, OUT UCHAR_T **ret_data, OUT USHORT_T *ret_len)
{

    PR_DEBUG("###########cmd %02x", cmd);
    PR_DEBUG("###########len %d", len);
    //上位机产测入口

    return OPRT_OK;
}

/**
 * @brief ；模块初始化预处理
 * @return
 */
VOID pre_device_init(VOID)
{
    PR_DEBUG("%s", tuya_iot_get_sdk_info());
    PR_DEBUG("%s:%s", APP_BIN_NAME, DEV_SW_VERSION);

#if APP_DEBUG
    PR_DEBUG("DEBUG MODE");
    SetLogManageAttr(TY_LOG_LEVEL_DEBUG);
#else
    SetLogManageAttr(TY_LOG_LEVEL_INFO);
#endif
}

/**
 * @brief ；产测检测处理
 * @return
 */
STATIC VOID prod_test(BOOL_T flag, SCHAR_T rssi)
{
    //是成品产测回调接口

    return;
}

/**
 * @brief ；软件功能初始化
 * @return
 */
VOID app_init(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;

    //tuya_iot_oem_set(TRUE); /* 开启OEM功能 */

    op_ret = hw_feed_default_config_init();
    if (op_ret != OPRT_OK)
    {
        PR_ERR("json_config err:%d Using the default configuration!", op_ret);
    }

    GW_WF_CFG_MTHD_SEL mthd = GWCM_SPCL_AUTOCFG;

    app_cfg_set(mthd, prod_test); /* 查找产测SSID 进入产测模式 */
    return;
}

/**
 * @brief ；获取wifi状态
 * @return
 */
STATIC VOID __get_wf_status(IN CONST GW_WIFI_NW_STAT_E stat)
{
    wifi_stat = stat;
}

/**
 * @brief ；gpio测试
 * @return
 */
#if defined(TY_GPIO_TEST_V2) && (TY_GPIO_TEST_V2 == 1)
BOOL_T gpio_test(IN CONST CHAR_T *in, OUT CHAR_T *out)
{
    return gpio_test_all(in, out);
}
#else
BOOL_T gpio_test(VOID)
{
    return gpio_test_all();
}
#endif

/**
 * @brief ；WiFi led状态设置
 * @return
 */
VOID led_stat_set(HW_FEED_SET_S *feed_config, LED_STAT_SET stat)
{
    if ((stat < 0) || (stat > 3))
    {
        PR_ERR("led stat err!!!");
        return;
    }
    switch (stat)
    {
        case NET_LINK:
        {
            if (feed_config->feed_com_set.led_pin_g.nety == FALSE)
                tuya_set_led_light_type(wf_light, (feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
            else
                tuya_set_led_light_type(wf_light, !(feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
        }
        break;
        case NET_UNLINK:
        {
            if (feed_config->feed_com_set.led_pin_g.netn == FALSE)
                tuya_set_led_light_type(wf_light, (feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
            else
                tuya_set_led_light_type(wf_light, !(feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
        }
        break;
        case SENDOR_STUDY:
        {
            if ((feed_config->feed_com_set.led_pin_g.nety == TRUE &&        //联网后灭
                feed_config->feed_com_set.led_pin_g.detect_sta == TRUE) || //高有效
                (feed_config->feed_com_set.led_pin_g.nety == FALSE &&       //联网后亮
                feed_config->feed_com_set.led_pin_g.detect_sta == FALSE))
                tuya_set_led_light_type(wf_light, !(feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
            else
                tuya_set_led_light_type(wf_light, (feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);

            break;
        }
        case SENDOR_STUDY_EXIT:
        {
            if ((feed_config->feed_com_set.led_pin_g.nety == TRUE &&        //联网后灭
                feed_config->feed_com_set.led_pin_g.detect_sta == TRUE) || //高有效
                (feed_config->feed_com_set.led_pin_g.nety == FALSE &&       //联网后亮
                feed_config->feed_com_set.led_pin_g.detect_sta == FALSE))
                tuya_set_led_light_type(wf_light, (feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);
            else
                tuya_set_led_light_type(wf_light, !(feed_config->feed_com_set.led_pin_g.detect_sta), 0, 0);

            break;
        }
    }
}

/**
 * @brief ；WiFi 状态变化处理回调
 * @return
 */
STATIC VOID wifi_stat_timer_cb(UINT_T timerID, PVOID_T pTimerArg)
{
    STATIC GW_WIFI_NW_STAT_E stat = STAT_LOW_POWER;

    if (stat != wifi_stat)
    {
        PR_DEBUG("size:%d", tuya_hal_system_getheapsize());
        PR_NOTICE("wifi_stat:%d", wifi_stat);
        switch (wifi_stat)
        {
        case STAT_UNPROVISION:
        {
            if (get_feed_config()->feed_com_set.led_pin_g.detect_sta)
                tuya_set_led_light_type(wf_light, OL_FLASH_LOW, 250, LED_TIMER_UNINIT);
            else
                tuya_set_led_light_type(wf_light, OL_FLASH_HIGH, 250, LED_TIMER_UNINIT);
            break;
        }
        case STAT_AP_STA_UNCFG:
        {
            if (get_feed_config()->feed_com_set.led_pin_g.detect_sta)
                tuya_set_led_light_type(wf_light, OL_FLASH_LOW, 1500, LED_TIMER_UNINIT);
            else
                tuya_set_led_light_type(wf_light, OL_FLASH_HIGH, 1500, LED_TIMER_UNINIT);
            break;
        }
        case STAT_LOW_POWER:
        case STAT_AP_STA_DISC:
        case STAT_STA_DISC:
        case STAT_AP_STA_CONN:
        case STAT_STA_CONN:
        case STAT_OFFLINE:
        case STAT_MQTT_OFFLINE:
        {
            HW_FEED_SET_S *feed_config = get_feed_config();
            led_stat_set(feed_config, NET_UNLINK);
            break;
        }
        case STAT_CLOUD_CONN:
        case STAT_AP_CLOUD_CONN:
        {
            HW_FEED_SET_S *feed_config = get_feed_config();
            led_stat_set(feed_config, NET_LINK);
            break;
        }
        case STAT_UNPROVISION_AP_STA_UNCFG:
        {
            if (get_feed_config()->feed_com_set.led_pin_g.detect_sta)
                tuya_set_led_light_type(wf_light, OL_FLASH_LOW, 250, LED_TIMER_UNINIT);
            else
                tuya_set_led_light_type(wf_light, OL_FLASH_HIGH, 250, LED_TIMER_UNINIT);
            break;
        }
        default:
        {
            tuya_set_led_light_type(wf_light, (get_feed_config()->feed_com_set.led_pin_g.detect_sta), 0, 0);
            break;
        }
        }
        stat = wifi_stat;
    }

    return;
}

/**
 * @brief ；WiFi 按键回调处理函数
 * @return
 */
STATIC VOID key_process(IN tuya_pin_name_t port, IN PUSH_KEY_TYPE_E type, IN INT_T cnt)
{
    PR_DEBUG("port: %d", port);
    PR_DEBUG("type: %d", type);
    PR_DEBUG("cnt: %d", cnt);

    OPERATE_RET op_ret = OPRT_OK;
    HW_FEED_SET_S *feed_config = get_feed_config();
    if (feed_config->feed_com_set.reset_pin.key_num == port)
    {
        if (LONG_KEY == type)
        {
            op_ret = tuya_iot_wf_gw_unactive();
            if (OPRT_OK != op_ret)
            {
                PR_ERR("tuya_iot_wf_gw_unactive op_ret:%d", op_ret);
                return;
            }
        }
        else if (NORMAL_KEY == type)
        {
            PR_NOTICE("remain size:%d", tuya_hal_system_getheapsize());
        }
    }
}

/**
 * @brief ；设备重置回调
 * @return
 */
VOID dev_gw_reset_cb(GW_RESET_TYPE_E type)
{
    PR_NOTICE("user_gw_reset_cb:%d", type);
    switch (type)
    {
    case GW_LOCAL_RESET_FACTORY:
    case GW_LOCAL_UNACTIVE:  //本地重置
    case GW_REMOTE_UNACTIVE: // app解绑
    case GW_REMOTE_RESET_FACTORY:
        break;
    case GW_RESET_DATA_FACTORY: //恢复出厂设置
        tuya_feed_plan_clean();
        break;
    }
    return;
}

/**
 * @brief ；obj类型dp回调
 * @return
 */
VOID dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    for (UINT_T i = 0; i < dp->dps_cnt; i++)
    {
        PR_NOTICE("######dpid:%d", dp->dps[i].dpid);
        // obj dp控制处理
        tuya_obj_dp_recv_handler(&dp->dps[i]);
    }
    return;
}

/**
 * @brief ；raw类型dp回调
 * @return
 */
VOID dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    // raw dp控制处理
    PR_NOTICE("######dpid:%d", dp->dpid);
    tuya_raw_dp_recv_handler(dp);
    return;
}

/**
 * @brief ；模块初始化入口
 * @return
 */
OPERATE_RET device_init(VOID) //main
{
    OPERATE_RET op_ret = OPRT_OK;
    LED_HANDLE led_handle;
    uint8_t uart0_rx_buf[BUF_SIZE];
    #define LORA_SLAVE_ACK "I'M SLAVE!!!"
    #define COMMAND_LENGTH 10
    #define MAC_LENGTH 6
    #define SYSTEM_SN 0x001e1000
    uint8_t CMD[COMMAND_LENGTH] = {0};
    uint8_t MAC[MAC_LENGTH];
    uint8_t length = 0;
    int i = 0;

    PR_DEBUG("CHRIS LORA INIT");
    op_ret = tuya_create_led_handle(TUYA_PA16, TRUE, &led_handle);
    if (OPRT_OK != op_ret) {
        PR_ERR("key_init err:%d", op_ret);
        return;
    }

    tuya_os_adapt_flash_read(SYSTEM_SN, MAC, sizeof(MAC));
    PR_NOTICE("MAC:%.2X%.2X%.2X%.2X%.2X%.2X",MAC[0],MAC[1],MAC[2],MAC[3],MAC[4],MAC[5]);

    tuya_set_led_light_type(led_handle, OL_FLASH_HIGH, 200, 2000);

    uart0_init();

    tuya_uart_write(uart0, "hello uart0, baudrate 9600\r\n", 31);
    /* gpio output init */
    tuya_pin_init(TUYA_PA15, TUYA_PIN_MODE_OUT_PP_HIGH);
    tuya_pin_write(TUYA_PA15, TUYA_PIN_LOW);
    memset(CMD,0,sizeof(CMD));
    PR_NOTICE("device_init ok  free_mem_size:%d", tuya_hal_system_getheapsize());
    PR_DEBUG("CHRIS LORA INIT SUCCESS!!!!!!!");
    for(;;)
    {
        op_ret = tuya_uart_read(uart0, uart0_rx_buf, BUF_SIZE);
        if (op_ret > 0)
        {
            PR_NOTICE("op_ret=%d", op_ret);
            for (i=0; i<op_ret; i++)
            {
                CMD[length+i] = uart0_rx_buf[i];
            }
            length += op_ret;
            PR_NOTICE("R:%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X",
                        CMD[0],CMD[1],CMD[2],CMD[3],CMD[4],CMD[5],CMD[6],CMD[7],CMD[8],CMD[9]);
            PR_NOTICE("MAC:%.2X%.2X%.2X%.2X%.2X%.2X",MAC[0],MAC[1],MAC[2],MAC[3],MAC[4],MAC[5]);
            tuya_uart_write(uart0, uart0_rx_buf, op_ret);
            tuya_uart_write(uart0, MAC, sizeof(MAC));
            memset(uart0_rx_buf,0,sizeof(uart0_rx_buf));
        }
        if (length >= COMMAND_LENGTH) {
            PR_NOTICE("OK:%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X",
                        CMD[0],CMD[1],CMD[2],CMD[3],CMD[4],CMD[5],CMD[6],CMD[7],CMD[8],CMD[9]);
            length = 0;
            if (CMD[0] == 0xEE &&
                CMD[1] == 0xFF &&
                CMD[2] == MAC[5] &&
                CMD[3] == MAC[4] &&
                CMD[4] == MAC[3] &&
                CMD[5] == MAC[2] &&
                CMD[6] == MAC[1] &&
                CMD[8] == 0xF1 &&
                CMD[9] == 0x0A) {
                  tuya_pin_write(TUYA_PA15, TUYA_PIN_HIGH);
                  tuya_set_led_light_type(led_handle, OL_FLASH_HIGH, 200, 2000);
                  PR_NOTICE("CHRIS LORA OPEN SUCCESS!!!!!!!");
            } else if (
                CMD[0] == 0xEE &&
                CMD[1] == 0xFF &&
                CMD[2] == MAC[5] &&
                CMD[3] == MAC[4] &&
                CMD[4] == MAC[3] &&
                CMD[5] == MAC[2] &&
                CMD[6] == MAC[1] &&
                CMD[8] == 0xF0 &&
                CMD[9] == 0x0A) {
                tuya_pin_write(TUYA_PA15, TUYA_PIN_LOW);
                PR_NOTICE("CHRIS LORA CLOSE SUCCESS!!!!!!!");
            }
            memset(CMD,0,sizeof(CMD));
        } else {
            tuya_hal_system_sleep(2);
            tuya_uart_write(uart0, LORA_SLAVE_ACK, sizeof(LORA_SLAVE_ACK));
        }
    }
    return OPRT_OK;
}

/*
 * PWM0 <-> TUYA_PA6
 * PWM1 <-> TUYA_PA7
 * PWM2 <-> TUYA_PA8
 * PWM3 <-> TUYA_PA9
 * PWM4 <-> TUYA_PA24
 * PWM5 <-> TUYA_PA26
 */
/**
* @brief initialization pwm
*
* @param[in] tuya_pwm: pwm port, this parameter must be a value of @ref PWMx
* @param[in] pin: pwm pin, this parameter must be a value of @ref TUYA_PAx
* @param[in] frequency: pwm frequency
* @param[in] percent: pwm percent, range: 0~1.0
* @param[in] polarity: pwm polarity, this parameter must be a value of @ref TUYA_PWM_POSITIVE or TUYA_PWM_NEGATIVE
* @return
*/
tuya_pwm_t* demo_pwm_init(tuya_pwm_port_t tuya_pwm, \
                        tuya_pin_name_t pin, \
                        float frequency, \
                        float percent, \
                        tuya_pwm_polarity_t polarity)
{
    tuya_pwm_t* tuya_pwm_handle = NULL;
    tuya_pwm_handle = tuya_driver_find(TUYA_DRV_PWM, tuya_pwm);
//    TUYA_PWM_CFG(tuya_pwm_handle, pin, frequency, percent);

    tuya_pwm_handle->cfg.pin = pin;
    tuya_pwm_handle->cfg.polarity = polarity;
    tuya_pwm_handle->cfg.period_ns = (uint32_t)1000000000 / (frequency);
    tuya_pwm_handle->cfg.percent = percent;
    tuya_pwm_handle->cfg.pulse_ns = (uint32_t)((tuya_pwm_handle)->cfg.period_ns * (percent));

    tuya_pwm_init(tuya_pwm_handle);

    return tuya_pwm_handle;
}

/**
* @brief init uart0
*
* @param[in] none
* @return none
*/
void uart0_init(void)
{
    int32_t op_ret;

/* TY_UART0:
 *      TX <-> TX1
 *      RX <-> RX1
 * TY_UART1:
 *      TX <-> TX2
 *      RX <-> RX2
 */
    uart0 = (tuya_uart_t *)tuya_driver_find(TUYA_DRV_UART, TY_UART0);
    if (NULL == uart0) {
        PR_DEBUG("find uart0 fail");
        return;
    }

#if UART_NONBLOCK_EN
    TUYA_UART_8N1_CFG(uart0, TUYA_UART_BAUDRATE_115200, 256, (TUYA_DRV_INT_RX_FLAG | TUYA_DRV_NONBLOCK_FLAG));
#else
    TUYA_UART_8N1_CFG(uart0, TUYA_UART_BAUDRATE_115200, 256, TUYA_DRV_INT_RX_FLAG);
#endif

    op_ret = tuya_uart_init(uart0);
    if (OPRT_OK != op_ret) {
        PR_ERR("uart init fail, error code: %d", op_ret);
    }

    return;
}

/**
* @brief set uart baudrate
*
* @param[in] uart: uart handle
* @param[in] baudrate: uart baudrate, this parameter must be a value of @ref TUYA_UART_BAUDRATE_xxx
* @param[in] data_bits: uart data bits, this parameter must be a value of @ref TUYA_UART_DATA_BITx
* @param[in] stop_bits: uart stop bits, this parameter must be a value of @ref TUYA_UART_STOP_BITx
* @param[in] parity: uart parity bits, this parameter must be a value of @ref TUYA_UART_PARITY_xxx
* @return none
*/
void uart_set(tuya_uart_t *uart,
              tuya_uart_baudrate_t baudrate,
              tuya_uart_databits_t data_bits,
              tuya_uart_stopbits_t stop_bits,
              tuya_uart_parity_t parity)
{
    int32_t op_ret;

    uart->cfg.baudrate = baudrate;
    uart->cfg.databits = data_bits;
    uart->cfg.stopbits = stop_bits;
    uart->cfg.parity = parity;

    op_ret = tuya_uart_control(uart, TUYA_DRV_CONFIG_CMD, &(uart->cfg));
    if (OPRT_OK != op_ret) {
        PR_ERR("uart0 cfg baud failed, error code: %d", op_ret);
    }
}
