/* ESPRESSIF MIT License
 * 
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 * 
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/sntp.h"
//#include "app_client.h"
//#include "sdlog.h"
#include <sys/param.h>
#include "esp_event.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"
//#include "addr_from_stdin.h"
#include "lwip/sockets.h"
#include <pthread.h>
#include "app_wifi.h"
#include "driver/gpio.h"
#include "app_camera.h"
#include "esp_camera.h"

/* The examples use WiFi configuration that you can set via 'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#if 1
#define EXAMPLE_ESP_WIFI_SSID "TEST_AP2"
#define EXAMPLE_ESP_WIFI_PASS "idataxa123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 200
#define EXAMPLE_ESP_WIFI_AP_SSID ""
#define EXAMPLE_ESP_WIFI_AP_PASS CONFIG_ESP_WIFI_AP_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_MAX_STA_CONN
#define EXAMPLE_IP_ADDR CONFIG_SERVER_IP
#define EXAMPLE_ESP_WIFI_AP_CHANNEL CONFIG_ESP_WIFI_AP_CHANNEL

#else
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY
#define EXAMPLE_ESP_WIFI_AP_SSID CONFIG_ESP_WIFI_AP_SSID
#define EXAMPLE_ESP_WIFI_AP_PASS CONFIG_ESP_WIFI_AP_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_MAX_STA_CONN
#define EXAMPLE_IP_ADDR CONFIG_SERVER_IP
#define EXAMPLE_ESP_WIFI_AP_CHANNEL CONFIG_ESP_WIFI_AP_CHANNEL

#endif

static int s_retry_num = 0;

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR "192.168.10.200"
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "192.168.1.200"
#endif
#define PORT 20001

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
int led_duty = 0;
bool isStreaming = false;
#ifdef CONFIG_LED_LEDC_LOW_SPEED_MODE
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#else
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#endif
#endif

static const char *TAG = "JZ_client";
//static const char *payload = "TEST 123456";

// app socket
static int m_sock = -1;
static int m_tcp_status = -1;
static enum esp_status m_esp_status = ESP_STATUS_INIT;

#define ESP_BUFFER_LENGTH 128
#define LEN_HEART_BEAT 10
static char mBuffer[ESP_BUFFER_LENGTH];

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
static void enable_led(bool en)
{ // Turn LED On or Off
    int duty = en ? led_duty : 0;
    ESP_LOGI(TAG, "Set LED intensity en is %d", en);
    ESP_LOGI(TAG, "Set LED intensity to %d", duty);

    if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY))
    {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
    ESP_LOGI(TAG, "Set LED intensity to %d", duty);
}
#endif

static int esp_send(char *data, int len);
static int sendUnlockedSocket(char *data, int len);
static int sendLockedSocket(char *data, int len);
static int recvSocket(char *data, int length);
static int esp_send(char *data, int len);
static void esp_heart_timer();
pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

static int first_takepic()
{
    camera_fb_t *fb = NULL;
    struct timeval tv_now;
    struct timeval tv_no1;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    int ret = 0;
    int camera_retry = 0;
    static int64_t last_frame = 0;

    gettimeofday(&tv_now, NULL);
    ESP_LOGI(TAG, "takePicture time:%lld.%lld ", (int64_t)tv_now.tv_sec, (int64_t)tv_now.tv_usec);

    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    do
    {
        if (camera_retry > 2)
        {
            ESP_LOGE(TAG, "stream_handler, reset camera.");
            esp_restart();
            camera_retry = 0;
            break;
        }

        fb = esp_camera_fb_get();
        if (!fb)
        {
            camera_retry++;
            ESP_LOGE(TAG, "Camera capture failed, camera_retry: %d", camera_retry);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            //m_esp_status = ESP_STATUS_CAMERA_FAILED;
            continue;
        }
        else
        {
            camera_retry = 0;
            gettimeofday(&tv_no1, NULL);
            //time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            ESP_LOGI(TAG, "esp_camera_fb_get time:%lld.%lld ", (int64_t)tv_no1.tv_sec, (int64_t)tv_no1.tv_usec);
            ESP_LOGE(TAG, "pic_width is %d pic_height is %d", fb->width, fb->height);
            ESP_LOGI(TAG, "Camera capture format=%d", fb->format);
            if (fb->format != PIXFORMAT_JPEG)
            {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted)
                {
                    ESP_LOGE(TAG, "JPEG compression failed");
                    //ret = -1;
                }
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        ESP_LOGI(TAG, "jpg _buffer_ len is %d", _jpg_buf_len);
        if (fb)
        {
            esp_camera_fb_return(fb);

            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        gettimeofday(&tv_no1, NULL);
        ESP_LOGI(TAG, "esp_camera_fb_return time:%lld.%lld ", (int64_t)tv_no1.tv_sec, (int64_t)tv_no1.tv_usec);
        break;
    } while (1);
    return ret;
}

char *log_gettime(void)
{
    static char timebuf[64] = {0};

    struct tm timeinfo;
    struct timeval tv_now;
    time_t now;

    time(&now);
    localtime_r(&now, &timeinfo);
    gettimeofday(&tv_now, NULL);

    memset(timebuf, 0, 64);
    sprintf(timebuf, "%04d%02d%02d %02d:%02d:%02d.%06lld",
            1900 + timeinfo.tm_year, 1 + timeinfo.tm_mon, timeinfo.tm_mday, 8 + timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, (int64_t)tv_now.tv_usec);

    return timebuf;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;

        app_client_main();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG, "connect to the AP fail");
        break;
    }
    default:
        break;
    }
    //mdns_handle_system_event(ctx, event);
    return ESP_OK;
}

void wifi_init_softap()
{
    if (strcmp(EXAMPLE_IP_ADDR, "192.168.4.1"))
    {
        int a, b, c, d;
        sscanf(EXAMPLE_IP_ADDR, "%d.%d.%d.%d", &a, &b, &c, &d);
        tcpip_adapter_ip_info_t ip_info;
        IP4_ADDR(&ip_info.ip, a, b, c, d);
        IP4_ADDR(&ip_info.gw, a, b, c, d);
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
        ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(WIFI_IF_AP));
        ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(WIFI_IF_AP, &ip_info));
        ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(WIFI_IF_AP));
    }
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    snprintf((char *)wifi_config.ap.ssid, 32, "%s", EXAMPLE_ESP_WIFI_AP_SSID);
    wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);
    snprintf((char *)wifi_config.ap.password, 64, "%s", EXAMPLE_ESP_WIFI_AP_PASS);
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(EXAMPLE_ESP_WIFI_AP_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    if (strlen(EXAMPLE_ESP_WIFI_AP_CHANNEL))
    {
        int channel;
        sscanf(EXAMPLE_ESP_WIFI_AP_CHANNEL, "%d", &channel);
        wifi_config.ap.channel = channel;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_AP_SSID, EXAMPLE_ESP_WIFI_AP_PASS);
}

void wifi_init_sta()
{
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    snprintf((char *)wifi_config.sta.ssid, 32, "%s", EXAMPLE_ESP_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, 64, "%s", EXAMPLE_ESP_WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void app_wifi_main()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_mode_t mode = WIFI_MODE_NULL;

    if (strlen(EXAMPLE_ESP_WIFI_AP_SSID) && strlen(EXAMPLE_ESP_WIFI_SSID))
    {
        mode = WIFI_MODE_STA;
    }
    else if (strlen(EXAMPLE_ESP_WIFI_AP_SSID))
    {
        mode = WIFI_MODE_AP;
    }
    else if (strlen(EXAMPLE_ESP_WIFI_SSID))
    {
        mode = WIFI_MODE_STA;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (mode == WIFI_MODE_NULL)
    {
        ESP_LOGI(TAG, "Neither AP or STA have been configured. WiFi will be off.");
        return;
    }

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));

    if (mode & WIFI_MODE_AP)
    {
        wifi_init_softap();
    }

    if (mode & WIFI_MODE_STA)
    {
        wifi_init_sta();
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

esp_timer_handle_t esp_heart_handle = 0;

esp_timer_create_args_t esp_heart_arg = {
    .callback = &esp_heart_timer,
    .arg = NULL,
    .name = "HeartPeriodicTimer"};

static void esp_heart_timer(void *arg)
{
    //struct timeval tv_now;
    //struct tm timeinfo;
    //time_t now;
    if (m_esp_status == ESP_STATUS_REG_SUCCESS)
    {

        int err = sendLockedSocket("HEART_BEAT", LEN_HEART_BEAT);
        ESP_LOGI(TAG, "heart_beat is len is %d", err);
		
        /*time(&now);
        localtime_r(&now, &timeinfo);
        gettimeofday(&tv_now, NULL);
        ESP_LOGI(TAG, "heart_beat %4d-%02d-%02d %02d:%02d:%02d.%lld", 1900 + timeinfo.tm_year, 1 + timeinfo.tm_mon,
                 timeinfo.tm_mday, 8 + timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, (int64_t)tv_now.tv_usec);*/
	}
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static void esp_initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "ntp1.aliyun.com");
    sntp_init();
}

// 0 : fail; 1 : success
/*static int synchronizeTime(struct tm *timeinfo)
{
    struct timeval tv_now;

    // wait for time to be set
    time_t now = 0;

    int ret = 0;

    ESP_LOGE(TAG, "sync time... (%d)", timeinfo->tm_year);
    if (timeinfo->tm_year < (2021 - 1900))
    {
        time(&now);
        localtime_r(&now, timeinfo);

        gettimeofday(&tv_now, NULL);
        ESP_LOGI(TAG, "=while=======time:%lld.%lld", (int64_t)tv_now.tv_sec, (int64_t)tv_now.tv_usec);
    }
    else
    {
        // set timezone to China Standard Time
        setenv("TZ", "CST-8", 1);
        tzset();

        gettimeofday(&tv_now, NULL);
        ESP_LOGI(TAG, "current time:%lld.%lld ", (int64_t)tv_now.tv_sec, (int64_t)tv_now.tv_usec);

        ret = 1;
    }

    return ret;
}

static void heart_beat()
{
    int timeSync = 0;
    struct tm timeinfo;

	ESP_LOGE(TAG, "heart_beat is starting");

    while (1)
    {
        if (timeSync == 0)
            timeSync = synchronizeTime(&timeinfo);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return;
}

void createHeartBeatThread()
{
    pthread_t pthread_id;

    int ret = pthread_create((pthread_t *)&pthread_id, NULL, heart_beat, NULL);
    if (-1 == ret)
    {
        ESP_LOGE(TAG, "pthread_create ");
    }
}
*/
static int stream_handler()
{
    camera_fb_t *fb = NULL;
    //struct timeval _timestamp;
    struct timeval tv_now;
    struct timeval tv_no1;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    int ret = 0;
    int camera_retry = 0;
    static int64_t last_frame = 0;
	int len_of_send = 0;

    gettimeofday(&tv_now, NULL);
    //time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    ESP_LOGI(TAG, "takePicture time:%lld.%lld ", (int64_t)tv_now.tv_sec, (int64_t)tv_now.tv_usec);

    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    do
    {
        if (camera_retry > 2)
        {
            ESP_LOGE(TAG, "stream_handler, reset camera.");
            //esp_restart();
            sprintf(mBuffer, "TAKE_PIC_REQ_REPLY FAIL");
            len_of_send = strlen(mBuffer);
            sendLockedSocket(mBuffer, len_of_send);
            len_of_send = 0;
            camera_retry = 0;
            break;
        }

        fb = esp_camera_fb_get();
        if (!fb)
        {
            camera_retry++;
            ESP_LOGE(TAG, "Camera capture failed, camera_retry: %d", camera_retry);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            //m_esp_status = ESP_STATUS_CAMERA_FAILED;
            continue;
        }
        else
        {
            camera_retry = 0;
            gettimeofday(&tv_no1, NULL);
            //time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            ESP_LOGI(TAG, "esp_camera_fb_get time:%lld.%lld ", (int64_t)tv_no1.tv_sec, (int64_t)tv_no1.tv_usec);
            ESP_LOGE(TAG, "pic_width is %d pic_height is %d", fb->width, fb->height);
            ESP_LOGI(TAG, "Camera capture format=%d", fb->format);
            if (fb->format != PIXFORMAT_JPEG)
            {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted)
                {
                    ESP_LOGE(TAG, "JPEG compression failed");
                    //ret = -1;
                }
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        //  send pic to app
        // 1. send pic start
        sprintf(mBuffer, "TAKE_PIC_REQ_REPLY %d %lld.%lld", _jpg_buf_len, (int64_t)tv_now.tv_sec, (int64_t)tv_now.tv_usec);
        ESP_LOGI(TAG, "%s", mBuffer);

        pthread_mutex_lock(&mut);
        len_of_send = strlen(mBuffer);
        ret = sendUnlockedSocket(mBuffer, len_of_send);
        len_of_send = 0;

        if (ret <= 0)
        {
            ESP_LOGE(TAG, "fails to send TAKE_PIC_REQ_REPLY   error : %d (%s)", errno, strerror(errno));
            m_esp_status = ESP_STATUS_INIT;
            pthread_mutex_unlock(&mut);
            return ret;
        }
        else
        {
            int offset = 0;
            int sendDataSize = 10 * 1024;

            // 2. send pic
            ESP_LOGI(TAG, "start to send picture data");
            //ret = esp_send((char*)_jpg_buf, _jpg_buf_len);

            while (offset < _jpg_buf_len)
            {
                if (sendDataSize > (_jpg_buf_len - offset))
                    sendDataSize = (_jpg_buf_len - offset);
                ret = esp_send(((char *)_jpg_buf) + offset, sendDataSize);
                if (ret <= 0)
                    break;
                offset += ret;
            }
            ESP_LOGI(TAG, "jpg _buffer_ len is %d", _jpg_buf_len);

            if (ret <= 0)
            {
                ESP_LOGE(TAG, "fails to send picture data error : %d (%s)", errno, strerror(errno));
                m_esp_status = ESP_STATUS_INIT;
                pthread_mutex_unlock(&mut);
                return ret;
            }
        }
        pthread_mutex_unlock(&mut);

        gettimeofday(&tv_no1, NULL);
        ESP_LOGI(TAG, "send pic end time:%lld.%lld ", (int64_t)tv_no1.tv_sec, (int64_t)tv_no1.tv_usec);

        if (fb)
        {
            esp_camera_fb_return(fb);

            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        gettimeofday(&tv_no1, NULL);
        ESP_LOGI(TAG, "esp_camera_fb_return time:%lld.%lld ", (int64_t)tv_no1.tv_sec, (int64_t)tv_no1.tv_usec);
        break;
    } while (1);

    return ret;
}

// red led
static void redled_enable()
{
    gpio_set_level(33, 0);
}
static void redled_disable()
{
    gpio_set_level(33, 1);
}

// net init
static void net_init()
{
    int ip_protocol = 0;
    int itcp_retry = 0;
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(host_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    while (1)
    {
        if (m_sock < 0)
        {
            m_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
            if (m_sock < 0)
            {
                ESP_LOGE(TAG, "%s   Unable to create socket: errno %d", log_gettime(), errno);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI(TAG, "%s   Socket created(%d), connecting to %s:%d", log_gettime(), m_sock, host_ip, PORT);
        }

        itcp_retry++;
        m_tcp_status = connect(m_sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in));
        if (m_tcp_status != 0)
        {
            ESP_LOGE(TAG, "%s   fails to connect server( errno %d  ,%s),(m_tcp_status %d) and retry %d.", log_gettime(), errno, strerror(errno), m_tcp_status, itcp_retry);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(TAG, "%s   connect server ok", log_gettime());
            m_esp_status = ESP_STATUS_SOCKET_OK;
            redled_disable();
            break;
        }

        if (itcp_retry >= 3)
        {
            ESP_LOGE(TAG, "reset model");
            m_esp_status = ESP_STATUS_INIT;
            break;
        }
    }
    //ESP_LOGI(TAG, "Successfully connected");
}

static void net_uninit()
{
    if (m_sock != -1)
    {
        ESP_LOGE(TAG, "%s   Shutting down socket and restarting to build socket...", log_gettime());
        shutdown(m_sock, 0);
        close(m_sock);

        redled_enable();
        ESP_LOGI(TAG, "%s   red led light", log_gettime());
        m_esp_status = ESP_STATUS_INIT;
        m_sock = -1;
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}

// 设备初始化 init
static void dev_init()
{
    int len = 0;
    uint8_t sta_mac[6];
    char rx_buffer[ESP_BUFFER_LENGTH] = {0};
	int len_of_send = 0;
    while (1)
    {
        if (m_sock < 0)
        {
            ESP_LOGE(TAG, "m_sock < 0...\n");
            break;
        }
        ESP_LOGI(TAG, "fetch mac add.");
        if (ESP_OK != (esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac)))
        {
            ESP_LOGE(TAG, "esp_wifi_get_mac failed..\n");
            continue;
        }
        ESP_LOGI(TAG, "register camera");

        sprintf(mBuffer, "REG %02X:%02X:%02X:%02X:%02X:%02X 1280x720 15 0_128 0 %s", sta_mac[0], sta_mac[1], sta_mac[2], sta_mac[3], sta_mac[4], sta_mac[5], VERSION_NAME);
        len_of_send = strlen(mBuffer);
        len = sendLockedSocket(mBuffer, len_of_send);
        len_of_send = 0;

        if (len < 0)
        {
            ESP_LOGE(TAG, "%s   fails to register camera: errno %d(%s)", log_gettime(), errno, strerror(errno));
            continue;
        }
        ESP_LOGE(TAG, "%s   send reg success", log_gettime());
        len = recvSocket(rx_buffer, ESP_BUFFER_LENGTH - 1);

        ESP_LOGI(TAG, "%s   recv reg  and      .....", log_gettime());
        if (0 == strcmp("REG_REPLY SUCCESS", rx_buffer))
        {
            //ESP_LOGI(TAG, "reg_reply success...\n");
            m_esp_status = ESP_STATUS_REG_SUCCESS;
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Successfully init");

    // dev config led. default close.
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    ESP_LOGI(TAG, "Camera capture CONFIG_LED_ILLUMINATOR_ENABLED");
    enable_led(true);
    isStreaming = true;
#endif
}

static void dev_uninit()
{
    // close led
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = false;
    enable_led(false);
#endif
led_duty = 0;

}

static void dev_cmd(void)
{
    char rx_buffer[ESP_BUFFER_LENGTH] = {0};
    int len = 0;
	int len_of_send = 0;
    len = recvSocket(rx_buffer, ESP_BUFFER_LENGTH - 1);
    if (len <= 0)
    {
        ESP_LOGE(TAG, "fails to received data len :%d", len);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        m_esp_status = ESP_STATUS_INIT;
        return;
    }

    if (0 == strncmp("TAKE_PIC_REQ", rx_buffer, 12))
    {
        sendLockedSocket("HEART_BEAT", LEN_HEART_BEAT);
        stream_handler();
    }
    else if (0 == strncmp("LED_REQ ", rx_buffer, 8))
    {
        int value = atoi(rx_buffer + 8);
        ESP_LOGI(TAG, "%s   request led brightness : %d", log_gettime(), value);

        led_duty = value;
        if (isStreaming)
            enable_led(true);
        memset(mBuffer, 0, ESP_BUFFER_LENGTH);
        sprintf(mBuffer, "LED_REQ_REPLY SUCCESS");
        len_of_send = strlen(mBuffer);
        sendLockedSocket(mBuffer, len_of_send);
        len_of_send = 0;
    }
    else if (0 == strcmp("LED_STATE_REQ", rx_buffer))
    {
        // reply current led brightness
        memset(mBuffer, 0, ESP_BUFFER_LENGTH);
        sprintf(mBuffer, "LED_STATE_REQ_REPLY ");
        sprintf(mBuffer + strlen(mBuffer), "%d", led_duty);
        len_of_send = strlen(mBuffer);
        sendLockedSocket(mBuffer, len_of_send);
        len_of_send = 0;
    }
    else if (0 == strncmp("SHUTDOWN", rx_buffer, 8))
    {
        ESP_LOGE(TAG, "%s   shutdown..................", log_gettime());
        m_esp_status = ESP_STATUS_INIT;
    }
    else if (0 == strncmp("CHECK_LIVE_HEART_BEAT", rx_buffer, 15))
    {
        memset(mBuffer, 0, ESP_BUFFER_LENGTH);
        sprintf(mBuffer, "RESPOND_HEART_BEAT");

        sendLockedSocket(mBuffer, 18);
    }
    else if (0 == strncmp("HEART_BEAT", rx_buffer, 10))
    {
    }
    else
    {
       // ESP_LOGE(TAG, "%s   unknown command", log_gettime());
    }
}

static void tcp_client_task(void *pvParameters)
{
    while (1)
    {
        switch (m_esp_status)
        {
        case ESP_STATUS_INIT:
            dev_uninit();
            net_uninit();
            net_init();
            break;
        case ESP_STATUS_SOCKET_OK:
            //dev_uninit();
            dev_init();
            break;
        case ESP_STATUS_REG_SUCCESS:
            if (m_tcp_status == 0 && m_sock > 0)
            {
                dev_cmd();
            }
            else
            {
                ESP_LOGI(TAG, "m_tcp_status != 0 && m_sock =%d", m_sock);
                m_esp_status = ESP_STATUS_INIT;
            }
            break;
        case ESP_STATUS_CAMERA_FAILED:
            esp_restart();
            ESP_LOGI(TAG, "reset camera");
            m_esp_status = ESP_STATUS_REG_SUCCESS;
            // reset camera;
            break;
        default:

            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static int esp_send(char *data, int len)
{
    int err = -1;
    err = send(m_sock, data, len, 0);
    if (err <= 0)
    {
        ESP_LOGE(TAG, "%s   esp_send rtn = %d, %d(%s)", log_gettime(), err, errno, strerror(errno));
        m_esp_status = ESP_STATUS_INIT;
        m_sock = -1;
    }
    else
    {
        ESP_LOGI(TAG, "%s   esp_send data(%d): %s(%d)", log_gettime(), len, data, err);
    }

    return err;
}

static int sendLockedSocket(char *data, int len)
{
    int err = -1;

    if (NULL == data || len <= 0)
    {
        ESP_LOGI(TAG, "sendlockedSocket data is null or len = %d", len);
        return err;
    }

    pthread_mutex_lock(&mut);

    err = esp_send((char *)(&len), 2);
    if (err > 0)
        err = esp_send(data, len);

    pthread_mutex_unlock(&mut);
    return err;
}

static int sendUnlockedSocket(char *data, int len)
{
    int err = -1;

    if (NULL == data || len <= 0)
    {
        ESP_LOGI(TAG, "sendUnlockedSocket data is null or len = %d", len);
        return err;
    }

    //ESP_LOGI(TAG, "realy to  send data len = %d", len);
    err = esp_send((char *)(&len), 2);
    if (err > 0)
        err = esp_send(data, len);

    return err;
}

static int recvSocket(char *data, int length)
{
    int recv_len = 0;
    //int err = -1;
    int readLen = 0;
    int readTotalLen = 0;
    ESP_LOGI(TAG, "%s   recv task start", log_gettime());
    memset(data, 0, length);
    readLen = recv(m_sock, data, 2, 0);
    //ESP_LOGE(TAG, "%s   recv  .......start .......  1.........",log_gettime());
    if (readLen <= 0)
    {
        ESP_LOGE(TAG, "%s   recvSocket readlen  < =0  errno = %s", log_gettime(), strerror(errno));
        m_esp_status = ESP_STATUS_INIT;
        return readLen;
    }
    if (readLen < 2)
    {

        ESP_LOGE(TAG, "%s   recvSocket readlen  < 2 errno = %s", log_gettime(), strerror(errno));
        readLen = recv(m_sock, data + 1, 1, 0);
        if (readLen <= 0)
        {
            ESP_LOGE(TAG, "%s   recvSocket readlen  <= 0 errno = %s", log_gettime(), strerror(errno));
            m_esp_status = ESP_STATUS_INIT;
            return readLen;
        }
    }
    //ESP_LOGI(TAG, "%s   recv  .......start .......  2  .........", log_gettime());
    recv_len = (*(unsigned short *)data);
    //ESP_LOGE(TAG, "%s   recv  .......start .......  3       ......  .........",log_gettime());

    if (recv_len > length)
    {
        ESP_LOGE(TAG, "receive data too long");
        recv_len = length;
    }

    do
    {
        memset(data, 0, length);
        readLen = recv(m_sock, data + readTotalLen, recv_len - readTotalLen, 0);
        if (readLen <= 0)
        {
            ESP_LOGI(TAG, "exception to receive data");
            m_esp_status = ESP_STATUS_INIT;
            return readLen;
        }
        //ESP_LOGE(TAG, "%s   recv  .......recv data............	   ......  .........",log_gettime());

        readTotalLen += readLen;

    } while (readTotalLen < recv_len);

    ESP_LOGI(TAG, "%s   received data : %s", log_gettime(), data);
    return readTotalLen;
}

/*
static uint8_t TX_CheckSum(uint8_t *buf, uint8_t len) 
{ 
    uint8_t i, ret = 0;
 
    for(i=0; i<len; i++)
    {
        ret += *(buf++);
    }
     ret = ~ret;
    return ret;
}
*/
void app_client_main(void)
{
    ESP_LOGE(TAG, "app_client_main start tast tcpclient............");

    // init led
    gpio_pad_select_gpio(33);
    gpio_set_direction(33, GPIO_MODE_OUTPUT);

    redled_enable();
    esp_initialize_sntp();
    //createHeartBeatThread();
    first_takepic();

    esp_timer_create(&esp_heart_arg, &esp_heart_handle);
    esp_timer_start_periodic(esp_heart_handle, 1000 * 1000);

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}
