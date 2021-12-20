/*
  * ESPRESSIF MIT License
  *
  * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
  *
  * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
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
  *
  */
#ifndef _APP_CLIENT_H_
#define _APP_CLIENT_H_

#ifdef __cplusplus
extern "C"
{
#endif

  void app_wifi_main();
  void ota_update();
  char *log_gettime();
  void app_client_main();
  void createHeartBeatThread();
#ifndef ESP_STATUS_
#define ESP_STATUS_
  enum esp_status
  {
    ESP_STATUS_INIT,
    ESP_STATUS_NET_OK,
    ESP_STATUS_CAMERA_OK,
    ESP_STATUS_SOCKET_OK,
    ESP_STATUS_REG_SUCCESS,
    ESP_STATUS_CAMERA_FAILED,
    ESP_OTA_START
  };
#endif

#ifndef VERSION_NAME_
#define VERSION_NAME_
#define VERSION_NAME "Foot3Dv1.0.2021122017"

#endif

#ifdef __cplusplus
}
#endif

#endif
