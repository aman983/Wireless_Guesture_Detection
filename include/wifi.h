
#ifndef WIFI_H
#define WIFI_H

#include <freertos/semphr.h>

#define SSID "Sentian"
#define PASS "S0h@1234567890"

xSemaphoreHandle got_ip_sem;

void wifi_setup();
void wifi_task(void *pvParameters);

#endif



