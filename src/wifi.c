#include <lwip/netif.h>
#include <lwip/ip_addr.h>
#include <lwip/inet.h>

#include <esp_wifi.h>
#include <esp_sta.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <log.h>
#include <wifi.h>

extern xSemaphoreHandle got_ip_sem;

static void wifi_evt_handler(System_Event_t *evt)
{
    uint8_t TAG[] = "WiFi Event";
    switch (evt->event_id)
    {
    case EVENT_STAMODE_CONNECTED:
            LOG(TAG, "[WiFi Event] Connected to %s", evt->event_info.connected.ssid);
        break;
    case EVENT_STAMODE_DISCONNECTED:
            LOG(TAG,"[Wifi Event] Disconnected");
        break;
    case EVENT_STAMODE_GOT_IP:
    {
            uint8_t *ip = (uint8_t *) &evt->event_info.got_ip.ip.addr;
            LOG(TAG,"[Wifi Event] Ip assigned -> %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            xSemaphoreGive(got_ip_sem);
            xSemaphoreGive(got_ip_sem);
    }break;
    default:
        break;
    }
}

void wifi_task(void *pvParameters) {
    while(1) {
        // This delay is crucial for the Wi-Fi stack's event loop to run.
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void wifi_setup()
{
    xTaskHandle wifi_t;
    got_ip_sem = xSemaphoreCreateCounting(2,0);
    struct station_config cfg;
    memset(&cfg, 0, sizeof(cfg));
    strcpy((char *)cfg.ssid, SSID);
    strcpy((char *)cfg.password, PASS);
    
    wifi_set_opmode(STATION_MODE);
    wifi_station_set_config(&cfg);
    wifi_set_event_handler_cb(wifi_evt_handler);
    wifi_station_connect();
    xTaskCreate(wifi_task, "wifi_task", 1024, NULL, 7, &wifi_t);

}
