
#include <lwip/netif.h>
#include <lwip/sockets.h>
#include <lwip/api.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "http_server.h"
#include <log.h>
extern xSemaphoreHandle got_ip_sem;

const char *http_index_page =
"<!DOCTYPE html><html><head><title>ESP8266 Gesture Monitor</title>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<style>"
"body { font-family: Arial, sans-serif; text-align:center; padding:30px; background:#f5f5f5; margin:0; }"
"h1 { color:#333; }"
".gesture-container { display:flex; flex-direction: column; align-items:center; gap:20px; margin-top:50px; }"
".gesture-box { padding: 20px 40px; font-size: 24px; border: 2px solid #ccc; border-radius: 10px; background:#fff; }"
"</style>"
"<script>"
"function updateGesture(){"
"  fetch('/gesture').then(r=>r.text()).then(gesture=>{"
"    document.getElementById('gestureDisplay').innerText = gesture;"
"  });"
"}"
"setInterval(updateGesture,500);" // update every 500 ms
"window.onload = updateGesture;"
"</script></head><body>"
"<h1>ESP8266 Gesture Monitor</h1>"
"<div class=\"gesture-container\">"
"<div id='gestureDisplay' class='gesture-box'>No Gesture</div>"
"</div>"
"</body></html>";




const char *not_found_response =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Connection: close\r\n\r\n"
            "OFF";

#define HANDLERS_LIMIT 10
http_handler_t* handlers_array[HANDLERS_LIMIT];
uint8_t handler_id = 0;

char request_buffer[1024];
char HTTP_Transmit_buffer[256];

static void http_handler(char *path, char *method, char *Post_data, int client_sock)
{
    method_type m_t = (strcmp(method, "GET") == 0) ? GET : POST;
    for(uint8_t i=0; i<handler_id; i++)
    {
        // root handler
        if((strcmp(path, "/") == 0))
        {
            write(client_sock, http_index_page, strlen(http_index_page));
            return;
        }
        else if((strcmp(path, handlers_array[i]->path) == 0) && (m_t == handlers_array[i]->method))
        {
            handlers_array[i]->handler(HTTP_Transmit_buffer, Post_data);
            write(client_sock, HTTP_Transmit_buffer, strlen(HTTP_Transmit_buffer));
            return;
        }
    }
    write(client_sock, not_found_response, strlen(not_found_response));
}
#define MAX_POST_DATA 25
static void parse_http_request(const char *request, char *method, char *path, char *Post_data, size_t max_len) {
    // Find the first space to get the method
    char *first_space = strchr(request, ' ');
    if (first_space) {
        size_t method_len = first_space - request;
        if (method_len < max_len) {
            strncpy(method, request, method_len);
            method[method_len] = '\0';
        }

        // Find the second space to get the path
        char *second_space = strchr(first_space + 1, ' ');
        if (second_space) {
            size_t path_len = second_space - (first_space + 1);
            if (path_len < max_len) {
                strncpy(path, first_space + 1, path_len);
                path[path_len] = '\0';
            }
        }
    }
    if(strcmp(method, "POST") == 0)
    {
        char *body = strstr(request, "\r\n\r\n");
        if (body) {
            body += 4; // skip \r\n\r\n
            snprintf(Post_data, MAX_POST_DATA, "%s", body); // store body safely
        }
    }
}

static void web_server_task(void *pv_parameter)
{
    uint8_t TAG[] = "WEB_SERVER";

    xSemaphoreTake(got_ip_sem, portMAX_DELAY);
    int listen_sock, client_sock;

    struct sockaddr_in serv_addr, client_addr;
    socklen_t client_len;

    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if(listen_sock < 0)
    {
        LOG(TAG, "Error Could not create socket");
        vTaskDelete(NULL);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(80);

    if(bind(listen_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        LOG(TAG, "Error Could not bind the socket");
        close(listen_sock);
        vTaskDelete(NULL);
    }
    if(listen(listen_sock, 5) < 0)
    {
        LOG(TAG, "Error: Could not listen on socket");
        close(listen_sock);
        vTaskDelete(NULL);
    }
    LOG(TAG, "Server up");
    char method[10];
    char path[15];
    char Post_Data[MAX_POST_DATA];
    while (1)
    {
        client_len = sizeof(client_addr);
        client_sock = accept(listen_sock, (struct sockaddr *) &client_addr, &client_len);
        if(client_sock < 0)
        {
            LOG(TAG, "Error accept() fail");
            vTaskDelay(100/portTICK_RATE_MS);
            continue;
        }
        int recieved_bytes = read(client_sock, request_buffer, sizeof(request_buffer));

        if(recieved_bytes > 0)
        {
            request_buffer[recieved_bytes] = 0;
            //LOG(TAG, "Recieved request %s",request_buffer);
            parse_http_request(request_buffer, method, path, Post_Data, sizeof(path));
            http_handler(path, method, Post_Data, client_sock); 
            close(client_sock);
        }
    }
}

uint8_t http_handler_register(http_handler_t *handle)
{
    uint8_t TAG[] = "HANDLER REGISTER";
    
    if(handler_id > HANDLERS_LIMIT)
    {
        LOG(TAG, " ERROR handler limit full");
        return 0;
    }else{
        handlers_array[handler_id] = handle;
        handler_id++;
    }
    return 1;
}

void Web_server_init()
{
    xTaskHandle web_server;
    xTaskCreate(web_server_task, "web_server", 2048, NULL, 4, &web_server);
}