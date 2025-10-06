
#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

typedef void (*http_handler_fn)(char *http_buffer, char *Post_data);

typedef enum method_type {
    GET,
    POST
}method_type;

typedef struct http_handler_t {
    http_handler_fn handler;
    method_type method;
    char *path; 
}http_handler_t;


void Web_server_init();
uint8_t http_handler_register(http_handler_t *handle);
#endif