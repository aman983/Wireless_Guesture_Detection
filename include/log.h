#ifndef LOG_H
#define LOG_H
#define LOG(TAG, fmt, ...) printf("[%s] " fmt "\n", TAG, ##__VA_ARGS__)
#endif