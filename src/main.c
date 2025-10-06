#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <gpio.h>
#include <stdio.h>

#include <esp_system.h>
#include <http_server.h>
#include <wifi.h>
#include <log.h>

#include <i2c_master.h>
#include <math.h>


// Required by old ESP8266 RTOS SDK (for RF calibration storage)
uint32 user_rf_cal_sector_set(void) {
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;
        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;
        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

#define HMC5883L_ADDR 0x1E
#define MPU_6050_ADDR 0x68
// MPU-6050 Registers
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B
#define ACCEL_XOUT_H    0x3B
void i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    i2c_master_start();
    i2c_master_writeByte((dev_addr << 1) | 0);  // Write mode
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return;
    }
    i2c_master_writeByte(reg);
    i2c_master_checkAck();
    i2c_master_writeByte(value);
    i2c_master_checkAck();
    i2c_master_stop();
}

void i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_master_start();
    i2c_master_writeByte((dev_addr << 1) | 0);  // Write mode
    i2c_master_checkAck();
    i2c_master_writeByte(reg);
    i2c_master_checkAck();

    // Restart condition
    i2c_master_start();
    i2c_master_writeByte((dev_addr << 1) | 1);  // Read mode
    i2c_master_checkAck();

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_master_readByte();
        if (i < (len - 1)) i2c_master_send_ack();
        else i2c_master_send_nack();
    }
    i2c_master_stop();
}

// --- HMC5883L helper ---
void hmc5883l_init(void)
{
    // CRA: 8-average, 15Hz, normal
    i2c_write_reg(HMC5883L_ADDR, 0x00, 0x70);
    // CRB: Gain = 5 (default)
    i2c_write_reg(HMC5883L_ADDR, 0x01, 0xA0);
    // Mode: continuous measurement
    i2c_write_reg(HMC5883L_ADDR, 0x02, 0x00);
}

void MPU_6050_init(void)
{
    // Wake up the MPU6050 (clear sleep bit)
    i2c_write_reg(MPU_6050_ADDR, PWR_MGMT_1, 0x00);

    // Set accelerometer full-scale range = ±2g (00)
    i2c_write_reg(MPU_6050_ADDR, ACCEL_CONFIG, 0x00);

    // Set gyroscope full-scale range = ±250 °/s (00)
    i2c_write_reg(MPU_6050_ADDR, GYRO_CONFIG, 0x00);
}

void MPU_6050_read(float *acc, float *gyro)
{
    uint8_t buf[14];
    int16_t ax, ay, az, gx, gy, gz;

    // Read 14 bytes: accel (6), temp (2), gyro (6)
    i2c_read_reg(MPU_6050_ADDR, ACCEL_XOUT_H, buf, 14);

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);

    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to physical units
    // Sensitivity scale factors (datasheet):
    // Accel ±2g: 16384 LSB/g
    // Gyro ±250°/s: 131 LSB/(°/s)

    acc[0] = (float)ax / 16384.0f;   // g
    acc[1] = (float)ay / 16384.0f;   // g
    acc[2] = (float)az / 16384.0f;   // g

    gyro[0] = (float)gx / 131.0f;    // deg/s
    gyro[1] = (float)gy / 131.0f;    // deg/s
    gyro[2] = (float)gz / 131.0f;    // deg/s
}


void hmc5883l_read(float *mag)
{
    uint8_t data[6];
    i2c_read_reg(HMC5883L_ADDR, 0x03, data, 6);

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_z = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_y = (int16_t)((data[4] << 8) | data[5]);

    mag[0] = (float)raw_x;
    mag[1] = (float)raw_y;
    mag[2] = (float)raw_z;
}


typedef struct Sense_data_t {
    float mag[3];
    float acc[3];
    float gyro[3];
}Sense_data_t;

xQueueHandle Sensor_Data_queue;
// producer task
void Sensor_Task(void *pvParameters)
{
    char TAG[] = "Sense Task";
    i2c_master_gpio_init();
    i2c_master_init();
    hmc5883l_init();
    MPU_6050_init();
    Sense_data_t data_packet;
    while (1) {
        hmc5883l_read(data_packet.mag);
        MPU_6050_read(data_packet.acc, data_packet.gyro);
        xQueueSend(Sensor_Data_queue, &data_packet, 10);
        vTaskDelay(10/portTICK_RATE_MS);
    }
}


void print_data(Sense_data_t *data_packet)
{
    char buf[50];
    sprintf(buf, "mag   => x = %.2f, y = %.2f, z = %.2f", data_packet->mag[0], data_packet->mag[1], data_packet->mag[2]);
    printf("%s\n", buf);
    sprintf(buf, "acc   => x = %.2f, y = %.2f, z = %.2f", data_packet->acc[0], data_packet->acc[1], data_packet->acc[2]);
    printf("%s\n", buf);
    sprintf(buf, "gyro  => x = %.2f, y = %.2f, z = %.2f", data_packet->gyro[0], data_packet->gyro[1], data_packet->gyro[2]);
    printf("%s\n", buf);
}

#define ALPHA 0.2
void Filter(Sense_data_t *in, Sense_data_t *out)
{
    out->acc[0] = ALPHA * in->acc[0] + (1 - ALPHA) * out->acc[0];
    out->acc[1] = ALPHA * in->acc[1] + (1 - ALPHA) * out->acc[1];
    out->acc[2] = ALPHA * in->acc[2] + (1 - ALPHA) * out->acc[2];

    out->gyro[0] = ALPHA * in->gyro[0] + (1 - ALPHA) * out->gyro[0];
    out->gyro[1] = ALPHA * in->gyro[1] + (1 - ALPHA) * out->gyro[1];
    out->gyro[2] = ALPHA * in->gyro[2] + (1 - ALPHA) * out->gyro[2];

    out->mag[0] = ALPHA * in->mag[0] + (1 - ALPHA) * out->mag[0];
    out->mag[1] = ALPHA * in->mag[1] + (1 - ALPHA) * out->mag[1];
    out->mag[2] = ALPHA * in->mag[2] + (1 - ALPHA) * out->mag[2];
}

#define COMPLEMENTARY_ALPHA 0.98f
#define DT 0.01f
#define RAD_TO_DEG (180.0f / M_PI)

void Complementary_filter(Sense_data_t *in, float *pitch, float *roll)
{
    // Calculate pitch/roll from accelerometer (gravity reference)
    float accel_pitch = atan2f(in->acc[1], sqrtf(in->acc[0]*in->acc[0] + in->acc[2]*in->acc[2])) * RAD_TO_DEG;
    float accel_roll  = atan2f(-in->acc[0], in->acc[2]) * RAD_TO_DEG;

    // Fuse gyro (fast response, drift) with accel (slow, stable)
    *pitch = COMPLEMENTARY_ALPHA * (*pitch + in->gyro[0] * DT) + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
    *roll  = COMPLEMENTARY_ALPHA * (*roll  + in->gyro[1] * DT) + (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
}

#define SHAKE_THRESHOLD   20.0f   // quick left/right change in roll
#define NOD_THRESHOLD     20.0f   // quick up/down change in pitch
#define TILT_THRESHOLD    30.0f   // sustained tilt angle

typedef enum {
    GESTURE_NONE,
    GESTURE_TILT_LEFT,
    GESTURE_TILT_RIGHT,
    GESTURE_TILT_FORWARD,
    GESTURE_TILT_BACKWARD,
    GESTURE_SHAKE,
    GESTURE_NOD
} Gesture_t;

Gesture_t Gesture_sense(float pitch, float roll, float prev_pitch, float prev_roll) {
    float dt_roll  = roll  - prev_roll;
    float dt_pitch = pitch - prev_pitch;

    // --- Dynamic Gestures (changes) ---
    if (fabsf(dt_roll) > SHAKE_THRESHOLD) {
        return GESTURE_SHAKE;
    }
    if (fabsf(dt_pitch) > NOD_THRESHOLD) {
        return GESTURE_NOD;
    }

    // --- Static Gestures (absolute position) ---
    if (roll > TILT_THRESHOLD) return GESTURE_TILT_RIGHT;
    if (roll < -TILT_THRESHOLD) return GESTURE_TILT_LEFT;
    if (pitch > TILT_THRESHOLD) return GESTURE_TILT_FORWARD;
    if (pitch < -TILT_THRESHOLD) return GESTURE_TILT_BACKWARD;

    return GESTURE_NONE;
}

xQueueHandle Guesture_queue;
void Motion_Sense_Task(void *pvParams)
{
    Sense_data_t new_data_packet;
    Sense_data_t filter_data_packet = {0}; // must zero init for first filter step
    uint8_t init = 0;
    float pitch = 0.0f, roll = 0.0f, prev_pitch = 0.0f, prev_roll = 0.0f;
    Gesture_t g,prev_g;
    while (1)
    {
        if (xQueueReceive(Sensor_Data_queue, &new_data_packet, 10) == pdTRUE)
        {
            // Apply low-pass filter
            Filter(&new_data_packet, &filter_data_packet);

            if (!init) {
                // Initialize pitch/roll using accel (first step only)
                pitch = atan2f(filter_data_packet.acc[1],
                               sqrtf(filter_data_packet.acc[0]*filter_data_packet.acc[0] +
                                     filter_data_packet.acc[2]*filter_data_packet.acc[2])) * RAD_TO_DEG;

                roll  = atan2f(-filter_data_packet.acc[0],
                               filter_data_packet.acc[2]) * RAD_TO_DEG;
                prev_roll = roll;
                prev_pitch = pitch;
                init = 1;
            } else {
                // Run complementary filter
                Complementary_filter(&filter_data_packet, &pitch, &roll);
                
                g = Gesture_sense(pitch, roll, prev_pitch, prev_roll);
                if(prev_g != g)
                {
                    xQueueSend(Guesture_queue, &g, 10);
                    if(g == GESTURE_TILT_LEFT)
                    {
                        printf("LEFT\n");

                    }else if(g == GESTURE_TILT_RIGHT)
                    {
                        printf("RIGHT\n");
                    }else if(g == GESTURE_TILT_FORWARD)
                    {
                        printf("FORWARD\n");
                    }else if(g == GESTURE_TILT_BACKWARD)
                    {
                        printf("BACKWARD\n");
                    }else if(g == GESTURE_SHAKE)
                    {
                        printf("SHAKE\n");
                    }else if(g == GESTURE_NOD)
                    {
                        printf("NOD\n");
                    }
                prev_g = g;
                }
                
                prev_pitch = pitch;
                prev_roll = roll;
            }
            /*
            char buf[30]; 
            sprintf(buf, "roll = %.2f, pitch = %.2f", roll, pitch); 
            printf("%s\n", buf);
            */
        }
        vTaskDelay(10/portTICK_RATE_MS);
    }
}



void guesture_handler(char *http_buffer, char *post_data)
{
    uint8_t g;
    if(xQueueReceive(Guesture_queue, &g, 10) == pdTRUE)
    {
        switch (g)
        {
        case GESTURE_TILT_LEFT:
            sprintf(http_buffer, "LEFT");
            break;
        case GESTURE_TILT_RIGHT:
            sprintf(http_buffer, "RIGHT");
            break;
        case GESTURE_TILT_BACKWARD:
            sprintf(http_buffer, "BACKWARD");
            break;
        case GESTURE_TILT_FORWARD:
            sprintf(http_buffer, "FORWARD");
            break;
        
        default:
            break;
        }
    }
}
http_handler_t guesture_handle = 
{
    .handler = guesture_handler,
    .method = GET,
    .path = "/gesture"
};

void user_init()
{   
    Sensor_Data_queue = xQueueCreate(10, sizeof(Sense_data_t));
    Guesture_queue = xQueueCreate(10, sizeof(uint8_t));
    http_handler_register(&guesture_handle);
    wifi_setup();
    Web_server_init();
    xTaskCreate(Sensor_Task, "sensetask", 512, NULL, 5, NULL);
    xTaskCreate(Motion_Sense_Task, "motiontask", 512, NULL, 5, NULL);
    
}

