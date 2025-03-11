#ifndef RB_LIDAR_H
#define RB_LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  // Include standard integer types
#include <pthread.h> // Include thread support

typedef void (*callback_t)(const char* data, int length);

// Configuration constants
#define CONFIG_BLOCK_COUNT    (16)   
#define CONFIG_UDP_BLOCKS     (12)   
#define FRAME_POINT_COUNT     (3600) 
#define CONFIG_DEGREE_MAX  (36000)
#define SUBPKT_BUF_COUNT    (300)   

#define UDP_BUF_SIZE        (1206)

// Structure alignment settings
#pragma pack(push)
#pragma pack(1)

// Raw point data structure
typedef struct {
    uint16_t dist_0; // First distance measurement
    uint8_t rssi_0;  // First RSSI value
    uint16_t dist_1; // Second distance measurement
    uint8_t rssi_1;  // Second RSSI value
} raw_point_t;

// Sub-packet structure
typedef struct {
    uint16_t header; // Sub-packet header
    uint16_t azimuth; // Azimuth angle
    raw_point_t point[CONFIG_BLOCK_COUNT]; // Array of point data
} sub_packet_t;

// UDP packet structure
typedef struct {
    sub_packet_t sub_packet[CONFIG_UDP_BLOCKS]; // Array of sub-packets
    uint32_t timestamp; // Timestamp
    uint16_t factory_byte; // Factory byte
} udp_packet_t;

// Point data structure
typedef struct {
    uint16_t azimuth; // Azimuth angle
    uint16_t dist; // Distance measurement
    uint16_t rssi; // RSSI value
    uint32_t timestamp; // Timestamp
} point_data_t;

// Frame data structure
typedef struct {
    point_data_t pointcloud[FRAME_POINT_COUNT]; // Depth data point cloud
} frame_data_t;

#pragma pack(pop)

// RBLidar structure
typedef struct {
    char* ip;
    int port;
    callback_t callback;
    sub_packet_t buffer[2][SUBPKT_BUF_COUNT]; // Double buffer
    uint32_t timestamps[2][SUBPKT_BUF_COUNT]; // Array to store timestamps
    int current;     // Current buffer index
    int valid_count[2]; // Number of valid data entries in each buffer
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} RBLidar;

// Create RBLidar instance
RBLidar* rblidar_create(const char* ip, int port, callback_t callback);

// Destroy RBLidar instance
void rblidar_destroy(RBLidar* lidar);

#ifdef __cplusplus
}
#endif

#endif // RB_LIDAR_H
