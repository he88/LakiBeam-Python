#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

#include "rb_lidar.h"

#define BUFFER_SIZE 1024

// UDP reception thread
void *udp_listener(void *arg)
{
	RBLidar *lidar = (RBLidar *)arg;
	int sockfd;
	struct sockaddr_in server_addr, client_addr;
	socklen_t addr_len = sizeof(client_addr);
	udp_packet_t udp_packet_temp;
	uint32_t timestamp_last = 0;
	uint32_t timestamp_div = 1;

	// Create UDP socket
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0)
	{
		perror("socket");
		return NULL;
	}

	// Set up server address
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	inet_pton(AF_INET, lidar->ip, &server_addr.sin_addr);
	server_addr.sin_port = htons(lidar->port);

	// Bind the socket
	if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		perror("bind");
		close(sockfd);
		return NULL;
	}

	printf("Listening for UDP packets on %s:%d...\n", lidar->ip, lidar->port);

	// Receive data
	while (1)
	{
		int len = recvfrom(sockfd, (char *)&udp_packet_temp, UDP_BUF_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
		if (len < 0)
		{
			perror("recvfrom");
			continue;
		}

		// Use double-buffering mechanism
		pthread_mutex_lock(&lidar->mutex);

		uint32_t timestamp_current = udp_packet_temp.timestamp;

		if(timestamp_current == 0)
		{
			timestamp_current = timestamp_last;
		}
		else
		{
			timestamp_div = (timestamp_current - timestamp_last) / CONFIG_UDP_BLOCKS;
			timestamp_last = timestamp_current;
		}

		// Store received UDP sub-packets into the current buffer
		for (int i = 0; i < CONFIG_UDP_BLOCKS; i++)
		{
			sub_packet_t *sub_packet = &udp_packet_temp.sub_packet[i];

			// Check if azimuth is not 0
			if (sub_packet->azimuth != 0xffff)
			{
				// Store sub-packet into the current buffer
				memcpy(&lidar->buffer[lidar->current][lidar->valid_count[lidar->current]], sub_packet, sizeof(sub_packet_t));
				lidar->timestamps[lidar->current][lidar->valid_count[lidar->current]] = timestamp_current + i * timestamp_div;

				// Count valid data
				lidar->valid_count[lidar->current]++;
			}

			// Check if azimuth is 0
			if (sub_packet->azimuth == 0)
			{
				// Switch to the next buffer and notify processing thread
				lidar->current = (lidar->current + 1) % 2; // Switch buffer

				lidar->valid_count[lidar->current] = 0;
				memcpy(&lidar->buffer[lidar->current][lidar->valid_count[lidar->current]], sub_packet, sizeof(sub_packet_t));
				lidar->timestamps[lidar->current][lidar->valid_count[lidar->current]] = timestamp_current + i * timestamp_div;
				lidar->valid_count[lidar->current]++;

				pthread_cond_signal(&lidar->cond); // Notify processing thread
			}
		}

		pthread_mutex_unlock(&lidar->mutex);
	}

	close(sockfd);
	return NULL;
}

// Packet processing thread
void *packet_processor(void *arg)
{
	RBLidar *lidar = (RBLidar *)arg;

	// Process data
	while (1)
	{
		pthread_mutex_lock(&lidar->mutex);
		pthread_cond_wait(&lidar->cond, &lidar->mutex); // Wait for data

		// Process data in the current buffer
		int next_index = (lidar->current + 1) % 2; // Get previous buffer index
		int count = lidar->valid_count[next_index]; // Get valid data count

		// Create temporary buffer for point data
		point_data_t temp_buffer[BUFFER_SIZE * CONFIG_BLOCK_COUNT]; // Assume BUFFER_SIZE is sufficient
		int temp_count = 0;

		// Process each valid sub-packet in the current buffer
		for (int i = 0; i < count - 1; i++)
		{
			sub_packet_t *sub_packet = &lidar->buffer[next_index][i];

			int start_angle = sub_packet->azimuth % 36000;
			int end_angle = lidar->buffer[next_index][i + 1].azimuth % 36000;

			// Compute angle increment
			int div_angle;
			if (end_angle >= start_angle)
			{
				div_angle = (end_angle - start_angle) / CONFIG_BLOCK_COUNT;
			}
			else
			{
				div_angle = (36000 - start_angle + end_angle) / CONFIG_BLOCK_COUNT;
			}

			// Compute time interval
			uint32_t start_time = lidar->timestamps[next_index][i];
			uint32_t end_time = lidar->timestamps[next_index][i + 1];
			uint32_t div_time = (end_time - start_time) / CONFIG_BLOCK_COUNT;

			// Fill point data
			for (int j = 0; j < CONFIG_BLOCK_COUNT; j++) {
				point_data_t point_data;
				point_data.azimuth = (start_angle + j * div_angle) % 36000;
				point_data.dist = sub_packet->point[j].dist_0;
				point_data.rssi = sub_packet->point[j].rssi_0;
				point_data.timestamp = start_time + j * div_time;
				temp_buffer[temp_count++] = point_data;
			}
		}

		// Invoke callback function
		lidar->callback((void*)temp_buffer, sizeof(point_data_t) * temp_count);

		pthread_mutex_unlock(&lidar->mutex);
	}
	return NULL;
}

// Create RBLidar instance
RBLidar *rblidar_create(const char *ip, int port, callback_t callback)
{
	RBLidar *lidar = (RBLidar *)malloc(sizeof(RBLidar));
	lidar->ip = strdup(ip);
	lidar->port = port;
	lidar->callback = callback;

	lidar->current = 0;
	pthread_mutex_init(&lidar->mutex, NULL);
	pthread_cond_init(&lidar->cond, NULL);

	// Create reception thread
	pthread_t listener_thread;
	pthread_create(&listener_thread, NULL, udp_listener, lidar);
	pthread_detach(listener_thread);

	// Create packet processing thread
	pthread_t processor_thread;
	pthread_create(&processor_thread, NULL, packet_processor, lidar);
	pthread_detach(processor_thread);

	return lidar;
}

// Free RBLidar instance
void rblidar_destroy(RBLidar *lidar)
{
	pthread_mutex_destroy(&lidar->mutex);
	pthread_cond_destroy(&lidar->cond);
	free(lidar->ip);
	free(lidar);
}
