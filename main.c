/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */

#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <common/mavlink.h>
#include <sys/time.h>
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
uint64_t microsSinceEpoch();
void parse_recieved(uint8_t* recieved, ssize_t rec_size,  uint8_t chan, int sock);
int mission_flag = 1;
int sent_flag = 0;

int main(int argc, char *argv[]) {
    char help[] = "--help";
    struct timeval start, stop;
    gettimeofday(&start, NULL);
    char target_ip[100];
    int j, mission_item_count;
    float position[6] = {};
    int sock_server = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in gcAddr;
    struct sockaddr_in locAddr;
    //struct sockaddr_in fromAddr;
    uint8_t *buf = malloc(BUFFER_LENGTH);
    ssize_t recsize;
    socklen_t fromlen = sizeof(locAddr);
    int bytes_sent;
    mavlink_message_t msg;
    uint16_t len;
    int i = 0;
    //int success = 0;
    unsigned int temp = 0;

    // Check if --help flag was used
    if ((argc == 2) && (strcmp(argv[1], help) == 0)) {
        printf("\n");
        printf("\tUsage:\n\n");
        printf("\t");
        printf("%s", argv[0]);
        printf(" <ip address of QGroundControl>\n");
        printf("\tDefault for localhost: udp-server 127.0.0.1\n\n");
        exit(EXIT_FAILURE);
    }

    // Change the target ip if parameter was given
    strcpy(target_ip, "127.0.0.1");
    if (argc == 2) {
        strcpy(target_ip, argv[1]);
    }

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(4250);

    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
    if (-1 == bind(sock_server, (struct sockaddr *) &locAddr, sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(sock_server);
        exit(EXIT_FAILURE);
    }
    listen(sock_server, 100);
    int sock = accept(sock_server, (struct sockaddr*)&locAddr, (socklen_t*)&fromlen);

    /* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
#else
    if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
#endif

    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(14550);


    for (;;) {
        /*Send Heartbeat */
        mavlink_msg_heartbeat_pack(254, 1, &msg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                                   MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        write(sock, buf, sizeof(buf));
        /* Send Status */
        mavlink_msg_sys_status_pack(254, 1, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        write(sock, buf, sizeof(buf));
        /*Send Mission*/
        gettimeofday(&stop, NULL);
        //4 Min Setup time
        if(stop.tv_sec > (start.tv_sec + 30) && mission_flag){
            printf("Begining Mission Transmission\n");
            //Clear Waypoints
            //TODO just set everything to zero why not
            mavlink_msg_mission_clear_all_pack(254, 1, &msg, 1, 0, MAV_MISSION_TYPE_MISSION);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            write(sock, buf, BUFFER_LENGTH);
            //Send WPs
            //Start Mission Item Transfer with Count
            mavlink_msg_mission_count_pack(254, 1, &msg, 1, 0, 2, MAV_MISSION_TYPE_MISSION);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            write(sock, buf, BUFFER_LENGTH);
            //Arm
            //Mode Auto
            //Genuine Dummy shit hours here on parameter custom mode
            //mavlink_msg_set_mode_pack(254, 1, &msg, 1, 1, 10);
            //len = mavlink_msg_to_send_buffer(buf, &msg);
            //write(sock, buf, sizeof(buf));
            gettimeofday(&start, NULL);
        }

        memset(buf, 0, BUFFER_LENGTH);
        recsize = read(sock, buf, BUFFER_LENGTH);
        parse_recieved(buf, recsize, MAVLINK_COMM_0, sock);
        //parse_recieved(buf, recsize, MAVLINK_COMM_1, sock);
        //parse_recieved(buf, recsize, MAVLINK_COMM_2, sock);
        //parse_recieved(buf, recsize, MAVLINK_COMM_3, sock);
        memset(buf, 0, BUFFER_LENGTH);
        usleep(500);
    }
}

void parse_recieved(uint8_t* recieved, ssize_t rec_size, uint8_t chan, int sock){
    int i;
    mavlink_message_t msg;
    mavlink_status_t status;
    mavlink_mission_item_t mission_item;
    mavlink_mission_request_t mission_request;
    mavlink_mission_ack_t ack;
    int len;
    uint8_t send_buf[BUFFER_LENGTH];
    for (i = 0; i < rec_size; ++i) {
        //If CRC pass
        if (mavlink_parse_char(chan, recieved[i], &msg, &status)) {
            // Packet received
            if(msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST && !sent_flag){
                memset(send_buf, 0, BUFFER_LENGTH);
                mavlink_msg_mission_request_decode(&msg, &mission_request);
                printf("Sending WP %d\n", mission_request.seq);
                fflush(stdout);
                mavlink_msg_mission_item_pack(254, 1, &msg, 1, 0, mission_request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.3626368 + mission_request.seq, 149.1651584, 583.49, MAV_MISSION_TYPE_MISSION);
                len = mavlink_msg_to_send_buffer(send_buf, &msg);
                write(sock, send_buf, BUFFER_LENGTH);
            }
            else if(msg.msgid == MAVLINK_MSG_ID_MISSION_ACK){
                mavlink_msg_mission_ack_decode(&msg, &ack);
                printf("ACK Result: %d\n", ack.type);
                //If successful Tranmsmission Start mission
                if(ack.type == 0 && !mission_flag){
                    //mavlink_msg_set_mode_pack(254, 1, &msg, 1, 1, 10);
                    //len = mavlink_msg_to_send_buffer(send_buf, &msg);
                    //write(sock, send_buf, BUFFER_LENGTH);
                }
                if(sent_flag){
                    mission_flag = 0;
                }
            }
            else if(msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM){
                mavlink_msg_mission_item_decode(&msg, &mission_item);
                printf("FRAME: %d, x: %f, y: %f, z: %f\n", mission_item.frame, mission_item.x, mission_item.y, mission_item.z);
            }
            else{

            }
        }
    }
}


/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{

    struct timespec time;

    uint64_t micros = 0;

    clock_gettime(CLOCK_REALTIME, &time);
    micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;

    return micros;
}
#else

uint64_t microsSinceEpoch() {

    struct timeval tv;

    uint64_t micros = 0;

    gettimeofday(&tv, NULL);
    micros = ((uint64_t) tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}

#endif