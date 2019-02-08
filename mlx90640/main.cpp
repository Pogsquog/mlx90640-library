#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h>
#include <sched.h>

#include "MLX90640_API.h"

#define MLX_I2C_ADDR 0x33

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
constexpr int _1_MHZ = 1000000;
constexpr int FPS = 32;
constexpr int FRAME_TIME_MICROS = (_1_MHZ/ (FPS ? FPS : 0.5));

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS 850

void set_realtime_priority()
{
    int ret;

    // We'll operate on the currently running thread.
    pthread_t this_thread = pthread_self();

    // struct sched_param is used to store the scheduling priority
    struct sched_param params;

    // We'll set the priority to the maximum.
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);

    std::cout << "Trying to set thread realtime prio = " << params.sched_priority << std::endl;

    // Attempt to set thread real-time priority to the SCHED_FIFO policy
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0)
    {
        // Print the error
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    }

    // Now verify the change in thread priority
    int policy = 0;
    ret = pthread_getschedparam(this_thread, &policy, &params);
    if (ret != 0)
    {
        std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
        return;
    }

    // Check the correct policy was applied
    if (policy != SCHED_FIFO)
    {
        std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
    }
    else
    {
        std::cout << "SCHED_FIFO OK" << std::endl;
    }

    // Print thread scheduling priority
    std::cout << "Thread priority is " << params.sched_priority << std::endl;
}

void diep(const char *s)
{
    perror(s);
    exit(1);
}


#define BUFLEN 512
#define PORT 5222

void udp_send(int socket, sockaddr_in *address, const uint8_t *buff, socklen_t buffersize)
{
    socklen_t slen = sizeof(sockaddr_in);
    auto result = sendto(socket, buff, buffersize, 0, (const sockaddr *) address, slen);
    if (result == -1)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            diep("sendto()");
        }
    }
}

bool udp_recieve(int socket, uint8_t *buff, int max_buffersize, sockaddr_in *source_info)
{
    socklen_t slen = sizeof(sockaddr_in);
    auto bytes_received = recvfrom(socket, buff, max_buffersize, 0, (sockaddr *) source_info, &slen);
    if (bytes_received == -1)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            diep("recvfrom()");
        }
        return false;
    }

    //printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(source_info->sin_addr), ntohs(source_info->sin_port), buff);
    return true;
}

void transmit_image(int s, float pDouble[768], sockaddr_in *pIn);

using namespace std;

int main()
{
    set_realtime_priority();

    sockaddr_in si_me, si_other;
    int s;
    uint8_t buff[BUFLEN];
    bool got_target = false;

    if ((s = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP)) == -1)
    {
        diep("socket");
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr *) &si_me, sizeof(si_me)) == -1)
    {
        diep("bind");
    }

    static uint16_t eeMLX90640[832];
    float emissivity = 1;
    uint16_t frame[834];
    static float image[768];
    static float mlx90640To[768];
    float eTa;
    static uint16_t data[768 * sizeof(float)];

    auto frame_time = std::chrono::microseconds(FRAME_TIME_MICROS + OFFSET_MICROS);

    while (true)
    {
        std::cout << "I2c init\n";
        MLX90640_I2CInit();

        std::cout << "set frequency to 400khz\n";
        MLX90640_I2CFreqSet(400000); //slow speed to read EEPROM

        std::cout << "Set device mode\n";
        MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);

        std::cout << "set sub page repeat\n";
        MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);

        std::cout << "set fps to " << FPS << "\n";
        switch (FPS)
        {
        case 0:
            // actually 0.5hz
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b000);
            break;
        case 1:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
            break;
        case 2:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
            break;
        case 4:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
            break;
        case 8:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
            break;
        case 16:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
            break;
        case 32:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
            break;
        case 64:MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
            break;
        default:printf("Unsupported framerate: %d", FPS);
            return 1;
        }

        std::cout << "set chess mode\n";
        MLX90640_SetChessMode(MLX_I2C_ADDR);

        paramsMLX90640 mlx90640;
        MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
        MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

        std::cout << "set frequency to 1MHz\n";
        MLX90640_I2CFreqSet(_1_MHZ); //high speed to read frame data

        while (true)
        {
            auto start = std::chrono::system_clock::now();
            int result = MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
            if (result != 0)
            {
                std::cout << "breaking from capture loop\n";
                break;
            }

            eTa = MLX90640_GetTa(frame, &mlx90640);
            MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);
            //MLX90640_GetImage(frame, &mlx90640, mlx90640To);

            bool received_ping = udp_recieve(s, buff, sizeof(buff), &si_other);
            if (received_ping)
            {
                got_target = true;
            }

            if (got_target)
            {
                transmit_image(s, mlx90640To, &si_other);
            }

            auto end = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            auto sleep_time = std::chrono::microseconds(frame_time - elapsed);

            if (sleep_time <= std::chrono::microseconds(0))
            {
                //std::cout << "uh oh, maybe I missed a frame, sleep time = " << sleep_time.count() << std::endl;
            }
            else
            {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    }

    std::cout << "close socket\n";
    close(s);
    return 0;
}

void transmit_image(int s, float image[768], sockaddr_in *target)
{
    uint8_t packet_buffer[768 + 1];

    for (int i = 0; i < 4; ++i)
    {
        packet_buffer[0] = i;
        memcpy(&packet_buffer[1], &image[i * 768 / 4], 768);
        udp_send(s, target, packet_buffer, sizeof(packet_buffer));
    }
}
