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

#include "MLX90640_API.h"
#include "fb.h"

#ifdef __arm__
#include "bcm2835.h"
#else

void bcm2835_close()
{}

#endif

#define MLX_I2C_ADDR 0x33

#define IMAGE_SCALE 5

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
#define FPS 32
#define FRAME_TIME_MICROS (1000000/FPS)

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS 850

void put_pixel_false_colour(int x, int y, double v)
{
  // Heatmap code borrowed from: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
  const int NUM_COLORS = 7;
  static float color[NUM_COLORS][3] = {{0, 0, 0},
                                       {0, 0, 1},
                                       {0, 1, 0},
                                       {1, 1, 0},
                                       {1, 0, 0},
                                       {1, 0, 1},
                                       {1, 1, 1}};
  int idx1, idx2;
  float fractBetween = 0;
  float vmin = 5.0;
  float vmax = 50.0;
  float vrange = vmax - vmin;
  v -= vmin;
  v /= vrange;
  if (v <= 0)
  { idx1 = idx2 = 0; }
  else if (v >= 1)
  { idx1 = idx2 = NUM_COLORS - 1; }
  else
  {
    v *= (NUM_COLORS - 1);
    idx1 = floor(v);
    idx2 = idx1 + 1;
    fractBetween = v - float(idx1);
  }

  int ir, ig, ib;


  ir = (int) ((((color[idx2][0] - color[idx1][0]) * fractBetween) + color[idx1][0]) * 255.0);
  ig = (int) ((((color[idx2][1] - color[idx1][1]) * fractBetween) + color[idx1][1]) * 255.0);
  ib = (int) ((((color[idx2][2] - color[idx1][2]) * fractBetween) + color[idx1][2]) * 255.0);

  for (int px = 0; px < IMAGE_SCALE; px++)
  {
    for (int py = 0; py < IMAGE_SCALE; py++)
    {
      fb_put_pixel(x + px, y + py, ir, ig, ib);
    }
  }
}


void diep(const char *s)
{
  perror(s);
  exit(1);
}


#define BUFLEN 512
#define PORT 5005

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


int main()
{
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
    MLX90640_I2CInit();
    MLX90640_I2CFreqSet(400000); //slow speed to read EEPROM
    MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
    MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);
    switch (FPS)
    {
      case 1:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
        break;
      case 2:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
        break;
      case 4:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
        break;
      case 8:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
        break;
      case 16:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
        break;
      case 32:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
        break;
      case 64:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
        break;
      default:
        printf("Unsupported framerate: %d", FPS);
        return 1;
    }
    MLX90640_SetChessMode(MLX_I2C_ADDR);

    paramsMLX90640 mlx90640;
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    MLX90640_I2CFreqSet(1000000); //high speed to read frame data

#ifdef __arm__
    fb_init();
#endif

    while (true)
    {
      std::cout << "." << std::flush;
      auto start = std::chrono::system_clock::now();
      int result = MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
      if (result != 0)
      {
        break;
      }

      eTa = MLX90640_GetTa(frame, &mlx90640);
      MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);
      std::cout << "P" << std::flush;

#ifdef __arm__
      for (int y = 0; y < 24; y++) for (int x = 0; x < 32; x++)
      {
        float val = mlx90640To[32 * (23 - y) + x];
        put_pixel_false_colour((y * IMAGE_SCALE), (x * IMAGE_SCALE), val);
      }
#endif

      std::cout << "U" << std::flush;
      bool received_ping = udp_recieve(s, buff, sizeof(buff), &si_other);
      if (received_ping)
      {
        got_target = true;
      }

      if (got_target)
      {
        std::cout << "T" << std::flush;
        transmit_image(s, mlx90640To, &si_other);
      }

      auto end = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      std::this_thread::sleep_for(std::chrono::microseconds(frame_time - elapsed));
    }

    bcm2835_close();
  }

#ifdef __arm
  fb_cleanup();
#endif

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
