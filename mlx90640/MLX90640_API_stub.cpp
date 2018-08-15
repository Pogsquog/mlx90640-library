
#include <cstdlib>
#include "MLX90640_API.h"

int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
  return 0;
}

int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
  return 0;
}

int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
  return 0;
}

void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params, float emissivity, float tr, float *result)
{
  for (int i=0; i<768; ++i)
  {
    result[i] = 255.0f * (float)rand() / (float)RAND_MAX;
  }
}

float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params)
{
  return 0;
}

int MLX90640_SetChessMode(uint8_t slaveAddr)
{
  return 0;
}

int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate)
{
  return 0;
}

int MLX90640_SetDeviceMode(uint8_t slaveAddr, uint8_t deviceMode)
{
  return 0;
}

int MLX90640_SetSubPageRepeat(uint8_t slaveAddr, uint8_t subPageRepeat)
{
  return 0;
}
