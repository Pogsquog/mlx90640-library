/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90640_I2C_Driver.h"
#include <iostream>
#include <bcm2835.h>

void MLX90640_I2CInit()
{
  bcm2835_init();
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_07, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_i2c_begin();
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
  char cmd[2] = {(char) (startAddress >> 8), (char) (startAddress & 0xFF)};

  bcm2835_i2c_setSlaveAddress(slaveAddr);

  char buf[1664];
  uint16_t *p = data;

  int result = bcm2835_i2c_write_read_rs(cmd, 2, buf, nMemAddressRead * 2);

  if (result != BCM2835_I2C_REASON_OK)
  {
    std::cout << "error reading" << std::endl;
    return -1;
  }

  for (int count = 0; count < nMemAddressRead; ++count)
  {
    int i = count << 1;
    *p++ = ((uint16_t) buf[i] << 8) | buf[i + 1];
  }
  return 0;
}

void MLX90640_I2CFreqSet(int freq)
{
  bcm2835_i2c_set_baudrate(freq);
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
  char cmd[4] =
    {(char) (writeAddress >> 8),
     (char) (writeAddress & 0x00FF),
     (char) (data >> 8),
     (char) (data & 0x00FF)};

  auto result = bcm2835_i2c_write(cmd, 4);

  if (result != BCM2835_I2C_REASON_OK)
  {
    std::cout << "error writing" << std::endl;
    return -1;
  }

  return 0;
}
