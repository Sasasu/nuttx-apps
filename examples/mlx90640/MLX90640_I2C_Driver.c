#include "./MLX90640_I2C_Driver.h"

#include <fcntl.h>
#include <stdio.h>
#include <syslog.h>

#include <sys/ioctl.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#define BOARD_I2C_DEV "/dev/i2c0"

int I2C = 0;
int I2C_freq = 400 * 1000;

void MLX90640_I2CInit(void) {
  I2C = open(BOARD_I2C_DEV, O_RDWR);
  if (I2C < 0) {
    syslog(LOG_ALERT, "failed to open %s", BOARD_I2C_DEV);
  }
}

void MLX90640_I2CFreqSet(int freq) { I2C_freq = freq * 1000; }

int MLX90640_I2CGeneralReset(void) {
  uint8_t txbuffer[1] = {0x06};

  struct i2c_msg_s i2c_msg = {
      .addr = 0x00,
      .flags = 0,
      .length = 1,
      .buffer = (uint8_t *)txbuffer,
      .frequency = I2C_freq,
  };
  struct i2c_transfer_s i2c_transfer = {.msgc = 1, .msgv = &i2c_msg};

  int ret = ioctl(I2C, I2CIOC_TRANSFER, (uintptr_t)&i2c_transfer);

  if (ret < 0) {
    syslog(LOG_ALERT, "write to i2c %d", errno);
    return -1;
  }

  return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data) {
  struct i2c_msg_s i2c_w_msg = {
      .addr = slaveAddr,
      .flags = 0,
      .length = 2,
      .buffer = (uint8_t *)&startAddress,
      .frequency = I2C_freq,
  };
  struct i2c_msg_s i2c_r_msg = {
      .addr = slaveAddr,
      .flags = I2C_M_READ,
      .length = nMemAddressRead * 2,
      .buffer = (uint8_t *)data,
      .frequency = I2C_freq,
  };

  struct i2c_msg_s i2c_msgs[] = {i2c_w_msg, i2c_r_msg};
  struct i2c_transfer_s i2c_transfer = {.msgc = 2, .msgv = i2c_msgs};

  int ret = ioctl(I2C, I2CIOC_TRANSFER, (uintptr_t)&i2c_transfer);

  if (ret < 0) {
    syslog(LOG_ALERT, "read from i2c %d", errno);
    return -1;
  }

  return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  uint16_t txbuffer[2] = {writeAddress, data};

  struct i2c_msg_s i2c_msg = {
      .addr = slaveAddr,
      .flags = 0,
      .length = 4,
      .buffer = (uint8_t *)txbuffer,
      .frequency = I2C_freq,
  };
  struct i2c_transfer_s i2c_transfer = {.msgc = 1, .msgv = &i2c_msg};

  int ret = ioctl(I2C, I2CIOC_TRANSFER, (uintptr_t)&i2c_transfer);

  if (ret < 0) {
    syslog(LOG_ALERT, "write to i2c %d", errno);
    return -1;
  }

  return 0;
}
