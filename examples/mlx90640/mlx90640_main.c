/****************************************************************************
 * apps/examples/hello/hello_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>

#include "./MLX90640_API.h"
#include "./MLX90640_I2C_Driver.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

#define TA_SHIFT 8
float emissivity = 0.95;
float temperatures[MLX90640_PIXEL_NUM] = {};
int8_t addr = 0x33;
uint16_t frame[834] = {};
paramsMLX90640 params = {};

void dump_hex(int size, uint8_t *data, FILE *out) {
  const char *hex = "0123456789ABCDEF";

  for (int i = 0; i < size; i++) {
    if (i % 24 == 0) {
      putc('\n', out);
    }
    int d1 = (data[i] >> 4) & 0xF;
    int d2 = data[i] & 0xF;

    fputs("0x", out);
    putc(hex[d1], out);
    putc(hex[d2], out);
    fputs(", ", out);
  }
  putc('\n', out);
}

int main(int argc, FAR char *argv[]) {
  MLX90640_I2CInit();

  int ret = MLX90640_I2CGeneralReset();
  if (ret < 0) {
    fprintf(stderr, "reset %d\n", errno);
    exit(-1);
  }

  // max frequency for eeprom is 400kHz
  MLX90640_I2CFreqSet(50);
  fprintf(stderr, "reading eeprom useing %dkHz\n", 50);
  ret = MLX90640_DumpEE(addr, frame);
  if (ret < 0) {
    fprintf(stderr, "read %d\n", errno);
    exit(-1);
  }

  dump_hex(MLX90640_EEPROM_DUMP_NUM * 2, (uint8_t *)frame, stderr);

  ret = MLX90640_ExtractParameters(frame, &params);
  if (ret < 0) {
    fprintf(stderr, "extract params %d\n", ret);
    exit(-1);
  }

  MLX90640_I2CFreqSet(1000); // bump to 1MHz
  return 0;

  while (true) {
    // request a frame
    ret = MLX90640_TriggerMeasurement(addr);
    if (ret < 0) {
      fprintf(stderr, "request frame %d\n", errno);
      exit(-1);
    }

    for (int i = 0; i < 2; i++) {
      ret = MLX90640_GetFrameData(addr, frame);
      if (ret < 0) {
        fprintf(stderr, "frame read %d page %d\n", errno, i);
        exit(-1);
      }

      float vdd = MLX90640_GetVdd(frame, &params);
      float ta = MLX90640_GetTa(frame, &params);

      MLX90640_CalculateTo(frame, &params, emissivity, ta - TA_SHIFT,
                           temperatures);
    }
  }
  return 0;
}
