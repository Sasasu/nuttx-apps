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
  MLX90640_I2CFreqSet(400); // max frequency for eeprom is 400kHz

  int ret = MLX90640_I2CGeneralReset();
  if (ret < 0) {
    fprintf(stdout, "reset %d\n", errno);
    exit(-1);
  }

  ret = MLX90640_DumpEE(addr, frame);
  if (ret < 0) {
    fprintf(stdout, "read %d\n", errno);
    exit(-1);
  }

  ret = MLX90640_ExtractParameters(frame, &params);
  if (ret < 0) {
    fprintf(stdout, "extract params %d\n", ret);
    exit(-1);
  }

  MLX90640_I2CFreqSet(1000); // bump to 1MHz

  while (true) {
    // request a frame
    ret = MLX90640_TriggerMeasurement(addr);
    if (ret < 0) {
      fprintf(stdout, "request frame %d\n", errno);
      exit(-1);
    }

    for (int i = 0; i < 2; i++) {
      ret = MLX90640_GetFrameData(addr, frame);
      if (ret < 0) {
        fprintf(stdout, "frame read %d page %d\n", errno, i);
        exit(-1);
      }

      float ta = MLX90640_GetTa(frame, &params);

      MLX90640_CalculateTo(frame, &params, emissivity, ta - TA_SHIFT,
                           temperatures);
    }

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

    for (int x = 0; x < 32; x++) {
      fprintf(stdout, "\n %s", ANSI_COLOR_RESET);
      for (int y = 0; y < 24; y++) {
        float t = temperatures[32 * (23 - y) + x];

        if (t > 99.99)
          t = 99.99;

        if (t > 29.0) {
          fprintf(stdout, ANSI_COLOR_MAGENTA "%3.1f " ANSI_COLOR_RESET, t);
        } else if (t > 26.0) {
          fprintf(stdout, ANSI_COLOR_RED "%3.1f " ANSI_COLOR_RESET, t);
        } else if (t > 21.0) {
          fprintf(stdout, ANSI_COLOR_YELLOW "%3.1f " ANSI_COLOR_YELLOW, t);
        } else if (t < 10.0) {
          fprintf(stdout, ANSI_COLOR_BLUE "%3.1f " ANSI_COLOR_RESET, t);
        } else if (t < 17.0) {
          fprintf(stdout, ANSI_COLOR_CYAN "%3.1f " ANSI_COLOR_RESET, t);
        } else if (t < 22.0) {
          fprintf(stdout, ANSI_COLOR_GREEN "%3.1f " ANSI_COLOR_RESET, t);
        } else {
          fprintf(stdout, ANSI_COLOR_RED "%3.1f " ANSI_COLOR_RESET, t);
        }
      }
    }

    fprintf(stdout, "\n %s", ANSI_COLOR_RESET);
    sleep(1);
  }

  return 0;
}
