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
#include <nuttx/video/fb.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include "./MLX90640_API.h"
#include "./MLX90640_I2C_Driver.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/// Convert seconds to milliseconds
#define SEC_TO_MS(sec) ((sec)*1000)
/// Convert seconds to microseconds
#define SEC_TO_US(sec) ((sec)*1000000)
/// Convert seconds to nanoseconds
#define SEC_TO_NS(sec) ((sec)*1000000000)

/// Convert nanoseconds to seconds
#define NS_TO_SEC(ns) ((ns) / 1000000000)
/// Convert nanoseconds to milliseconds
#define NS_TO_MS(ns) ((ns) / 1000000)
/// Convert nanoseconds to microseconds
#define NS_TO_US(ns) ((ns) / 1000)

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

#define FB_DEV "/dev/fb0"

struct fb_state_s {
  int fd;
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
  FAR void *fbmem;
} fb;

void init_fb(void) {
  fb.fd = open(FB_DEV, O_RDWR);
  if (fb.fd < 0) {
    syslog(LOG_ALERT, "failed to open %s", FB_DEV);
    exit(-1);
  }

  int ret = ioctl(fb.fd, FBIOGET_VIDEOINFO, (uintptr_t)&fb.vinfo);
  if (ret < 0) {
    syslog(LOG_ALERT, "FBIOGET_VIDEOINFO %d", errno);
    exit(-1);
  }

  printf("VideoInfo:\n");
  printf("      fmt: %u\n", fb.vinfo.fmt);
  printf("     xres: %u\n", fb.vinfo.xres);
  printf("     yres: %u\n", fb.vinfo.yres);
  printf("  nplanes: %u\n", fb.vinfo.nplanes);
  printf("\n");

  ret = ioctl(fb.fd, FBIOGET_PLANEINFO, (uintptr_t)&fb.pinfo);
  if (ret < 0) {
    syslog(LOG_ALERT, "FBIOGET_PLANEINFO %d", errno);
    exit(-1);
  }

  printf("PlaneInfo (plane 0):\n");
  printf("    fbmem: %p\n", fb.pinfo.fbmem);
  printf("    fblen: %lu\n", (unsigned long)fb.pinfo.fblen);
  printf("   stride: %u\n", fb.pinfo.stride);
  printf("  display: %u\n", fb.pinfo.display);
  printf("      bpp: %u\n", fb.pinfo.bpp);
  printf("\n");

  fb.fbmem = mmap(NULL, fb.pinfo.fblen, PROT_READ | PROT_WRITE,
                  MAP_SHARED | MAP_FILE, fb.fd, 0);
  if (fb.fbmem == MAP_FAILED) {
    syslog(LOG_ALERT, "MMAP %d", errno);
    exit(-1);
  }
}

struct fb_area_s area = {.x = 0, .y = 0, .h = 240, .w = 320};

void rander_fb(void) {
  float max = -40, min = 340;
  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 24; y++) {
      float t = temperatures[32 * y + x];
      if (t > max)
        max = t;
      if (t < min)
        min = t;
    }
  }

  uint16_t MAXR = 63, MAXG = 127, MAXB = 63; // RGB565
  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 24; y++) {
      float t = temperatures[32 * (23 - y) + x];

      int X = ((t - min) / (max - min)) * 1024;
      int B = (-1024.0 / 116281) * X * X + (2048.0 / 341) * X - 0;
      int G =
          (-512.0 / 58311) * X * X + (232960.0 / 19437) * X - (524288.0 / 171);
      int R =
          (-512.0 / 58311) * X * X + (1048064.0 / 58311) * X - (465920.0 / 57);

      B = (MAXB * B) / 1024;
      G = (MAXG * G) / 1024;
      R = (MAXR * R) / 1024;

      uint16_t P = (uint16_t)B << 11 | (uint16_t)G << 6 | (uint16_t)R;

      for (int fx = 0; fx < 10; fx++) {
        for (int fy = 0; fy < 10; fy++) {
          int px = x * 10 + fx;
          int py = y * 10 + fy;

          uint16_t *screen = fb.fbmem;
          screen[320 * py + px] = P;
        }
      }
    }

    int ret = ioctl(fb.fd, FBIO_UPDATE, (uintptr_t)&area);
    if (ret < 0) {
      int errcode = errno;
      fprintf(stderr, "ERROR: ioctl(FBIO_UPDATE) failed: %d\n", errcode);
    }
  }
}

void rander_stdout(void) {
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
      float t = temperatures[32 * y + x];

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

  init_fb();

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

      struct timespec s = {}, e = {};
      clock_gettime(CLOCK_REALTIME, &s);
      MLX90640_CalculateTo(frame, &params, emissivity, ta - TA_SHIFT,
                           temperatures);
      clock_gettime(CLOCK_REALTIME, &e);
      clock_timespec_subtract(&e, &s, &s);

      fprintf(stdout, "\nrander time %ldms\n",
              SEC_TO_MS(s.tv_sec) + NS_TO_MS(s.tv_nsec));
    }

    rander_fb();
  }

  return 0;
}
