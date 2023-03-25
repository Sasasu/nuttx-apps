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
#include <nuttx/nx/nxfonts.h>
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
float temperatures[MLX90640_PIXEL_NUM] = {0};
uint16_t temperatures_p[MLX90640_PIXEL_NUM] = {0};
int8_t addr = 0x33;
uint16_t frame[834] = {};
paramsMLX90640 params = {};
sem_t sem;

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

uint32_t I(uint32_t p00, uint32_t p01, uint32_t p10, uint32_t p11, int x,
           int y) {
  int32_t dx = 10;
  int32_t dy = 10;
  int32_t p0 = p00 * (dx - x) * (dy - y) + p10 * x * (dy - y);
  int32_t p1 = p01 * (dx - x) * y + p11 * x * y;
  int32_t p = p0 + p1;
  return p / (dx * dy);
}

uint16_t get_t_3224(int x, int y) { return temperatures_p[32 * (23 - y) + x]; }

uint16_t get_t_bound_check_3224(int x, int y) {
  if (x < 0)
    x = 0;

  if (y < 0)
    y = 0;

  if (x >= 32)
    x = 31;

  if (y >= 24)
    y = 23;

  return get_t_3224(x, y);
}

uint16_t get_t_320x240(int x, int y) {
  int xx = x / 10;
  int yy = y / 10;

  uint16_t X0Y0 = get_t_bound_check_3224(xx, yy);
  uint16_t X1Y0 = get_t_bound_check_3224(xx + 1, yy);
  uint16_t X0Y1 = get_t_bound_check_3224(xx, yy + 1);
  uint16_t X1Y1 = get_t_bound_check_3224(xx + 1, yy + 1);

#define R(X) ((X & 0b11111 << 11) >> 10)
#define G(X) ((X & 0b11111 << 6) >> 5)
#define B(X) ((X & 0b11111) >> 0)

  int offx = x - xx * 10;
  int offy = y - yy * 10;

  uint32_t g = I(X0Y0, X0Y1, X1Y0, X1Y1, offx, offy);

  g = ((float)g / INT16_MAX) * 0b11111;
  if (g > 0b11111)
    g = 0b11111;

  g &= 0b11111;

  return g << 11 | g << 6 | g;
}

#define FB_DEV "/dev/fb0"

struct fb_state_s {
  int fd;
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
  FAR void *fbmem;
} fb;
NXHANDLE font;

void init_fb(void) {
  font = nxf_getfonthandle(FONTID_X11_MISC_FIXED_6X10);

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
  memset(fb.fbmem, 0x00, fb.pinfo.fblen);
}

void rander_fb(void) {
  float max = -100, min = 500;
  int maxX = 0, maxY = 0, minX = 0, minY = 0;
  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 24; y++) {
      float t = temperatures[32 * y + x];
      if (t > max) {
        max = t;
        maxX = x;
        maxY = y;
      }
      if (t < min) {
        min = t;
        minX = x;
        minY = y;
      }
    }
  }

  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 24; y++) {
      int index = (32 * (23 - y) + x);
      float t = temperatures[index];

      int X = ((t - min) / (max - min)) * 1024;
      int32_t G = (INT16_MAX / 1024) * X;
      temperatures_p[index] = G;
    }
  }

  maxX = maxX * 10;
  maxY = (23 - maxY) * 10;
  minX = minX * 10;
  minY = (23 - minY) * 10;

  uint16_t *screen = fb.fbmem;
  for (int x = 0; x < 320; x++) {
    for (int y = 0; y < 240; y++) {
      if (y < 10) {
        continue;
      }

      if (abs(x - minX) < 3 && abs(y - minY) < 3) {
        screen[320 * y + x] = (uint16_t)0b11111 << 0;
      } else if (abs(x - maxX) < 3 && abs(y - maxY) < 3) {
        screen[320 * y + x] = (uint16_t)0b11111 << 11;
      } else {
        screen[320 * y + x] = get_t_320x240(x, y);
      }
    }
  }

  // x -> [0, 320)
  // y -> [0, 10)
  // 屏幕底边
  for (int x = 0; x < 320; x++) {
    for (int y = 0; y < 10; y++) {
      // screen[320 * y + x] = (uint16_t)0b1111111 << 6;
      screen[320 * y + x] = INT16_MAX; // miku 蓝
    }
  }

  {
    char b[22] = {};
    sprintf(b, "MAX %3.1fC   MIN %3.1fC", max, min);
    for (int i = 0; i < sizeof(b); i++) {
      if (b[i] == ' ' || b[i] == 0)
        continue;

      // 宽度 19 * 6 = 114 像素
      const struct nx_fontbitmap_s *bitmap = nxf_getbitmap(font, b[i]);
      for (int fy = 0; fy < 10; fy++) {
        for (int fx = 0; fx < 6; fx++) {
          bool bit = (bitmap->bitmap[fy]) & (1 << (8 - fx));
          if (bit) {
            int y = 9 - fy;
            int x = 310 - (i * 7 + fx);
            screen[320 * y + x] = i < 10 ? 0b11111 << 11 : 0b11111;
          }
        }
      }
    }
  }

  static struct fb_area_s area = {.x = 0, .y = 0, .h = 240, .w = 320};
  int ret = ioctl(fb.fd, FBIO_UPDATE, (uintptr_t)&area);
  if (ret < 0) {
    int errcode = errno;
    fprintf(stderr, "ERROR: ioctl(FBIO_UPDATE) failed: %d\n", errcode);
    return;
  }
}

int fb_update(int _, char **__) {
  while (true) {
    sem_wait(&sem);
    rander_fb();
  }

  return 0;
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

  MLX90640_I2CFreqSet(1500);           // bump to 1MHz
  MLX90640_SetRefreshRate(addr, 0x07); // 0x06 = 32Hz, 0x07 = 64Hz

  init_fb();

  sem_init(&sem, 0, 0);
  task_create("rander", 100, 4096, fb_update, NULL);

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
      sem_post(&sem);
    }
  }

  return 0;
}
