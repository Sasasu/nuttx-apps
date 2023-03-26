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
float temperatures_f[MLX90640_PIXEL_NUM] = {0};
uint16_t temperatures_p[MLX90640_PIXEL_NUM] = {0};
int32_t temperatures[MLX90640_PIXEL_NUM] = {0};
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

int32_t div_10_small(int32_t x) {
  static const uint8_t t[] = {
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
      1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,
      3,  3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,
      5,  5,  5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  7,  7,
      7,  7,  7,  7,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,
      9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  10, 10, 10, 10, 10, 10, 10, 10,
      10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12,
      12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14,
      14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16,
      16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
      18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19,
      19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21,
      21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23,
      23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25,
      25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
      27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28,
      28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30,
      30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31};

  if (x <= 320) {
    return t[x];
  }

  return x / 10;
}

static inline uint16_t I(uint32_t p00, uint32_t p01, uint32_t p10, uint32_t p11,
                         int x, int y) {
  const int8_t dx = 10;
  const int8_t dy = 10;
  int8_t dxx = dx - x;
  int8_t dyy = dy - y;

  int32_t p0 = p00 * dxx * dyy + p10 * x * dyy;
  int32_t p1 = p01 * dxx * y + p11 * x * y;
  int32_t p = p0 + p1;

  p += p / 4;
  p = p / 128;
  return p;
}

static inline uint16_t get_t_3224(int x, int y) {
  return temperatures_p[32 * (23 - y) + x];
}

static inline uint16_t get_t_320x240(int x, int y) {
  int xx = div_10_small(x);
  int yy = div_10_small(y);
  int offx = x - xx * 10;
  int offy = y - yy * 10;
  int xx1 = (xx + 1) >= 32 ? 31 : xx + 1;
  int yy1 = (yy + 1) >= 24 ? 23 : yy + 1;

  uint16_t X0Y0 = get_t_3224(xx, yy);
  uint16_t X1Y0 = get_t_3224(xx1, yy);
  uint16_t X0Y1 = get_t_3224(xx, yy1);
  uint16_t X1Y1 = get_t_3224(xx1, yy1);

  uint32_t g = I(X0Y0, X0Y1, X1Y0, X1Y1, offx, offy);

  g = (g * 0b11111 / INT16_MAX);
  if (g > 0b11111)
    g = 0b11111;

  g &= 0b11111;

  return g << 11 | g << 6 | g;
}

void render_fb(void) {
  int maxid = 0, minid = 0;
  int maxX = 0, maxY = 0, minX = 0, minY = 0;
  for (int i = 0; i < (32 * 24); i++) {
    int t = temperatures[i];
    if (t > temperatures[maxid]) {
      maxid = i;
    }
    if (t < temperatures[minid]) {
      minid = i;
    }
  }

  maxX = maxid % 32;
  maxY = maxid / 32;
  minX = minid % 32;
  minY = minid / 32;

  int32_t union_arg = (temperatures[maxid] - temperatures[minid]);
  for (int x = 0; x < 32; x++) {
    for (int y = 0; y < 24; y++) {
      int index = (32 * (23 - y) + x);

      int32_t t = temperatures[index];
      int32_t X = ((t - temperatures[minid]) * INT16_MAX) / union_arg;
      temperatures_p[index] = X;
    }
  }

  maxX = maxX * 10;
  maxY = (23 - maxY) * 10;
  minX = minX * 10;
  minY = (23 - minY) * 10;

  uint16_t *screen = fb.fbmem;
  for (int x = 0; x < 320; x++) {
#define STATUS_BG 0x0000000
    screen[320 * 0 + x] = STATUS_BG;
    screen[320 * 1 + x] = STATUS_BG;
    screen[320 * 2 + x] = STATUS_BG;
    screen[320 * 3 + x] = STATUS_BG;
    screen[320 * 4 + x] = STATUS_BG;
    screen[320 * 5 + x] = STATUS_BG;
    screen[320 * 6 + x] = STATUS_BG;
    screen[320 * 7 + x] = STATUS_BG;
    screen[320 * 8 + x] = STATUS_BG;
    screen[320 * 9 + x] = STATUS_BG;
#define ABS(x) ((x) > 0 ? (x) : -(x))
    int rxmin = ABS(x - minX);
    int rxmax = ABS(x - maxX);

    for (int y = 10; y < 240; y++) {
      if (rxmin < 3 && ABS(y - minY) < 3) {
        screen[320 * y + x] = (uint16_t)0b11111 << 0;
      } else if (rxmax < 3 && ABS(y - maxY) < 3) {
        screen[320 * y + x] = (uint16_t)0b11111 << 11;
      } else {
        screen[320 * y + x] = get_t_320x240(x, y);
      }
    }
  }

  {
    char b[sizeof("\1MAX 1.1C \1 MIN 1.1C \1 1.1Hz")] = {};
    static struct timespec last = {}, cuttent = {};
    clock_gettime(CLOCK_MONOTONIC, &cuttent);
    clock_timespec_subtract(&cuttent, &last, &last);
    sprintf(b, "\1MAX %3.1fC \2MIN %3.1fC \3%2.1fHz", temperatures_f[maxid],
            temperatures_f[minid],
            1000.0 / (SEC_TO_MS(last.tv_sec) + NS_TO_MS(last.tv_nsec)));
    last = cuttent;

    int count = 0;
    uint16_t fb_color = 0; // black
    for (int i = 0; i < sizeof(b); i++) {
      switch (b[i]) {
      case ' ':
        count++;
      case 0:
        continue;
      case '\1':
        fb_color = 0b11111 << 11; // red
        continue;
      case '\2':
        fb_color = 0b11111 << 0; // blue
        continue;
      case '\3':
        fb_color = UINT16_MAX;
        continue;
      default:
        break;
      }

      const struct nx_fontbitmap_s *bitmap = nxf_getbitmap(font, b[i]);
      for (int fx = 0; fx < 6; fx++) {
        uint8_t mask = 1 << (8 - fx);
        for (int fy = 0; fy < 10; fy++) {
          bool bit = bitmap->bitmap[fy] & mask;
          if (bit) {
            int y = 8 - fy;
            int x = 310 - (count * 7 + fx);
            screen[320 * y + x] = fb_color;
          }
        }
      }
      count++;
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

void wakeup_render(void) { sem_post(&sem); }

int fb_update(int _, char **__) {
  while (true) {
    sem_wait(&sem);
    render_fb();
  }

  return 0;
}

void render_stdout(void) {
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
      float t = temperatures_f[32 * y + x];

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

  MLX90640_SetRefreshRate(addr, 0x04); // 0x06 = 32Hz, 0x07 = 64Hz

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

  MLX90640_I2CFreqSet(1200);

  init_fb();

  sem_init(&sem, 0, 0);
  task_create("render", 101, 4096, fb_update, NULL);

  while (true) {

    for (int i = 0; i < 2; i++) {
      ret = MLX90640_GetFrameData(addr, frame);
      if (ret < 0) {
        fprintf(stdout, "frame read %d page 2\n", errno);
        exit(-1);
      }
      float ta = MLX90640_GetTa(frame, &params);
      MLX90640_CalculateTo(frame, &params, emissivity, ta - TA_SHIFT,
                           temperatures_f);

      for (int x = 0; x < (32 * 24); x++) {
        temperatures[x] = temperatures_f[x] * 1024; // [-40, 350]
      }

      wakeup_render();
    }
  }

  return 0;
}
