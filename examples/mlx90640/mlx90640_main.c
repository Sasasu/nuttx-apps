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

void div_10_small(int32_t x, int *r, int *d) {
  static const uint8_t t[] = {
      0,  0, 0,  1, 0,  2, 0,  3, 0,  4, 0,  5, 0,  6, 0,  7, 0,  8, 0,  9,
      1,  0, 1,  1, 1,  2, 1,  3, 1,  4, 1,  5, 1,  6, 1,  7, 1,  8, 1,  9,
      2,  0, 2,  1, 2,  2, 2,  3, 2,  4, 2,  5, 2,  6, 2,  7, 2,  8, 2,  9,
      3,  0, 3,  1, 3,  2, 3,  3, 3,  4, 3,  5, 3,  6, 3,  7, 3,  8, 3,  9,
      4,  0, 4,  1, 4,  2, 4,  3, 4,  4, 4,  5, 4,  6, 4,  7, 4,  8, 4,  9,
      5,  0, 5,  1, 5,  2, 5,  3, 5,  4, 5,  5, 5,  6, 5,  7, 5,  8, 5,  9,
      6,  0, 6,  1, 6,  2, 6,  3, 6,  4, 6,  5, 6,  6, 6,  7, 6,  8, 6,  9,
      7,  0, 7,  1, 7,  2, 7,  3, 7,  4, 7,  5, 7,  6, 7,  7, 7,  8, 7,  9,
      8,  0, 8,  1, 8,  2, 8,  3, 8,  4, 8,  5, 8,  6, 8,  7, 8,  8, 8,  9,
      9,  0, 9,  1, 9,  2, 9,  3, 9,  4, 9,  5, 9,  6, 9,  7, 9,  8, 9,  9,
      10, 0, 10, 1, 10, 2, 10, 3, 10, 4, 10, 5, 10, 6, 10, 7, 10, 8, 10, 9,
      11, 0, 11, 1, 11, 2, 11, 3, 11, 4, 11, 5, 11, 6, 11, 7, 11, 8, 11, 9,
      12, 0, 12, 1, 12, 2, 12, 3, 12, 4, 12, 5, 12, 6, 12, 7, 12, 8, 12, 9,
      13, 0, 13, 1, 13, 2, 13, 3, 13, 4, 13, 5, 13, 6, 13, 7, 13, 8, 13, 9,
      14, 0, 14, 1, 14, 2, 14, 3, 14, 4, 14, 5, 14, 6, 14, 7, 14, 8, 14, 9,
      15, 0, 15, 1, 15, 2, 15, 3, 15, 4, 15, 5, 15, 6, 15, 7, 15, 8, 15, 9,
      16, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 8, 16, 9,
      17, 0, 17, 1, 17, 2, 17, 3, 17, 4, 17, 5, 17, 6, 17, 7, 17, 8, 17, 9,
      18, 0, 18, 1, 18, 2, 18, 3, 18, 4, 18, 5, 18, 6, 18, 7, 18, 8, 18, 9,
      19, 0, 19, 1, 19, 2, 19, 3, 19, 4, 19, 5, 19, 6, 19, 7, 19, 8, 19, 9,
      20, 0, 20, 1, 20, 2, 20, 3, 20, 4, 20, 5, 20, 6, 20, 7, 20, 8, 20, 9,
      21, 0, 21, 1, 21, 2, 21, 3, 21, 4, 21, 5, 21, 6, 21, 7, 21, 8, 21, 9,
      22, 0, 22, 1, 22, 2, 22, 3, 22, 4, 22, 5, 22, 6, 22, 7, 22, 8, 22, 9,
      23, 0, 23, 1, 23, 2, 23, 3, 23, 4, 23, 5, 23, 6, 23, 7, 23, 8, 23, 9,
      24, 0, 24, 1, 24, 2, 24, 3, 24, 4, 24, 5, 24, 6, 24, 7, 24, 8, 24, 9,
      25, 0, 25, 1, 25, 2, 25, 3, 25, 4, 25, 5, 25, 6, 25, 7, 25, 8, 25, 9,
      26, 0, 26, 1, 26, 2, 26, 3, 26, 4, 26, 5, 26, 6, 26, 7, 26, 8, 26, 9,
      27, 0, 27, 1, 27, 2, 27, 3, 27, 4, 27, 5, 27, 6, 27, 7, 27, 8, 27, 9,
      28, 0, 28, 1, 28, 2, 28, 3, 28, 4, 28, 5, 28, 6, 28, 7, 28, 8, 28, 9,
      29, 0, 29, 1, 29, 2, 29, 3, 29, 4, 29, 5, 29, 6, 29, 7, 29, 8, 29, 9,
      30, 0, 30, 1, 30, 2, 30, 3, 30, 4, 30, 5, 30, 6, 30, 7, 30, 8, 30, 9,
      31, 0, 31, 1, 31, 2, 31, 3, 31, 4, 31, 5, 31, 6, 31, 7, 31, 8, 31, 9};

  uint16_t a = ((uint16_t *)t)[x];
  int8_t *b = (int8_t *)&a;

  *r = b[0];
  *d = b[1];
}

static inline uint32_t I(uint32_t p00, uint32_t p01, uint32_t p10, uint32_t p11,
                         int x, int y) {
  const int8_t dx = 10;
  const int8_t dy = 10;
  int8_t dxx = dx - x;
  int8_t dyy = dy - y;

  int32_t p0 = p00 * dxx * dyy + p10 * x * dyy;
  int32_t p1 = p01 * dxx * y + p11 * x * y;
  int32_t p = p0 + p1;

  return p;
}

static inline uint16_t get_t_3224(int x, int y) {
  return temperatures_p[32 * (23 - y) + x];
}

static inline uint16_t get_t_320x240(int x, int y) {
  int xx, offx;
  div_10_small(x, &xx, &offx);
  int yy, offy;
  div_10_small(y, &yy, &offy);

  int ix0y0 = 32 * (23 - yy) + xx;
  int ix1y0 = ix0y0;
  int ix0y1 = ix0y0;
  int ix1y1 = ix0y0;

  if (xx != 31) {
    ix1y0 += 1;
    ix1y1 += 1;
  }

  if (yy != 23) {
    ix0y1 -= 32;
    ix1y1 -= 32;
  }

  // 16bit grayscale
  uint16_t X0Y0 = temperatures_p[ix0y0];
  uint16_t X1Y0 = temperatures_p[ix1y0];
  uint16_t X0Y1 = temperatures_p[ix0y1];
  uint16_t X1Y1 = temperatures_p[ix1y1];

  // 16bit * 100 grayscale
  uint32_t g = I(X0Y0, X0Y1, X1Y0, X1Y1, offx, offy);

  // to 5bit grayscale
  g = g >> 17;
  g += g / 4;

  g = g & 0b11111;

  return g << 11 | g << 6 | g;
}

int maxid = 0, minid = 0;
int maxX = 0, maxY = 0, minX = 0, minY = 0;

void render_fb(void) {

  uint16_t *screen = fb.fbmem;
  for (int ii = 0; ii < (32 * 24); ii++) {
    int t = temperatures[ii];
    if (t > temperatures[maxid]) {
      maxid = ii;
    }
    if (t < temperatures[minid]) {
      minid = ii;
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
    // sem_wait(&sem);
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

bool update_in_page(int pixelNumber, int subPage) {
  if ((pixelNumber / 32) % 2 == 1) {
    // odd
    if (subPage == 1 && pixelNumber % 2 == 1) {
      return false;
    }
    if (subPage == 0 && pixelNumber % 2 == 0) {
      return false;
    }
  } else {
    // even
    if (subPage == 1 && pixelNumber % 2 == 0) {
      return false;
    }
    if (subPage == 0 && pixelNumber % 2 == 1) {
      return false;
    }
  }

  return true;
}

int main(int argc, FAR char *argv[]) {
  MLX90640_I2CInit();
  MLX90640_I2CFreqSet(400); // max frequency for eeprom is 400kHz

  int ret = MLX90640_I2CGeneralReset();
  if (ret < 0) {
    fprintf(stdout, "reset %d\n", errno);
    exit(-1);
  }

  MLX90640_SetRefreshRate(addr, 0x06); // 0x06 = 32Hz, 0x07 = 64Hz

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

  MLX90640_I2CFreqSet(1000);

  init_fb();

  sem_init(&sem, 0, 0);
  task_create("render", 101, 4096, fb_update, NULL);

  while (true) {
    ret = MLX90640_GetFrameData(addr, frame);
    if (ret < 0) {
      fprintf(stdout, "frame read %d page 2\n", errno);
      exit(-1);
    }
    float ta = MLX90640_GetTa(frame, &params);
    MLX90640_CalculateTo(frame, &params, emissivity, ta - TA_SHIFT,
                         temperatures_f);

    int subPage = frame[833];
    for (int x = 0; x < (32 * 24); x++) {
      if (update_in_page(x, subPage)) {
        temperatures[x] = temperatures_f[x] * 1024; // [-40, 350]
      }
    }
    wakeup_render();
  }

  return 0;
}
