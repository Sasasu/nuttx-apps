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

#include <fcntl.h>
#include <stdio.h>
#include <syslog.h>

#include <sys/ioctl.h>

#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define BOARD_GPIO_POWER_EN_DEV "/dev/gpio0"
#define BOARD_GPIO_LED_PIN_DEV "/dev/gpio25"
#define BOARD_GPIO_SW_1_DEV "/dev/gpio1"
#define BOARD_GPIO_SW_2_DEV "/dev/gpio2"

#define SW_POOL_INTERVAL 10 // 10ms
#define SW_DEBOUNCE 2       // < 50ms
#define SW_SHORT 20         // < 100ms
#define SW_LONG 20          // > 200ms

#define SW_BOUNCE 0
#define SW_SHORT_PRESS 1
#define SW_LONG_PRESS 2

int POWER_EN, LED, SW1, SW2;
int ret = 0;

/****************************************************************************
 * hello_main
 ****************************************************************************/

void init_gpio(void) {
  POWER_EN = open(BOARD_GPIO_POWER_EN_DEV, O_RDWR);
  LED = open(BOARD_GPIO_LED_PIN_DEV, O_RDWR);
  SW1 = open(BOARD_GPIO_SW_1_DEV, O_RDONLY);
  SW2 = open(BOARD_GPIO_SW_2_DEV, O_RDONLY);

  if (POWER_EN < 1 || LED < 1 || SW1 < 1 || SW2 < 1) {
    syslog(LOG_ALERT, "ERROR: gpio device not exists");
    exit(EXIT_FAILURE);
  }

  syslog(LOG_INFO, "gpio device initialize OK");

  ret = ioctl(POWER_EN, GPIOC_WRITE, 1);
  if (ret < 0) {
    syslog(LOG_ALERT, "ERROR: Failed to enabled power %d\n", errno);
    exit(EXIT_FAILURE);
  }

  ret = ioctl(LED, GPIOC_WRITE, 1);
  if (ret < 0) {
    syslog(LOG_ALERT, "ERROR: Failed to turn LED %d\n", errno);
    exit(EXIT_FAILURE);
  }
}

void sw1(void) {}

void sw2(void) {
  static int count = 0;
  static int last = 0;
  int current = 0;
  int key_status = 0;

  ret = ioctl(SW2, GPIOC_READ, &current);
  assert(ret >= 0);

  if (last == 0 && (current == 1 || current == 0)) {
    last = current;
    return;
  }

  if (last == 1 && current == 1) {
    count++;

    if (count > SW_LONG) {
      current = 0;
    }
  }

  // key release
  if (last == 1 && current == 0) {
    if (count < SW_DEBOUNCE) {
      key_status = SW_BOUNCE; // ignore
    } else if (count < SW_SHORT) {
      key_status = SW_SHORT_PRESS; // short press
    } else {
      key_status = SW_LONG_PRESS; // long press
    }

    count = 0;
    last = 0;
  }

  switch (key_status) {
  case SW_BOUNCE:
    break;
  case SW_SHORT_PRESS:
    break;
  case SW_LONG_PRESS:
    // long power off
    syslog(LOG_ALERT, "Power off due to user request\n");
    ret = ioctl(POWER_EN, GPIOC_WRITE, 0);
    assert(ret >= 0);
    ret = ioctl(LED, GPIOC_WRITE, 0);
    assert(ret >= 0);
  }
}

int main(int argc, FAR char *argv[]) {
  init_gpio();

  while (true) {
    usleep(100 * SW_POOL_INTERVAL);

    sw1();
    sw2();
  }

  return 0;
}
