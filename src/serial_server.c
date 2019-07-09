/* Copyright 2016 Daniel Galron

Licensed under the Apache License, Version 2.0 (the "License"); you may not
use this file except in compliance with the License.  You may obtain a copy
of the License at: http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.  See the License for the
specific language governing permissions and limitations under the License. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "cli.h"
#include "opc.h"
#include "spi.h"

#define NUM_DRIVERS 1
#define BAUD_RATE 1572864

char *serial_driver_fds[NUM_DRIVERS];
static u8 buffer[OPC_MAX_PIXELS_PER_MESSAGE * 4];
uint serial_drivers[NUM_DRIVERS];

void setup_serial_device(uint driver_no) {
  /* Open file descriptor */
  serial_drivers[driver_no] = open(serial_driver_fds[driver_no], O_RDWR | O_NOCTTY);

  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof(tty));

  /* Error Handling */
  if (tcgetattr(serial_drivers[driver_no], &tty) != 0) {
    fprintf(stderr, "Error %d from tcgetattr: %s\n", errno, strerror(errno));
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)BAUD_RATE);
  cfsetispeed (&tty, (speed_t)BAUD_RATE);

  /* Setting other port stuff */
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS;        // no flow control
  tty.c_cc[VMIN]  = 1;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush port, then applies attributes */
  tcflush(serial_drivers[driver_no], TCIFLUSH);
  if (tcsetattr(serial_drivers[driver_no], TCSANOW, &tty) != 0) {
    fprintf(stderr, "Error %d from tcsetattr\n", errno);
  }
}

void setup_serial_devices() {
  for (u8 i = 0; i < NUM_DRIVERS; ++i) {
    setup_serial_device(i);
  }
}

void serial_write(u8* buffer) {
  for (uint driver_no = 0; driver_no < NUM_DRIVERS; ++driver_no) {
    fprintf(stderr, "Writing buffer %d\n", buffer);
    int n_written = write(serial_drivers[driver_no], buffer, OPC_MAX_PIXELS_PER_MESSAGE*4);
    fprintf(stderr, "Serial wrote %d bytes\n", n_written);
  }
}

void serial_put_pixels(u8* buffer, u16 count, pixel* pixels) {
  int i;
  pixel* p;
  u8* d;

  d = buffer;
  for (i = 0, p = pixels; i < count; i++, p++) {
    *d++ = 0;
    *d++ = p->b;
    *d++ = p->g;
    *d++ = p->r;
  }
  serial_write(d);
}

int main(int argc, char** argv) {
  u16 port = OPC_DEFAULT_PORT;

  u32 spi_speed_hz = 8000000;

  serial_driver_fds[0] = "/dev/ttyACM0";
  
  setup_serial_devices();
  
  get_speed_and_port(&spi_speed_hz, &port, argc, argv);
  return opc_serve_main(port, serial_put_pixels, buffer);
}
