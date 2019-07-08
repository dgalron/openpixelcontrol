/* Copyright 2016 Daniel Galron

Licensed under the Apache License, Version 2.0 (the "License"); you may not
use this file except in compliance with the License.  You may obtain a copy
of the License at: http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.  See the License for the
specific language governing permissions and limitations under the License. */

#include "cli.h"
#include "opc.h"
#include "spi.h"

#define NUM_DRIVERS 4

char*[NUM_DRIVERS] serial_drivers;
static u8 buffer[OPC_MAX_PIXELS_PER_MESSAGE * 4];

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
  Serial.write(d, OPC_MAX_PIXELS_PER_MESSAGE*4);
}

int main(int argc, char** argv) {
  u16 port = OPC_DEFAULT_PORT;

  u32 spi_speed_hz = 8000000;

  get_speed_and_port(&spi_speed_hz, &port, argc, argv);
  return opc_serve_main(port, serial_put_pixels, buffer);
}
