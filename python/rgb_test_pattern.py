#!/usr/bin/env python
"""
A demo client for Open Pixel Control
Modified from the raver_plaid client by 
    David Wallace / https://github.com/longears

Sets all pixels in strip to red, then green, then blue, then dark.
Useful for diagnosing color input order issues with LED strips that
may expect input orders other than RGB (e.g. WS2801).

Usage:
    python_clients/rgb_test_pattern.py
"""

from __future__ import division
import time
import math
import sys
import optparse

import opc 


#-------------------------------------------------------------------------------
# handle command line
parser = optparse.OptionParser()
parser.add_option('-l', '--layout', dest='layout',
                    action='store', type='string',
                    help='layout file')
parser.add_option('-s', '--server', dest='server', default='127.0.0.1:7890',
                    action='store', type='string',
                    help='ip and port of server')
parser.add_option('-f', '--fps', dest='fps', default=20,
                    action='store', type='int',
                    help='frames per second')

options, args = parser.parse_args()


#-------------------------------------------------------------------------------
# connect to server

client = opc.Client(options.server)
if client.can_connect():
    print('    connected to %s' % options.server)
else:
    # can't connect, but keep running in case the server appears later
    print('    WARNING: could not connect to %s' % options.server)
print()


#-------------------------------------------------------------------------------
# send pixels

print('    sending pixels forever (control-c to exit)...')
print()

n_pixels = 44*64  # number of pixels in the included "wall" layout
fps = 1         # frames per second (color switches every frame)

while True:
    for c in range(4):
        pixels = []
        rgb = [ 0, 0, 0 ]
        if c < 3:
            rgb[c%3] = 255
        rgb = tuple(rgb)
        for ii in range(n_pixels):
            pixels.append(rgb)
        client.put_pixels(pixels, channel=0)
        time.sleep(1 / fps)

