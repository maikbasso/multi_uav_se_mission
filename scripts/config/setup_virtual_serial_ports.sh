#!/bin/bash

# setup the virtual serial ports

socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1

exit 0
