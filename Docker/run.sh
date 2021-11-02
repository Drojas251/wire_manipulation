#!/usr/bin/env bash

docker run -it --net=host -v /tmp/.X11-unix -v /home/drojas/wire_manipulation_framework:/usr/local/src/wire_manipulation_framework -e DISPLAY --privileged drojas11/wire bash
