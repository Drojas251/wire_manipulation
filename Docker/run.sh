#!/usr/bin/env bash


docker run -it --net=host -v /tmp/.X11-unix -v /home/$USERNAME/wire_manipulation_framework:/usr/local/src/wire_manipulation_framework -v /dev:/dev -e DISPLAY --privileged drojas11/wire bash
