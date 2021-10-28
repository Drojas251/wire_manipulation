#!/usr/bin/env bash

docker run -it --net=host -v /tmp/.X11-unix -v /home/diego/wire_manipulation_framework:/usr/local/src/wire_manipulation_framework -e DISPLAY drojas11/wire bash
