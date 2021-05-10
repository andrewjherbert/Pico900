#!/bin/sh
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program /home/ajh2/pico/Pico900/build/pico900.elf verify reset exit"
