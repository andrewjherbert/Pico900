#!/bin/bash
pushd ~/home/pico/Pico900
openocd -f interface/picoprobe.cfg -f pico900.cfg -f target/rp2040.cfg -c "program /home/ajh2/pico/Pico900/build/pico900.elf verify reset exit"
popd

