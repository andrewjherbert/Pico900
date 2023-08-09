#!/bin/bash
pushd ~/home/pico/Pico900
sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f pico900.cfg -f target/rp2040.cfg -c "program /home/ajh2/home/pico/Pico900/build/pico900.elf verify reset exit" -s tcl
popd

