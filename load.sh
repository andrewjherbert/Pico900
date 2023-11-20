#!/bin/bash
pushd $HOME/Dropbox/pico/Pico900
openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f pico900.cfg -f target/rp2040.cfg -c "program $HOME/Dropbox/pico/Pico900/build/pico900.elf verify reset exit" -s tcl
popd

