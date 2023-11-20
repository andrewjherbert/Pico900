#!/bin/bash
pushd $HOME/Dropbox/pico/Pico900
cd ../openocd/
src/openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f $HOME/Dropbox/pico/Pico900/pico900m.cfg -f target/rp2040.cfg -c "program $HOME/Dropbox/pico/Pico900/build/pico900.elf verify reset exit" -s tcl
popd

