openocd \
    -f interface/cmsis-dap.cfg \
    -f target/stm32f1x.cfg \
    -c "adapter speed 1000" \
    -c "init" \
    -c "halt" \
    -c "stm32f1x mass_erase 0" \
    -c "program build/product_3/MRMBootloader.bin 0x08000000 verify" \
    -c "shutdown"