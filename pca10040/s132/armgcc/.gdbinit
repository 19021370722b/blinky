define hook-quit
    set confirm off
end

file _build/nrf52832_xxaa.out
target remote localhost:2331
monitor interface SWD
monitor endian little
monitor reset 0
monitor flash device=nrf52832
monitor speed 4000
break app_error_fault_handler
load ./_build/nrf52832_xxaa.out
monitor reset

