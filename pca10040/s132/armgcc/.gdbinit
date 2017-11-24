define hook-quit
    set confirm off
end

file _build/dk.out
target remote localhost:2331
monitor interface SWD
monitor endian little
monitor reset 0
monitor flash device=nrf52832
monitor speed 4000
break app_error_fault_handler
load ./_build/dk.out
monitor reset

