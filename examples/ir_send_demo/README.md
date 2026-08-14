# ir_send_demo

Cycles through NEC, Sony, Samsung, RC5 and a raw timing array so you can verify
the transmit wiring. Point a phone camera at the LED — most sensors see 940 nm.

Wire an IR LED to GPIO 4 through a transistor (`kIrLedPin` in `main/main.cpp`).

```sh
idf.py set-target esp32
idf.py build flash monitor
```
