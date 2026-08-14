# ac_control

Drives an air conditioner through `IRac`, the vendor-neutral climate API. The
example alternates 24 °C cool, 20 °C cool and off.

Set `kProtocol` in `main/main.cpp` to your unit's protocol — run
`ir_receive_dump` against your original remote to find out which one it is.

Wire an IR LED to GPIO 4 through a transistor.

```sh
idf.py set-target esp32
idf.py build flash monitor
```
