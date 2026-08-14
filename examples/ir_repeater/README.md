# ir_repeater

Captures a message on GPIO 14 and immediately retransmits it on GPIO 4, in the
same protocol. Messages the library cannot identify are replayed from their raw
timings at 38 kHz.

Useful for reaching equipment around a corner, or for bridging a remote to a
device in another room.

```sh
idf.py set-target esp32
idf.py build flash monitor
```

Keep the LED out of the receiver's line of sight, or the repeater will hear
itself.
