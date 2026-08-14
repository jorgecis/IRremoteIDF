# ir_receive_dump

Decodes anything the library recognises and prints three things: a one-line
summary, the decoded A/C settings when the message is a climate protocol, and a
`uint16_t rawData[]` array you can paste straight into your own code.

Wire a 38 kHz demodulator (TSOP38238 or similar) OUT pin to GPIO 14
(`kRecvPin` in `main/main.cpp`).

```sh
idf.py set-target esp32
idf.py build flash monitor
```

If you see `Buffer overflow`, raise `kCaptureBufferSize`. If long A/C messages
come out truncated, raise `kTimeout`.
