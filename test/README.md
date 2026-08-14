# Host unit tests

The upstream IRremoteESP8266 test suite, run against the ported sources. It
compiles the library with `-DUNIT_TEST`, which swaps the hardware calls for
simulated ones, so the protocol encoders and decoders are exercised end to end
on your workstation — no ESP32 required.

```sh
# once
git clone -b v1.12.x --depth 1 https://github.com/google/googletest.git ../lib/googletest

make -j"$(nproc)" run       # build and run everything
make run-ir_Daikin          # one file
make clean
```

83 binaries, 1669 test cases.
