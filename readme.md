# Zephyr producer consumer with threads

- This is a Zephyr RTOS application in freestanding topology that implements a producer / consumer concept
- This project was developed using `STM32F407 Discovery` development kit but it should be able to run in other targets, since there is no "special" feature being used

- The overall structure of the project is:
    - Python script produces serialized data and sends it to embedded target over UART
    - Embedded target received data, process it and produces some sort of output


- The python script in `scripts/stream_sin_wave.py` is responsible for producing the serialized data
    - In this example, the data being produced corresponds to a stream of samples of a sinusoidal wave
    - [Read more about the sinusoidal wave tunning here](scripts/readme.md)
    - The data generated can be generated as json, raw binary protobuf or HDLC frames and it can be forwarded to a file, stdout or to a serial port

- The source code under `src` implements the embedded side code

## How to operate the repository
- Remember to have the zephyr virtual environment active on the current shell
```bash
source ~/zephyrproject/.venv/bin/activate
```

- To format the code base with clang-format:
```bash
./bbuild.sh -f
```

- To build:
```bash
./bbuild.sh -b
```

- To rebuild:
```bash
./bbuild.sh -r
```

- To flash the built binary onto the board:
```bash
./bbuild.sh -l
```

- To format, build and flash:
```bash
./bbuild.sh -f -b -l
```

- To check all options available:
```bash
./bbuild.sh --help
```

- To debug protobuf encapsulation:
```bash
$ python scripts/stream_sin_wave.py -op PROTOBUF_TO_FILE -c channel_20_200_2
# file "batch.bin" will be created...
$ protoc --decode Batch proto/sin_wave.proto < batch.bin 
```

- Once the embedded target is running with the application code and the TTL/USB device is connected between target and host machine, it is time to use the python script to stream a sinusoidal wave to the target
```bash
python scripts/stream_sin_wave.py --loglevel DEBUG --operation HDLC_TO_SERIAL --channel channel_20_20_1 --device /dev/ttyUSB0
```

## This project uses:
- **zephyr rtos** is used as platform
- **clang-format** is used as formatter/code beautifier
- **autopep8** is used to format python scripts
- **python4yahdlc** is a Python binding of the [yahdlc library](https://github.com/bang-olufsen/yahdlc/tree/master), allowing to encode and decode HDLC frames.
    - This was installed within zephyr venv with `pip install --upgrade python4yahdlc`
- **yahdlc** is used as HDLC protocol library in C/C++.
    - Link: [yahdlc - Yet Another HDLC](https://github.com/bang-olufsen/yahdlc/tree/master)

### Versions present in development machine:
- **zephyr:** Zephyr version: 3.4.0 (/home/ggm/zephyrproject/zephyr), build: zephyr-v3.4.0-30-g61824410a9be
- **clang-format:** Ubuntu clang-format version 14.0.0-1ubuntu1
- **autopep8** 2.0.4 (was installed within zephyr venv)
- **python4yahdlc:** 1.3.5 (was installed within zephyr venv)
- **yahdlc** 1.1

## HDLC
- High-Level Data Link Control (HDLC) is a bit-oriented code-transparent synchronous data link layer protocol developed by the International Organization for Standardization (ISO). The standard for HDLC is ISO/IEC 13239:2002.