# Zephyr producer consumer with threads



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

## This project uses:
- **zephyr rtos** is used as platform
- **clang-format** is used as formatter/code beautifier
- **python4yahdlc** is a Python binding of the [yahdlc library](https://github.com/bang-olufsen/yahdlc/tree/master), allowing to encode and decode HDLC frames.
    - This was installed within zephyr venv with `pip install --upgrade python4yahdlc`
- **yahdlc** is used as HDLC protocol library.
    - Link: [yahdlc - Yet Another HDLC](https://github.com/bang-olufsen/yahdlc/tree/master)

### Versions present in development machine:
- **zephyr:** Zephyr version: 3.4.0 (/home/ggm/zephyrproject/zephyr), build: zephyr-v3.4.0-30-g61824410a9be
- **clang-format:** Ubuntu clang-format version 14.0.0-1ubuntu1
- **python4yahdlc:** 1.3.5
- **yahdlc** 1.1

## HDLC
- High-Level Data Link Control (HDLC) is a bit-oriented code-transparent synchronous data link layer protocol developed by the International Organization for Standardization (ISO). The standard for HDLC is ISO/IEC 13239:2002.