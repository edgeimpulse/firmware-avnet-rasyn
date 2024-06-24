# firmware-avnet-rasyn
Firmware for RASynboad
https://www.avnet.com/wps/portal/us/products/avnet-boards/avnet-board-families/rasynboard/

The firmware is to be used with the [Evaluation kit](https://www.avnet.com/shop/us/products/avnet-engineering-services/aes-rasynb-120-sk-g-3074457345651740186/)

Project generated using FSP 4.5.0.
The Syntiant SDK in use is v1v110, which is compatible with model generated with TDK v14.1 or with the Syntiant model converter.

## Build with docker

> **Note:** Docker build can be done with MacOs (x86), Windows 10 & 11 and Linux with x86_64 architecture. M1 and M2 Mac or other AArch64 OS are not supported.

1. Build container

    ```
    docker build -t edge-impulse-rasyn .
    ```

1. Build firmware

    ```
    docker run --rm -v $PWD:/app/workspace/firmware-avnet-rasyn edge-impulse-rasyn
    ```

## SD card content
The model and the `mcu_fw_120.synpkg` and `dsp_firmware.synpkg` (for SDK v110) should be copied into the SD card.
There is an additional file which is required to be copied, `config.ini`

## Model
You can load any pre built model (See below how to decide which model is loaded) or use [Edge Impulse](https://studio.edgeimpulse.com/) to build your own model.

### config.ini
To decide which model is loaded in your application, the relevant field are:
* `Mode` is used to decide which `Function` is loaded
* `Port` is used to decide which communication channel is used, 1 for UART and 2 for USB.
