# truetouch-firmware
Firmware for the TrueTouch VR haptic glove.

## Nordic SDK Version
This project uses SDK version: **7.0.2**.

## Development Environment Setup
This project uses Segger Embedded Studio (SES) as its development environment. To get the project up and building, follow these steps:
1. Install [Segger Embedded Studio](https://www.segger.com/products/development-tools/embedded-studio/)
2. Download the appropriate [Nordic SDK](https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK) (see [Nordic SDK Version](#Nordic-SDK-Version) for version used in this project)
3. Open the project (SES project file is located at `ses/truetouch.emProject`)
4. Create a global macro in SES called `SDK_DIR` that contains the root directory of the Nordic SDK
    * See [this tutorial](https://studio.segger.com/index.htm?https://studio.segger.com/ide_project_macros.htm) for how to make global macros
    * For example if the SDK root directory is "C:\nordic\SDK\17_0_2", the global macro would be: `SDK_DIR=C:\nordic\SDK\17_0_2`
5. Build the project
