# Custom addressable lightstrip in the iOS Home app using ESP32
Enables control over color and brightness of a strip of WS2812b 
addressable LEDs via iOS natively in the Home app. 

## Components
- ESP32 module
- WS2812b LED strip
- iOS device such as iPhone or iPad
- External power supply for LEDs

## Building
1. Obtain [ESP-IDF](https://github.com/espressif/esp-idf)
  - `git clone https://github.com/espressif/esp-idf.git release/v4.4`
  - `git submodule update --init --recursive`
2. Obtain [ESP-Homekit-SDK](https://github.com/espressif/esp-homekit-sdk)
  - `git clone https://github.com/espressif/esp-homekit-sdk.git`
  - `git submodule update --init --recursive`
3. Clone this repository into the examples directory of ESP-Homekit
  - `cd esp-homekit/examples`
  - `clone https://github.com/andrewteta/lightstrip_homekit.git`
4. Install ESP-IDF VSCode extension
5. Configure ESP-IDF project
6. Connect ESP32 to your computer
7. Run VSCode task "ESP-IDF: Build, Flash, and monitor your program"

## iOS Setup
1. Obtain ESP Provisioning app from iOS App Store
2. Provision device using app and POP code shown in the serial monitor 
output
3. Open iOS Home app and select "Add accessory"
4. Press "More options..."
5. Select device and enter code (default setup code is `11122333`

