# ESP32-I2S-Audio-Sampling
This is a test program to test a few audio transport methods from an ESP32 board.

The audio receiver can be another computer that can run netcat for UDP or TCP transport; or a computer that runs a node server to capture raw audio into a file.

Three modes currently supported, all over your local wifi:

1. Use UDP for clientless streaming and capture (e.g., using "nc -lu -p 12333 | aplay -f S16_LE -r 16000" on a remote computer).
2. Use TCP for a client based receiver using "nc -l -p 12333 | aplay -f S16_LE -r 16000"
3. Use TCP/IP (HTTP) to capture raw files at on a node server. (For my setup I run "yarn start" on the remoted server. See [Atomic14's esp32_audio example] (https://github.com/atomic14/esp32_audio/tree/master/server/javascript) for instructions setting up a simple test server to record the raw audio).

# Setup
1. Rename "secrets.example.h" "secrets.h" and edit the SSID and PASSWORD to match your WiFi setup.
1. In "main.cpp", in the Wifi Settings section: 
    + Uncomment one of {#define USE_UDP, #define USE_TCP, or #define USE_HTTP} in the main.cpp file to select that mode of transport.
    + For TCP or UDP: Edit the "#define HOST" IP address and "#define PORT" number to that of your receiving computer/server.
    + For HTTP: Edit the "#define I2S_SERVER_URL" and/or "#define ADC_SERVER_URL" lines to match your receiving server. (Note:  I2S what is used in the code as posted, if you want to use the ESP32's built-in ADC and an analog mic there is commented out coode to support that in main.cpp.  I haven't tested the ADC function.).
1. Uncomment or create one set of pin connections to match your board/mic setup in the "ESP32 BOARD PINOUT FOR MIC" section.
1. Compile and upload your code onto your device.
1. Run the receiving command on your listening computer and listen using headphones (to prevent feedback).