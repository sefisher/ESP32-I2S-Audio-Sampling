#include <Arduino.h>         //built-in to esp32 framework
#include <WiFi.h>            //built-in to esp32 framework
#include <WiFiUdp.h>         //built-in to esp32 framework
#include <HTTPClient.h>      //built-in to esp32 framework

//TODO: Choose a MQTT library and remove the others
#include <AsyncMqttClient.h> // marvinroger/AsyncMqttClient@^0.9.0
#include <PubSubClient.h>    // knolleary/PubSubClient@^2.8

#include "I2SMEMSSampler.h"  //in "src"
#include "ADCSampler.h"      //in "src"
#include "secrets.h"         //in "src" - .gitignored - contains wifi and mqtt credentials

#include "audioFrameSessionData.h" //data for test replay from MQTT sessions

//==========SELECT TRANSPORT MODE========
//1. Make sure your WiFi credentials (& MQTT credentials if using HERMES) are stored in "secrets.h"
//2. Uncomment *ONE* OF THE "#define USE_" STATEMENTS TO PICK AN AUDIO TRANSPORT OPTION
//3. Enter the other related data for that type
//#define USE_UDP   //1. clientless UDP capture using "nc -lu -p 12333 | aplay -f S16_LE -r 16000"
#define USE_TCP   //2. capture at TCP receiver using "nc -l -p 12333 | aplay -f S16_LE -r 16000"
//#define USE_HTTP  //3. capture raw files at using "yarn start" on a node server (see below)
//#define USE_HERMES  //4. This posts audio and other commands to MQTT. See below.

//===SETUP WIFI and TRANSPORT SPECIFICS===

#ifdef USE_TCP
  #define TCP_HOST "192.168.86.98"
  #define TCP_PORT 12333
#endif 

#ifdef USE_UDP
  #define UDP_HOST "192.168.86.98"
  #define UDP_PORT 12333
#endif 

#ifdef USE_HTTP
  // (See https://github.com/atomic14/esp32_audio/tree/master/server/javascript for 
  // an example on setting up a node server that accepts the raw audio.)

  // Replace this with your machines IP Address.
  #define ADC_SERVER_URL "http://192.168.86.98:5003/adc_samples"
  #define I2S_SERVER_URL "http://192.168.86.98:5003/i2s_samples"
#endif

#ifdef USE_HERMES

  //What is working with HERMES:
  // turn on RAWSAMPLETEST and PUBSUBMQTT below.  It will stream audio via MQTT and it seems like
  // the Rhasspy server is catching it.
  //You can run "mosquitto_sub -L mqtt://<username for mqtt server>:<pass for mqtt server>@192.168.86.102:1883/hermes/# -v" to see the MQTT traffic.
  //If you modify the call to sendTestHermesAudioFrameBySamplingRawAudio to:
  //    sendTestHermesAudioFrameBySamplingRawAudio(512, true);
  // you can see the playBytes test running using "hermes-audio-player" - but it doesn't seem to actually work
  
  #include "hermes.h"  //located in "src" it has functions/vars needed for HERMES MQTT
  //Testing to find the right MQTT library
  //Choose a MQTT library:
  #define PUBSUBMQTT
  //#define ASYNCMQTT
  
  //if you want to sample raw audio stored in audioFrameSessionData.h uncomment this:
  #define RAWSAMPLETEST //does a sampling test from raw audio - ***requires you to also uncomment #define PUBSUBMQTT

  //Note: SITEID is defined as "default" in hermes.h
  //Note: MQTT USERNAME/PASSWORD is set in "secrets.h"
  #define MQTTHOST "192.168.86.102"
  #define MQTTPORT 1883
#endif

//===========SETUP ESP32 BOARD PINOUT FOR MIC============

//Steve's Dev Board Setup:
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT

//Brian's Custom Board Build:
//#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
// #define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
// #define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_25  //THIS IS WS on INMP441
// #define I2S_MIC_SERIAL_DATA GPIO_NUM_27

//================= SAMPLING SETTINGS =================

#define SAMPLE_RATE 16000
#define SAMPLE_FORMAT I2S_COMM_FORMAT_I2S_LSB //Signed Least Significant Bit (aka, Little Endian), S16_LE in aplay

#define NUM_BUF 2
#define BUF_LEN 512
#define SAMPLE_SIZE 256 //Rhasspy needs an audiofeed of 512 bytes+header per message (256 samples *2 bytes per value)

//#define NUM_BUF 4
//#define BUF_LEN 1024
//#define SAMPLE_SIZE 16384 //how many samples to read/send at once 


//================ Code Starts Here ===================

ADCSampler *adcSampler = NULL;
I2SSampler *i2sSampler = NULL;

bool firstpacket = true;
long packetCount = 0;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = SAMPLE_FORMAT,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = NUM_BUF,
    .dma_buf_len = BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s config for reading from left channel of I2S
i2s_config_t i2sMemsConfigLeftChannel = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, //read as 32 bit but converted to 16 bit - deals with bug in ESP32
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = NUM_BUF,
    .dma_buf_len = BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2sPins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

#ifdef USE_HTTP
  WiFiClient *wifiClientADC = NULL;
  HTTPClient *httpClientADC = NULL;
  WiFiClient *wifiClientI2S = NULL;
  HTTPClient *httpClientI2S = NULL;
  // send data to a remote address
  void sendData(WiFiClient *wifiClient, HTTPClient *httpClient, const char *url, uint8_t *bytes, size_t count)
  {
    // send them off to the server
    digitalWrite(2, HIGH);
    httpClient->begin(*wifiClient, url);
    httpClient->addHeader("content-type", "application/octet-stream");
    httpClient->POST(bytes, count);
    httpClient->end();
    digitalWrite(2, LOW);
  }
#endif

#ifdef USE_TCP
  WiFiClient client; //for TCP send
#endif 

#ifdef USE_UDP
  WiFiUDP Udp; //for UDP send
#endif

#ifdef USE_HERMES
  
  WiFiClient client; //for comms over wifi

  long lastReconnectAttempt = 0;
  
  long transmissionTimer = 0;
  long lastTransmissionCompleted = 0;
  bool transmitting = false;
  bool completedAudioTransmission = false;
  int testcount = 0;

  char sessionID[30];
  char clientID[30];

  //Define the selected mqtt client
  #ifdef PUBSUBMQTT
  PubSubClient mqttAudioClient(client);
  #endif
  #ifdef ASYNCMQTT
  AsyncMqttClient asyncMQTTclient; 
  #endif

  void setupHermes(){
    sessionID[0] = '\0';clientID[0] = '\0';
    randIDgen(clientID,30);
    Serial.print("Generated Client ID: ");Serial.println(clientID);
    
  // Set up MQTT server:
    Serial.printf("Connecting MQTT: %s, %d\r\n", MQTTHOST, MQTTPORT);

  #ifdef PUBSUBMQTT
  //Initialize PUBSUBMQTT Variant======================
    mqttAudioClient.setServer(MQTTHOST, MQTTPORT);
    mqttAudioClient.connect(SITEID, MQTTUSER, MQTTPASS);
    Serial.print("Waiting for mqttAudioClient..");
    while(!mqttAudioClient.connected()){
      delay(100);
      Serial.print(".");
    }
    Serial.printf("\r\nmqttAudioClient connected. Site connected as: %s\r\n",SITEID);
    if(mqttAudioClient.setBufferSize(5500)){
      Serial.println("Set the max buffer for MQTT messages to 5500.");
    }else{
      Serial.println("Failed to increase the MQTT message buffer (tried to make it 4500.)");
    }
    mqttAudioClient.publish("hermes/audioServer/default/connected", clientID);
  //=======================================
  #endif

  #ifdef ASYNCMQTT
  //Initialize ASYNCMQTT Variant======================
    asyncMQTTclient.setCredentials(MQTTUSER,MQTTPASS);
    asyncMQTTclient.setServer(MQTTHOST,MQTTPORT);
    asyncMQTTclient.connect();

    Serial.print("Waiting for asyncMQTTclient..");
    while(!asyncMQTTclient.connected()){
      delay(100);
      Serial.print(".");
    }
      const char topic[] = "hermes/audioServer/default/connected";
    Serial.printf("\r\nasyncMQTTclient connected. Site connected as: %s\r\n",SITEID);
    asyncMQTTclient.publish(topic, 0, false, clientID);
  //=======================================
  #endif

    lastReconnectAttempt = 0;

    //set up the wave format header
    initHeader(SAMPLE_SIZE, 2, SAMPLE_RATE);

    /* 
    2) Trigger "Hotword Detected" by posting to topic:
    ---------------------
    if (device->isHotwordDetected() && !hotwordDetected) {
          hotwordDetected = true;
          //start session by publishing a message to hermes/dialogueManager/startSession
          std::string message = "{\"init\":{\"type\":\"action\",\"canBeEnqueued\": false},\"siteId\":\"" + std::string(config.siteid) + "\"}";
          asyncClient.publish("hermes/dialogueManager/startSession", 0, false, message.c_str());
      }
    -----------------------

    3)  Publish the streaming audio for 3 seconds at a time.
    */
  }

  //reconnect to HERMES server if disconnected
  bool reconnect() {
  #ifdef PUBSUBMQTT
    if (mqttAudioClient.connect(SITEID, MQTTUSER, MQTTPASS)) {
      Serial.println(" mqttAudioClient-->reconnected.");
      mqttAudioClient.publish("hermes/audioServer/default/connected","Hello - reconnected.");
    }
    return mqttAudioClient.connected();
  #endif
  #ifdef ASYNCMQTT
    asyncMQTTclient.connect();
    if (asyncMQTTclient.connected()) {
      Serial.println(" asyncMQTTclient-->reconnected.");
      // Once connected, publish an announcement...
      char topic[] = "hermes/audioServer/default/connected";
      asyncMQTTclient.publish(topic, 0, false,"Hello - reconnected.");
    }
    return asyncMQTTclient.connected();
  #endif
  }
 
  //generate session ID for hermes
  void randIDgen(char* sessionStr, int len){
   for(int i=0;i<(len-1);i++){
      byte randomValue = random(0, 36);
      if(randomValue >= 26){
        sessionStr[i] = (randomValue - 26) + '0';
      }else{
        sessionStr[i] = randomValue + 'a';
      }
   }
   sessionStr[len-1] = '\0';
  }
  void startHermesTransmission(){
    Serial.println("Starting to transmit audio via hermes-mqtt.");
    transmitting = true;
    transmissionTimer = millis();
    randIDgen(sessionID,30);
    String startHeader = String("{\"siteId\": \"default\",\"sessionId\": \"") + String(sessionID) + String("\",\"lang\": null,\"stopOnSilence\": false,\"sendAudioCaptured\": true,\"wakewordId\": null,\"intentFilter\": null}");
    const char topic[] = "hermes/asr/startListening";
    #ifdef PUBSUBMQTT
    mqttAudioClient.publish(topic, startHeader.c_str(),sizeof(startHeader.c_str()));
    #endif
    #ifdef ASYNCMQTT
    asyncMQTTclient.publish(topic, 0, false, startHeader.c_str());
    #endif
    /*TODO - 
      hermes/asr/startListening - startHeader;
      [send all]->hermes/audioServer/default/<sessionID>/audioSessionFrame - RIFF message; 
      hermes/asr/stopListening - endHeader;
     */
  }
  void endHermesTransmission(){
    completedAudioTransmission = true;
    transmitting=false;
    lastTransmissionCompleted = millis();
    String endHeader = String("{\"siteId\": \"default\",\"sessionId\": \"") + String(sessionID) + String("\"}");
    const char topic[] = "hermes/asr/stopListening";
    #ifdef PUBSUBMQTT
    mqttAudioClient.publish(topic, endHeader.c_str(),sizeof(endHeader.c_str()));
    #endif
    #ifdef ASYNCMQTT
    asyncMQTTclient.publish(topic, 0, false, endHeader.c_str());
    #endif
    Serial.println("Completed hermes-mqtt audio transmission.");
  }

  void sendHermesAudioFrame(int samples_read, int16_t *samples){
    //Pers notes online:
    //Rhasspy needs an audiofeed of 512 bytes+header per message
    //Some devices, like the Matrix Voice do 512 16 bit read in one mic read
    //That would be 1024 bytes, so two message are needed in that case.
    //For our setup this is (readsize=256) * (width=2) ==> 512; so 1 message 
    //is good enough.

    //const int message_count = sizeof(samples) / messageBytes;
    //for (int i = 0; i < message_count; i++) {
    String audioFrameHeader = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
    
    #ifdef PUBSUBMQTT  
    const int messageBytes = samples_read*2 + sizeof(header);
    unsigned char msg[messageBytes] = "";
    Serial.println("Trying PUBSUBMQTT audioFrameTransmission:");
    Serial.println(audioFrameHeader);
    //Serial.println(msg);
    memcpy(&msg, &header, sizeof(header));
    memcpy(&msg[sizeof(header)], &samples[0], 512);
    mqttAudioClient.publish(audioFrameHeader.c_str(),msg, sizeof(msg));
    if(packetCount<1){
      packetCount++;
      Serial.print("AudioFrameTopic: "); Serial.println(audioFrameHeader.c_str());
      Serial.print("Audio Samples Read: "); Serial.print(samples_read);
      Serial.print("; Audio Sample Size: "); Serial.print(sizeof((const uint8_t *)&samples[0]));
      Serial.print("; Audio header (size="); Serial.print(sizeof(header)); Serial.println("):"); 
      Serial.print(header.riff_tag);Serial.print(header.riff_length);Serial.print(header.wave_tag);
      Serial.print(header.fmt_tag);Serial.print(header.fmt_length);Serial.print(header.audio_format);
      Serial.print(header.num_channels);Serial.print(header.sample_rate);Serial.print(header.byte_rate);
      Serial.print(header.block_align);Serial.print( header.bits_per_sample);
      Serial.print(header.data_tag);Serial.println(header.data_length);
      Serial.print("Samples Size: "); Serial.print(samples_read * 2);
      Serial.print("; Total Message Size: "); Serial.print(sizeof(msg));
      Serial.println("\r\n---------------------------");
      Serial.println("Printing Header");
      Serial.println("---------------------------");
      printHexArray((uint8_t *)&header,messageBytes);
      Serial.println("\r\n---------------------------");
      Serial.println("Printing Message");
      Serial.println("---------------------------");
      printHexArray((uint8_t *)&msg,messageBytes);
      Serial.println("\r\n---------------------------");
    }
    #endif
    
    #ifdef ASYNCMQTT
    Serial.println("Trying ASYNCMQTT audioFrameTransmission:");
    Serial.println(audioFrameHeader);
    Serial.println((const char *) samples);
   
    asyncMQTTclient.publish(audioFrameHeader.c_str(), 0, false, (const char *) samples);
    #endif

      //snprintf (msg, messageBytes, "%ld", value);
      
    
  }

  #ifdef RAWSAMPLETEST
  #define MAXCHUNKSIZE 5500
  //THIS FUNCTION TESTS SAMPLING AND SENDING RAW AUDIO IN CHUNKS
  //**THIS FUNCTION IS SET UP FOR PUBSUBCLIENT ONLY

  //set playBytes to true to stream for just testing audio being streamed to a hermes_audio_player.
  //--->"hermes/audioServer/default/playBytes/
  //set playBytes to false for sending chunks in audioFrameSession for processing by Rhasspy
  //--->"hermes/audioServer/default/" + sessionID + "/audioSessionFrame"
  // If use8bit==true this processes the unsigned int, 8bit rawData[] data set, if it is processed 
  // as signed int 16bit data.
  void sendTestHermesAudioFrameBySamplingRawAudio(int waveChunkSize, bool playBytes = false){
    #ifdef USE8BIT 
    bool use8bit = true;
    #else
    bool use8bit = false;
    #endif
    // Note on data formatting: Whether using 8 bits or 16 bits in the audio stream, 
    // we are just going to send the data around here in 8 bit chunks.
    // (Reminder: 0xFF is 2 hexidecimal digits which is 8 bits or 1 byte. 
    // So these arrays are an array of single bytes and uint8_t is also a 
    // single byte; and uint_8_t is identical to unsigned char in ESP32).)
    
    //audioChunk[] this is for pulling from rawData[] with size 45294 for 16bit or 22647 for 8bit; 44 is for header
    Serial.println("Steps 1.");
    uint8_t audioChunk[MAXCHUNKSIZE+44]; 
    int totalLength,chunkCount,numberOfChunks;
    String audioFrameTopic;

    /* 
    From turnoffthedesklamp-downsample-U8b-Left.wav

    waveheader8[4-7] needs to be updated to be (uint32_t)(waveChunkSize*sizeof(uint8_t)+36)
    waveheader8[40-43] needs to be updated to be (uint32_t)(waveChunkSize*sizeof(uint8_t))
    */
     uint8_t waveheader8[44] = {
      0x52, 0x49, 0x46, 0x46, //R I F F
      0x9B, 0x58, 0x00, 0x00, //22683 = Length of "file" (32 bit integer) excluding first 8 header bytes (22647 + 44(header) - 8(first 8) for full array)
      0x57, 0x41, 0x56, 0x45, //W A V E
      0x66, 0x6D, 0x74, 0x20, //fmt_
      0x10, 0x00, 0x00, 0x00, //16 - number of bytes in above format data packet
      0x01, 0x00, //PCM format (2 byte int)
      0x01, 0x00, //1 channel
      0x80, 0x3E, 0x00, 0x00, //16000 (Sample Rate - 32 bit integer). 
      0x80, 0x3E, 0x00, 0x00, //16000 (Bit Rate = (Sample Rate * BitsPerSample * Channels) / 8).
      0x01, 0x00, //bitsPerSample*Channels --> 1 - 8 bit mono; 2 - 8 bit stereo/16 bit mono; 
      0x08, 0x00, //8 Bits per sample 
      0x64, 0x61, 0x74, 0x61, //d a t a
      0x77, 0x58, 0x00, 0x00  //22647 (Size of the data section - 32 bit integer)
    };

    /* 
    From turnoffthedesklamp-downsample-S16b-Left.wav 
    */
    uint8_t waveheader16[44] = {
      0x52, 0x49, 0x46, 0x46, //R I F F
      0x12, 0xB1, 0x00, 0x00, //45330 = Length of "file" (32 bit integer) excluding first 8 header bytes (45294 + 44(header) - 8(first 8) for full array)
      0x57, 0x41, 0x56, 0x45, //W A V E
      0x66, 0x6D, 0x74, 0x20, //fmt_ 
      0x10, 0x00, 0x00, 0x00, //16 - number of bytes in above format data packet
      0x01, 0x00, //PCM format (2 byte int)
      0x01, 0x00, //1 channel
      0x80, 0x3E, 0x00, 0x00, //16000 (Sample Rate - 32 bit integer).  
      0x00, 0x7D, 0x00, 0x00, //32000 (Bit Rate = (Sample Rate * BitsPerSample * Channels) / 8).
      0x02, 0x00, //bitsPerSample*Channels --> 1 - 8 bit mono; 2 - 8 bit stereo/16 bit mono; 
      0x10, 0x00, //16 Bits per sample 
      0x64, 0x61, 0x74, 0x61, //d a t a
      0xEE, 0xB0, 0x00, 0x00  //45294 (Size of the data section - 32 bit integer)
    };
    
    Serial.print("2.");
    //Prepare header
    unsigned long x; //part size to convert to byte array
    uint8_t *headerArrayPtr;
    uint8_t *currentPtr;

    if(use8bit){
      headerArrayPtr = waveheader8;
    }else{
      headerArrayPtr = waveheader16;
    }
    //waveheaderXX[4-7] needs to be updated to be (uint32_t)((waveChunkSize*sizeof(uint8_t)))/sizeof(uint16_t) + 36);
    x=( (waveChunkSize*sizeof(uint8_t)) / sizeof(uint16_t) + 36);
    headerArrayPtr[7] = static_cast<unsigned char>((x & 0xFF000000) >> 24);
    headerArrayPtr[6] = static_cast<unsigned char>((x & 0x00FF0000) >> 16);
    headerArrayPtr[5] = static_cast<unsigned char>((x & 0x0000FF00) >> 8); 
    headerArrayPtr[4] = static_cast<unsigned char>(x & 0x000000FF); 
    Serial.println("\r\nFile size calc:");
    printHexArray(&headerArrayPtr[4],4); 
    
    //waveheaderXX[40-43] needs to be updated to be (uint32_t)((waveChunkSize*sizeof(uint8_t))/sizeof(uint16_t))
    x=(x-36);
    headerArrayPtr[43] = static_cast<unsigned char>((x & 0xFF000000) >> 24);
    headerArrayPtr[42] = static_cast<unsigned char>((x & 0x00FF0000) >> 16);  
    headerArrayPtr[41] = static_cast<unsigned char>((x & 0x0000FF00) >> 8);   
    headerArrayPtr[40] = static_cast<unsigned char>(x & 0x000000FF); 
    Serial.println("\r\nData size calc:");
    printHexArray(&headerArrayPtr[40],4);
    
    Serial.println("\r\nPrinting Wave Header:");
    printHexArray(headerArrayPtr,44);

    Serial.printf("\r\nPreps completed. File length calc: %lu (%04x). Data section length calc: %lu (%04x).\r\n",x+36,x+36,x,x);
    if(use8bit) {totalLength = 22647;}else{totalLength = 45294;}
    numberOfChunks = totalLength/waveChunkSize;
    
    Serial.printf("\r\n========sendTestHermesAudioFrameBySamplingRawAudio==========\r\n");
    Serial.printf("USE8BIT=%d; totalLength=%d;\r\nMAXCHUNKSIZE=%d; waveChunkSize=%d; numberOfChunks=%d\r\n", use8bit,totalLength,MAXCHUNKSIZE,waveChunkSize,numberOfChunks);

    //Note: last chunk (e.g., chunkCount==numberOfChunks) will have any remainder that didn't fit
    //TODO - deal with the last chunk
    //for(chunkCount=0;chunkCount <= numberOfChunks;chunkCount++){}
    
    for(chunkCount=0;chunkCount < numberOfChunks;chunkCount++){
      //get a chunk
      Serial.printf("Chunk number: %d of %d. Begins with:\r\n",chunkCount,numberOfChunks);
      // std::copy(headerArrayPtr, headerArrayPtr + 44, audioChunk);
      // std::copy(rawData + chunkCount*waveChunkSize, rawData + chunkCount*(waveChunkSize+1) , audioChunk + 44);
      currentPtr = &rawData[chunkCount*waveChunkSize];
      memcpy(&audioChunk[0], &headerArrayPtr[0], 44*sizeof(uint8_t));
      memcpy(&audioChunk[44], currentPtr, waveChunkSize*sizeof(uint8_t));
      printHexArray(&audioChunk[0],12);
      Serial.printf("Audio Data Begins at %d with:\r\n", chunkCount*waveChunkSize);
      printHexArray(&audioChunk[44],8);

      //start off
      if(chunkCount==0){ //if first chunk, kick things off for audioSessionFrame mode
        if(playBytes){
          randIDgen(sessionID,30);
        }else{
          startHermesTransmission();
        }
      }

      //Set MQTT topic
      if(playBytes){
        
        //For streaming test===============
        //audioFrameTopic = String("hermes/audioServer/default/playBytesStreaming/")+String(sessionID)+String("/")+String(chunkCount);
        //if(chunkCount==numberOfChunks-1){audioFrameTopic = audioFrameTopic + String("/1");}else{audioFrameTopic = audioFrameTopic + String("/0");}
        //=================================

        //For single playBytes test========
        audioFrameTopic = String("hermes/audioServer/default/playBytes/")+String(sessionID);
        //=================================
      }else{
        audioFrameTopic = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
      }
      
      //Publish the message binary
      mqttAudioClient.publish(audioFrameTopic.c_str(),(const uint8_t *)audioChunk, (unsigned int)(sizeof(uint8_t)*(44+waveChunkSize)));

      //Send message that we are done
      if(chunkCount==numberOfChunks-1){
        if(playBytes){
          // THIS SEEMS TO NOT BE NEEDED:
          // audioFrameTopic = String("{\"id\": \"")+String(sessionID)+String("\", \"sessionId\": \"")+String(sessionID)+String("\"}");
          // mqttAudioClient.publish("hermes/audioServer/default/playFinished",audioFrameTopic.c_str());
          //============================
        }else {
          endHermesTransmission();
        }
      }
    }
  }
  #endif //RAWSAMPLETEST
  
  //THIS TESTS SENDING THE RIFFS IN THE 4096 BIT CHUNKS
  void sendTestHermesChunkedAudioFrame(){
    String audioFrameHeader;
    
    if(testcount<11){
      audioFrameHeader = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
      Serial.println((const char *) RIFF1);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF1, (uint16_t)sizeof(RIFF1));

        Serial.print("RIFF size: ");Serial.println(sizeof(RIFF2));
        #ifdef PUBSUBMQTT
            Serial.println("Trying PUBSUBMQTT audioFrameTransmission:");
            Serial.println(audioFrameHeader);
            switch(testcount){
              case 0:
              Serial.println((const char *) RIFF1);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF1, (uint16_t)sizeof(RIFF1));
              Serial.print("AudioFrameTopic: "); Serial.println(audioFrameHeader.c_str());
              Serial.print("Total Message Size: "); Serial.print(sizeof(RIFF2));
              Serial.println("\r\n---------------------------");
              Serial.println("Printing Part of Message");
              Serial.println("---------------------------");
              printHexArray((uint8_t *)&RIFF2, 96);
              Serial.println("\r\n---------------------------");              
              break;
              case 1:Serial.println((const char *) RIFF2);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF2, (uint16_t)sizeof(RIFF2));break;
              case 2:Serial.println((const char *) RIFF3);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF3, (uint16_t)sizeof(RIFF3));break;
              case 3:Serial.println((const char *) RIFF4);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF4, (uint16_t)sizeof(RIFF4));break;
              case 4:Serial.println((const char *) RIFF5);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF5, (uint16_t)sizeof(RIFF5));break;
              case 5:Serial.println((const char *) RIFF6);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF6, (uint16_t)sizeof(RIFF6));break;
              case 6:Serial.println((const char *) RIFF7);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF7, (uint16_t)sizeof(RIFF7));break;
              case 7:Serial.println((const char *) RIFF8);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF8, (uint16_t)sizeof(RIFF8));break;
              case 8:Serial.println((const char *) RIFF9);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF9, (uint16_t)sizeof(RIFF9));break;
              case 9:Serial.println((const char *) RIFF10);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF10, (uint16_t)sizeof(RIFF10));break;
              case 10:Serial.println((const char *) RIFF11);mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF11, (uint16_t)sizeof(RIFF11));break;
            }
        #endif
        #ifdef ASYNCMQTT
            Serial.println("Trying ASYNCMQTT audioFrameTransmission:");
            Serial.println(audioFrameHeader);
            Serial.println((const char *)RIFF2);
            asyncMQTTclient.publish(audioFrameHeader.c_str(),0,false, (void *) RIFF2, (uint16_t)sizeof(RIFF2));
        #endif
        testcount++;
      }
    }

  //little utilty for printing bytes 
  void printHex(uint8_t num) {
    char hexCar[2];
    sprintf(hexCar, "%02X", num);
    Serial.print(hexCar);
  }
  //utility for printing byte arrays in 32 bit (4 byte, or 8 hex characters) rows
  void printHexArray(uint8_t* num, int count){
    for(int i=0;i<count;i++){
      printHex(num[i]);
      if((i+1)%4==0){
        Serial.println(" ");
      }else{
        Serial.print(" ");
      }
    }
  }
  
#endif 

// Task to write samples from ADC to our server
// I'm using I2S so I haven't tested this function
void adcWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  // if(firstpacket){
  //   Serial.print("Packet Size: ");
  //   Serial.print(SAMPLE_SIZE * sizeof(uint16_t)); 
  //   Serial.println(" bytes.");
  //   firstpacket=false;
  // }
  if (!samples)
  {
    Serial.println("Failed to allocate memory for samples");
    return;
  }
  while (true)
  {
    int samples_read = sampler->read(samples, SAMPLE_SIZE);
    #ifdef USE_HTTP
      //sending data to HTTP server
      sendData(wifiClientADC, httpClientADC, ADC_SERVER_URL, (uint8_t *)samples, samples_read * sizeof(uint16_t));
    #endif 
    #ifdef USE_TCP
      // sending the buffer to our TCP listener
      client.write((const uint8_t *)samples, samples_read * sizeof(uint16_t)); 
    #endif 
    #ifdef USE_UDP
      Udp.beginPacket(UDP_HOST, UDP_PORT);
      Udp.write((const uint8_t *)samples, samples_read * sizeof(uint16_t));
      Udp.endPacket();
    #endif
    #ifdef USE_HERMES
      //TODO - copy from i2sMemsWriterTask to here when that working.
    #endif
  }
}

// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  // if(firstpacket){
  //   Serial.print("Packet Size: ");
  //   Serial.print(SAMPLE_SIZE * sizeof(uint16_t)); 
  //   Serial.println(" bytes.");
  //   firstpacket=false;
  // }
  if (!samples)
  {
    Serial.println("Failed to allocate memory for samples");
    return;
  }
  while (true)
  {
    int samples_read = sampler->read(samples, SAMPLE_SIZE);
    #ifdef USE_HTTP
      sendData(wifiClientI2S, httpClientI2S, I2S_SERVER_URL, (uint8_t *)samples, samples_read * sizeof(uint16_t));
    #endif 
    #ifdef USE_TCP
      // sending the buffer to our TCP listener
      client.write((const uint8_t *)samples, samples_read * sizeof(uint16_t)); 
    #endif 
    #ifdef USE_UDP
      Udp.beginPacket(UDP_HOST, UDP_PORT);
      Udp.write((const uint8_t *)samples, samples_read * sizeof(uint16_t));
      Udp.endPacket();
    #endif
    #ifdef USE_HERMES
      #ifdef PUBSUBMQTT
       if(mqttAudioClient.connected()){
      #endif
      #ifdef ASYNCMQTT
       if(asyncMQTTclient.connected()){
      #endif
        if(samples_read&&transmitting){
            
            //TODO - the tests aren'tr working ; need to fix and then put in
            // in the streaming.
            
            //sendHermesAudioFrame(samples_read,samples);
            //sendTestHermesChunkedAudioFrame();
            //sendTestHermesAudioFrameBySamplingRawAudio(512);
        } 
        #ifdef PUBSUBMQTT
        else {
          //Loop, because otherwise this causes timeouts
          mqttAudioClient.loop();
        } 
        #endif
      } else {
        long now = millis();
        Serial.println("mqttAudioClient-->disconnected.");
        if (now - lastReconnectAttempt > 5000) {
          lastReconnectAttempt = now;
          // Attempt to reconnect
          if (reconnect()) {
            lastReconnectAttempt = 0;
          }
        }

      }  //if audioServer.connected
    #endif
  }
  
}

void setup()
{
  Serial.begin(115200);
  #ifdef USE_TCP
    Serial.println("=================================\r\nUsing TCP - run a server-listener.\r\n=================================");
  #endif 
  #ifdef USE_HTTP
    Serial.println("=================================\r\nUsing HTTP - run a HTTP server.\r\n=================================");
  #endif 
  #ifdef USE_UDP
    Serial.println("=================================\r\nUsing UDP - run a listener.\r\n=================================");
  #endif
  #ifdef USE_HERMES
    Serial.println("=================================\r\nUsing HERMES - run a Hermes Audio Player.\r\n=================================");
  #endif
  // launch WiFi
  Serial.printf("Connecting to WiFi.");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("Started up");
  // indicator LED
  pinMode(2, OUTPUT);

  #ifdef USE_HTTP
    // SETUP THE HTTP Client
    // this will time out if you aren't running a server to capture via HTTP
    // see node/yarn example on githib atomic14 https://github.com/atomic14/esp32_audio/tree/master/server/javascript
    wifiClientADC = new WiFiClient();
    httpClientADC = new HTTPClient();
    wifiClientI2S = new WiFiClient();
    httpClientI2S = new HTTPClient();
  #endif 
  #ifdef USE_TCP
    // SETUP THE TCP CLIENT
    // this will time out if on the host machine a listener isn't set up:
    //  nc -l -u -p 12333 | aplay -f S16_LE -r 16000
    while (!client.connect(TCP_HOST, TCP_PORT)) {
      Serial.println("connection failed");
      delay(1000);
    }
  #endif 
  #ifdef USE_UDP
    // NOTHING TO SETUP FOR UDP 
    delay(1000);
  #endif
  #ifdef USE_HERMES
    setupHermes();
  #endif
  // input from analog microphones such as the MAX9814 or MAX4466
  // internal analog to digital converter sampling using i2s
  // create our samplers
  // adcSampler = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_7, adcI2SConfig);

  // set up the adc sample writer task
  // TaskHandle_t adcWriterTaskHandle;
  // adcSampler->start();
  // xTaskCreatePinnedToCore(adcWriterTask, "ADC Writer Task", 4096, adcSampler, 1, &adcWriterTaskHandle, 1);

  // Direct i2s input from INMP441 or the SPH0645
  i2sSampler = new I2SMEMSSampler(I2S_NUM_0, i2sPins, i2sMemsConfigLeftChannel, false);
  i2sSampler->start();
  // set up the i2s sample writer task
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 8192, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
  #ifdef USE_HERMES
    #ifndef RAWSAMPLETEST
      long now = millis();
      if(!completedAudioTransmission && !transmitting){
        startHermesTransmission();
      }else if(transmitting){
        //Serial.print(".");
        if(now-transmissionTimer > 500){
          endHermesTransmission();
        }
      }
    #else
      if(lastTransmissionCompleted==0){
        lastTransmissionCompleted = millis();
        Serial.println("\r\n====\r\nRunning the Raw Sample Test.\r\nThis is a single shot running through a built in array of raw audio data.\r\n======");
        sendTestHermesAudioFrameBySamplingRawAudio(512,false);
      }
    #endif
  #endif
}
