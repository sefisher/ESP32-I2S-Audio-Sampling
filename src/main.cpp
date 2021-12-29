#include <Arduino.h>         //built-in to esp32 framework
#include <WiFi.h>            //built-in to esp32 framework
#include <WiFiUdp.h>         //built-in to esp32 framework
#include <HTTPClient.h>      //built-in to esp32 framework
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
//#define USE_TCP   //2. capture at TCP receiver using "nc -l -p 12333 | aplay -f S16_LE -r 16000"
//#define USE_HTTP  //3. capture raw files at using "yarn start" on a node server (see below)
#define USE_HERMES

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
  #include "hermes.h"  //located in "src" it has functions/vars needed for HERMES MQTT
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
  PubSubClient mqttAudioClient(client); 
  long lastReconnectAttempt = 0;
  
  long transmissionTimer = 0;
  long lastTransmissionCompleted = 0;
  bool transmitting = false;
  bool completedAudioTransmission = false;
  int testcount = 0;

  char sessionID[30];
  char clientID[30];

  void setupHermes(){
    sessionID[0] = '\0';clientID[0] = '\0';
    randIDgen(clientID,30);
    Serial.print("Generated Client ID: ");Serial.println(clientID);
    
    if(mqttAudioClient.setBufferSize(4500)){
      Serial.println("Set the max buffer for MQTT messages to 4500.");
    }else{
      Serial.println("Failed to increase the MQTT message buffer (tried to make it 4500.)");
    }

    //mqttAudioClient is the MQTT server
    mqttAudioClient.setServer(MQTTHOST, MQTTPORT);

    // Set up MQTT server:
    Serial.printf("Connecting MQTT: %s, %d\r\n", MQTTHOST, MQTTPORT);
    mqttAudioClient.setServer(MQTTHOST, MQTTPORT);
    mqttAudioClient.connect(SITEID, MQTTUSER, MQTTPASS);

    Serial.print("Waiting for mqttAudioClient..");
    while(!mqttAudioClient.connected()){
      delay(100);
      Serial.print(".");
    }
    Serial.printf("\r\nConnected. Site connected as: %s\r\n",SITEID);
    mqttAudioClient.publish("hermes/audioServer/default/connected", clientID);
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
    if (mqttAudioClient.connect(SITEID, MQTTUSER, MQTTPASS)) {
      Serial.println(" mqttAudioClient-->reconnected.");
      // Once connected, publish an announcement...
      mqttAudioClient.publish("hermes/audioServer/default/connected","Hello - reconnected.");
      // ... and resubscribe
      //mqttAudioClient.subscribe("inTopic");
    }
    return mqttAudioClient.connected();
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
    mqttAudioClient.publish("hermes/asr/startListening", startHeader.c_str(),sizeof(startHeader.c_str()));

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
    mqttAudioClient.publish("hermes/asr/stopListening", endHeader.c_str(),sizeof(endHeader.c_str()));
    Serial.println("Completed hermes-mqtt audio transmission.");
  }

  void sendHermesAudioFrame(int samples_read, int16_t *samples){
    //Pers notes online:
    //Rhasspy needs an audiofeed of 512 bytes+header per message
    //Some devices, like the Matrix Voice do 512 16 bit read in one mic read
    //That would be 1024 bytes, so two message are needed in that case.
    //For our setup this is (readsize=256) * (width=2) ==> 512; so 1 message 
    //is good enough.
    const int messageBytes = samples_read*2 + sizeof(header);
    unsigned char msg[messageBytes] = "";
    //const int message_count = sizeof(samples) / messageBytes;
    //for (int i = 0; i < message_count; i++) {
    String audioFrameHeader = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
      
    memcpy(&msg, &header, sizeof(header));
    memcpy(&msg[sizeof(header)], &samples[0], 512);
    mqttAudioClient.publish(audioFrameHeader.c_str(),msg, sizeof(msg));
      //snprintf (msg, messageBytes, "%ld", value);
      
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
  }

  void sendTestHermesAudioFrame(){
    if(testcount<2){
      testcount++;
      //unsigned char msg[4140];
      String audioFrameHeader = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
      
      switch(testcount){
        case 1:
            Serial.print("RIFF1 size: ");Serial.println(sizeof(RIFF1));
            mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF1, (uint8_t)sizeof(RIFF1));
        break;
        case 2:
            //memcpy(&msg, &RIFF2, sizeof(RIFF2));
            mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF2, (uint8_t)sizeof(RIFF2));
        break;
        case 3:
        break;
        case 4:
        break;
        case 5:
        break;
        case 6:
        break;
        case 7:
        break;
        case 8:
        break;
        case 9:
        break;
        case 10:
        break;
        case 11:
        break;
      }

     //mqttAudioClient.publish(audioFrameHeader.c_str(),msg, sizeof(msg));
        //snprintf (msg, messageBytes, "%ld", value);
        
      if(testcount<2){
        packetCount++;
        Serial.print("AudioFrameTopic: "); Serial.println(audioFrameHeader.c_str());
        Serial.print("; Total Message Size: "); Serial.print(sizeof(RIFF1));
        Serial.println("\r\n---------------------------");
        Serial.println("Printing Part of Message");
        Serial.println("---------------------------");
        printHexArray((uint8_t *)&RIFF1, 96);
        Serial.println("\r\n---------------------------");
      }
    }
  }

// void sendTestHermesAudioFrame2(){
//     if(testcount<1){
//       testcount++;
//       //unsigned char msg[4140];
//       String audioFrameHeader = String("hermes/audioServer/default/") + String(sessionID) + String("/audioSessionFrame");
      
//       //Rhasspy needs an audiofeed of 512 bytes+header per message
//       //Some devices, like the Matrix Voice do 512 16 bit read in one mic read
//       //This is 1024 bytes, so two message are needed in that case
//       const int messageBytes = 512;
//       uint8_t payload[sizeof(header) + messageBytes];
//       const int message_count = sizeof(data) / messageBytes;
//       for (int i = 0; i < message_count; i++) {
//         memcpy(payload, &header, sizeof(header));
//         memcpy(&payload[sizeof(header)], &data[messageBytes * i], messageBytes);
//         audioServer.publish(audioFrameTopic.c_str(),(uint8_t *)payload, sizeof(payload));
//       }
//       switch(testcount){
//         case 1:
//             Serial.print("RIFF1 size: ");Serial.println(sizeof(RIFF1));
//             mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF1, (uint8_t)sizeof(RIFF1));
//         break;
//         case 2:
//             //memcpy(&msg, &RIFF2, sizeof(RIFF2));
//             mqttAudioClient.publish(audioFrameHeader.c_str(),(const uint8_t *)RIFF2, (uint8_t)sizeof(RIFF2));
//         break;
//       }

//      //mqttAudioClient.publish(audioFrameHeader.c_str(),msg, sizeof(msg));
//         //snprintf (msg, messageBytes, "%ld", value);
        
//       if(testcount<2){
//         packetCount++;
//         Serial.print("AudioFrameTopic: "); Serial.println(audioFrameHeader.c_str());
//         Serial.print("; Total Message Size: "); Serial.print(sizeof(RIFF1));
//         Serial.println("\r\n---------------------------");
//         Serial.println("Printing Part of Message");
//         Serial.println("---------------------------");
//         printHexArray((uint8_t *)&RIFF1, 96);
//         Serial.println("\r\n---------------------------");
//       }
//     }
//   }


  //little utilty for printing bytes 
  void printHex(uint8_t num) {
    char hexCar[2];
    sprintf(hexCar, "%02X", num);
    Serial.print(hexCar);
  }
  //utility for printing byte arrays in 32 byte rows
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
      //TODO - copy from I2S to here when working.
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
       if(mqttAudioClient.connected()){
        if(samples_read&&transmitting){
            //sendHermesAudioFrame(samples_read,samples);
            sendTestHermesAudioFrame();
        } else {
          //Loop, because otherwise this causes timeouts
          mqttAudioClient.loop();
        } //if samples_read
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
    long now = millis();
    if(!completedAudioTransmission && !transmitting){
      startHermesTransmission();
    }else if(transmitting){
      //Serial.print(".");
      if(now-transmissionTimer > 500){
        endHermesTransmission();
      }
    }
  #endif
}
