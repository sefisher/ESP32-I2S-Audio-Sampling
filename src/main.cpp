#include <Arduino.h>        //built-in to esp32 framework
#include <WiFi.h>           //built-in to esp32 framework
#include <WiFiUdp.h>        //built-in to esp32 framework
#include <HTTPClient.h>     //built-in to esp32 framework
#include "secrets.h"        //in "src" - .gitignored - contains wifi credentials
#include "I2SMEMSSampler.h" //in "src"
#include "ADCSampler.h"     //in "src"

//=================================================
// SETUP WIFI AND TRANSPORT MODE
//=================================================

//1. Make sure your WiFi credentials are stored in "secrets.h"

//2. UNCOMMENT *ONE* OF THESE TO SET YOUR AUDIO TRANSPORT OPTION
#define USE_UDP   //1. clientless UDP capture using "nc -lu -p 12333 | aplay -f S16_LE -r 16000"
//#define USE_TCP   //2. capture at TCP receiver using "nc -l -p 12333 | aplay -f S16_LE -r 16000"
//#define USE_HTTP  //3. capture raw files at using "yarn start" on a node server 
                    //   (see https://github.com/atomic14/esp32_audio/tree/master/server/javascript)
#define HOST "192.168.86.98"
#define PORT 12333
#ifdef USE_TCP
  WiFiClient client; //for TCP send
  #define TCP_HOST HOST
  #define TCP_PORT PORT
#endif 
#ifdef USE_UDP
  WiFiUDP Udp; //for UDP send
  #define UDP_HOST HOST
  #define UDP_PORT PORT
#endif 
#ifdef USE_HTTP
  // replace this with your machines IP Address
  //#define ADC_SERVER_URL "http://192.168.1.72:5003/adc_samples"
  //#define I2S_SERVER_URL "http://192.168.1.72:5003/i2s_samples"
  #define ADC_SERVER_URL "http://192.168.86.98:5003/adc_samples"
  #define I2S_SERVER_URL "http://192.168.86.98:5003/i2s_samples"
  WiFiClient *wifiClientADC = NULL;
  HTTPClient *httpClientADC = NULL;
  WiFiClient *wifiClientI2S = NULL;
  HTTPClient *httpClientI2S = NULL;
#endif

//================================================
// SETUP ESP32 BOARD PINOUT FOR MIC
//================================================
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


//================================================
// SAMPLING SETTINGS
//================================================
#define SAMPLE_RATE 16000
#define SAMPLE_FORMAT I2S_COMM_FORMAT_I2S_LSB //Signed Least Significant Bit (aka, Little Endian), S16_LE in aplay

#define NUM_BUF 2
#define BUF_LEN 512
#define SAMPLE_SIZE 8192

//#define NUM_BUF 4
//#define BUF_LEN 1024
//#define SAMPLE_SIZE 16384 //how many samples to read/send at once 

//================================================


// Code Starts ===================
ADCSampler *adcSampler = NULL;
I2SSampler *i2sSampler = NULL;

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

// send data to a remote address
void sendData(WiFiClient *wifiClient, HTTPClient *httpClient, const char *url, uint8_t *bytes, size_t count)
{
  #ifdef USE_HTTP
  // send them off to the server
  digitalWrite(2, HIGH);
  httpClient->begin(*wifiClient, url);
  httpClient->addHeader("content-type", "application/octet-stream");
  httpClient->POST(bytes, count);
  httpClient->end();
  digitalWrite(2, LOW);
  #endif
}

// Task to write samples from ADC to our server
void adcWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
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
  }
}

// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
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
  }
  
}

void setup()
{
  Serial.begin(115200);
  #ifdef USE_TCP
    Serial.println("Using TCP - run a server/listener.");
  #endif 
  #ifdef USE_HTTP
    Serial.println("Using HTTP - run a HTTP server.");
  #endif 
  #ifdef USE_UDP
    Serial.println("Using UDP - run a listener.");
  #endif
  // launch WiFi
  Serial.printf("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
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
    // setup the HTTP Client
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
    //  nc -l -u -p 4444 | aplay -f S16_LE -r 16000
    while (!client.connect(TCP_HOST, TCP_PORT)) {
      Serial.println("connection failed");
      delay(1000);
    }
  #endif 
  #ifdef USE_UDP
    // NOTHING TO SETUP FOR UDP 
    delay(1000);
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
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);

  //start sampling from i2s device
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
}