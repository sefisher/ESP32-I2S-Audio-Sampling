#ifndef HERMES_LOADED
#define HERMES_LOADED 

#define SITEID "default"

//SELECT THE rawData Array to use and the formatting for streaming audio by HERMES
//#define USE8BIT
//#define USE8BIT_ALT //pick a back up file
//#define USE16BIT

struct wavfile_header {
    char riff_tag[4];       // 4
    int riff_length;        // 4
    char wave_tag[4];       // 4
    char fmt_tag[4];        // 4
    int fmt_length;         // 4
    short audio_format;     // 2
    short num_channels;     // 2
    int sample_rate;        // 4
    int byte_rate;          // 4
    short block_align;      // 2
    short bits_per_sample;  // 2
    char data_tag[4];       // 4
    int data_length;        // 4
};
struct wavfile_header header;

// //TODO: figure out what is needed in topics below and delete the rest

// std::string hotwordToggleOn = "hermes/hotword/toggleOn";
// std::string hotwordToggleOff = "hermes/hotword/toggleOff";
// std::string hotwordDetected = "hermes/hotword/default/detected";

// std::string asrToggleOn = "hermes/asr/toggleOn";
// std::string asrToggleOff = "hermes/asr/toggleOff";
// std::string asrStartListening = "hermes/asr/startListening";
// std::string asrStopListening = "hermes/asr/stopListening";

// std::string audioTopic = SITEID + std::string("/audio");

// // --> hermes/audioServer/<SITE_ID>/audioFrame
// // Replace <SITE_ID> with the site on which the sound frame was captured
// std::string audioFrameTopic = std::string("hermes/audioServer/") + SITEID + std::string("/audioFrame");

// // --> hermes/audioServer/<SITE_ID>/playBytes/<REQUEST_ID>
// // Replace <SITE_ID> with the site on which to play the sound (e.g. default), and <REQUEST_ID> with an id to be passed back on hermes/audioServer/<SITE_ID>/playFinished (see below).
// std::string playBytesTopic = std::string("hermes/audioServer/") + SITEID + std::string("/playBytes/#");
// std::string playFinishedTopic = std::string("hermes/audioServer/") + SITEID + std::string("/playFinished");

// //TTS
// std::string sayTopic = "hermes/tts/say";
// std::string sayFinishedTopic = "hermes/tts/sayFinished";

void initHeader(int readSize, int width, int rate);
void randIDgen(char* sessionStr, int len);

//=====WAVE HEADER FORMATTING=================

// THIS IS COMPLEX.....
// see https://github.com/snipsco/hermes-protocol/blob/develop/hermes/src/ontology/audio_server.rs
// long message_size = 0;
// int queueDelay = 10;
// int sampleRate = 16000;
// int numChannels = 2;
// int bitDepth = 16;

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

void initHeader(int readSize, int width, int rate) {
    strncpy(header.riff_tag, "RIFF", 4);
    strncpy(header.wave_tag, "WAVE", 4);
    strncpy(header.fmt_tag, "fmt ", 4);
    strncpy(header.data_tag, "data", 4);

    header.riff_length = (uint32_t)564;
    header.fmt_length = 16;
    header.audio_format = 1;
    header.num_channels = 1;
    header.sample_rate = rate;
    header.byte_rate = rate * width;
    header.block_align = width;
    header.bits_per_sample = width * 8;
    header.data_length = readSize * width;
}
void printHex(uint8_t num);
void printHexArray(uint8_t* num, int count);
void printHexArray(const uint8_t* num, int count);

#endif