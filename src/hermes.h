#define SITEID "default"

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

//TODO: figure out what is needed in topics below and delete the rest

std::string hotwordToggleOn = "hermes/hotword/toggleOn";
std::string hotwordToggleOff = "hermes/hotword/toggleOff";
std::string hotwordDetected = "hermes/hotword/default/detected";

std::string asrToggleOn = "hermes/asr/toggleOn";
std::string asrToggleOff = "hermes/asr/toggleOff";
std::string asrStartListening = "hermes/asr/startListening";
std::string asrStopListening = "hermes/asr/stopListening";

std::string audioTopic = SITEID + std::string("/audio");

// --> hermes/audioServer/<SITE_ID>/audioFrame
// Replace <SITE_ID> with the site on which the sound frame was captured
std::string audioFrameTopic = std::string("hermes/audioServer/") + SITEID + std::string("/audioFrame");

// --> hermes/audioServer/<SITE_ID>/playBytes/<REQUEST_ID>
// Replace <SITE_ID> with the site on which to play the sound (e.g. default), and <REQUEST_ID> with an id to be passed back on hermes/audioServer/<SITE_ID>/playFinished (see below).
std::string playBytesTopic = std::string("hermes/audioServer/") + SITEID + std::string("/playBytes/#");
std::string playFinishedTopic = std::string("hermes/audioServer/") + SITEID + std::string("/playFinished");

//TTS
std::string sayTopic = "hermes/tts/say";
std::string sayFinishedTopic = "hermes/tts/sayFinished";

void initHeader(int readSize, int width, int rate);
void randIDgen(char* sessionStr, int len);

//=====WAVE HEADER FORMATTING=================

// THIS IS COMPLEX
// see https://github.com/snipsco/hermes-protocol/blob/develop/hermes/src/ontology/audio_server.rs
long message_size = 0;
int queueDelay = 10;
int sampleRate = 16000;
int numChannels = 2;
int bitDepth = 16;

// void initHeader(int readSize, int width, int rate) {
//     strncpy(header.riff_tag, "RIFF", 4);
//     strncpy(header.wave_tag, "WAVE", 4);
//     strncpy(header.fmt_tag, "fmt ", 4);
//     strncpy(header.data_tag, "data", 4);

//     header.riff_length = (uint32_t)sizeof(header) + (readSize * width);
//     header.fmt_length = 16;
//     header.audio_format = 1;
//     header.num_channels = 1;
//     header.sample_rate = rate;
//     header.byte_rate = rate * width;
//     header.block_align = width;
//     header.bits_per_sample = width * 8;
//     header.data_length = readSize * width;
// }
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
