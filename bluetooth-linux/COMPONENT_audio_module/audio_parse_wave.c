#include "stdio.h"
#include "string.h"
#if (!defined(_WIN32) && !defined(_WIN64))
#include <unistd.h>
#endif
#include <stdint.h>
#include <stdbool.h>
#define NUMBER_OF_MAX_SAMPLES 2606952
#include "audio_parse_wave.h"
#include "wiced_bt_trace.h"

#include "log.h"
// #define WICED_BT_TRACE printf

//#define SAMPLE_WIDTH 2

//static uint8_t LEFT_DATA_BUF[NUMBER_OF_MAX_SAMPLES*SAMPLE_WIDTH];
//static uint8_t RIGHT_DATA_BUF[NUMBER_OF_MAX_SAMPLES*SAMPLE_WIDTH];
static uint8_t *left_data = NULL;
static uint8_t *right_data = NULL;
static int num_samples;
static int sample_rate;
static int bytes_per_sample;
static int curr_count = 0;
static uint8_t curr_file_name[CURR_FILE_NAME_MAX_LEN];

int static audio_module_file_read_helper(FILE **ptr, uint8_t num_bytes)
{
    int val = 0, index = 0;
    uint8_t buff[4];
    size_t result;

    if (num_bytes > 4)
        return 0;
    result = fread(buff, 1, num_bytes, *ptr);
    if (result != num_bytes)
        WICED_BT_TRACE("[%s] Did not read %d bytes\n", __FUNCTION__, num_bytes);

    while (num_bytes)
    {
        val = val | (buff[index] << (8 * index));
        index++;
        num_bytes--;
    }
    return val;
}

uint8_t a_buf[NUMBER_OF_MAX_SAMPLES * 2];
uint8_t b_buf[NUMBER_OF_MAX_SAMPLES * 2];

uint8_t audio_module_load_wave_file(char *file_name)
{
    FILE *ptr = fopen(file_name, "rb");

    int total_file_size;
    int format_type, num_channels, block_align, data_size;
    int size_of_sample;
    int index;
    int res = 0;
    size_t result;

    if (ptr == NULL)
    {
        TRACE_ERR("Error while opening %s wave file", file_name);
        return false;
    }

    if (strlen(file_name) > CURR_FILE_NAME_MAX_LEN)
    {
        TRACE_ERR("file_name length over %d bytes, modify default max length value or change file path\n", CURR_FILE_NAME_MAX_LEN);
        fclose(ptr);
        return false;
    }

    if (strcmp(curr_file_name, file_name) != 0)
    {
        strncpy(curr_file_name, file_name, strlen(file_name));
        curr_count = 0;
    }

    // skip 4 byte "RIFF"
    res = fseek(ptr, 4, SEEK_SET);
    if (res != 0){
        TRACE_ERR("fseek RIFF Error! %d", res);
        fclose(ptr);
        return false;
    }

    //read total file size (4 bytes)
    total_file_size = audio_module_file_read_helper(&ptr, 4);
    TRACE_LOG("Total File size %d bytes", total_file_size);

    //skip 4 byte "WAVE" marker, 4 byte fmt, 4 byte format len
    res = fseek(ptr, 12, SEEK_CUR);
    if (res != 0){
        TRACE_ERR("fseek WAVE marker Error! %d", res);
        fclose(ptr);
        return false;
    }

    // 2 byte format. PCM = 1
    format_type = audio_module_file_read_helper(&ptr, 2);
    TRACE_LOG("Format Type %d", format_type);
    if (format_type != 1)
    {
        TRACE_ERR("Invalid Format type %d\n", format_type);
        fclose(ptr);
        return false;
    }

    // 2 byte number of channels
    num_channels = audio_module_file_read_helper(&ptr, 2);
    TRACE_LOG("Number of Channels %d", num_channels);
    if (num_channels != 2)
    {
        TRACE_ERR("Number of channel check failed %d", num_channels);
        fclose(ptr);
        return false;
    }

    // 4 byte sample rate
    sample_rate = audio_module_file_read_helper(&ptr, 4);
    TRACE_LOG("Sample rate %d", sample_rate);

    // skip 4 byte byterate
    res = fseek(ptr, 4, SEEK_CUR);
    if (res != 0){
        TRACE_ERR("fseek byterate Error! %d", res);
        fclose(ptr);
        return false;
    }

    // 2 byte block align
    block_align = audio_module_file_read_helper(&ptr, 2);
    TRACE_LOG("Block Align %d", block_align);

    // 2 bytes bits per sample
    bytes_per_sample = (audio_module_file_read_helper(&ptr, 2) / 8);

    // skip 4 byte data string
    res = fseek(ptr, 4, SEEK_CUR);
    if (res != 0){
        TRACE_ERR("fseek data string Error! %d", res);
        fclose(ptr);
        return false;
    }

    // read 4 bytes of data size
    data_size = audio_module_file_read_helper(&ptr, 4);

    if ((num_channels * bytes_per_sample) == 0){
        TRACE_ERR("num_channels %d bytes_per_sample %d", num_channels, bytes_per_sample);
        fclose(ptr);
        return false;
    }
    
    num_samples = (data_size) / (num_channels * bytes_per_sample);
    size_of_sample = num_channels * bytes_per_sample;
    TRACE_LOG("Data size %d number of sample %d size of each sample %d", data_size, num_samples, size_of_sample);
    if (num_samples > NUMBER_OF_MAX_SAMPLES)
        num_samples = NUMBER_OF_MAX_SAMPLES;

    left_data = a_buf;
    right_data = b_buf;

    for (index = 0; index < num_samples * bytes_per_sample; index += bytes_per_sample)
    {
        uint8_t temp_buff[6]; // num channels * width

        result = fread(&temp_buff, 1, size_of_sample, ptr);
        if (result != size_of_sample)
            TRACE_LOG("Did not read %d bytes", size_of_sample);

        memcpy(left_data + index, &temp_buff[0], bytes_per_sample);
        memcpy(right_data + index, &temp_buff[bytes_per_sample], bytes_per_sample);
    }
    fclose(ptr);
    return true;
}

//This function is used to get the sample size for given sampling frequency and frame duration.
//This function rounds off the calculated value to nearest integer.
uint32_t get_frame_size(uint16_t sampling_frequency, uint16_t frame_duration)
{
    return ((sampling_frequency * frame_duration) + 500000) / 1000000;
}

int audio_module_get_wave_data(uint8_t *l_data, uint8_t *r_data, uint16_t frame_duration)
{

    // num of samples = (sampl. freq./1000) * (sdu interval),
    // for 10ms interval and 48k sampl freq, input samples will be 480 samples
    int req_samples = get_frame_size(sample_rate, frame_duration);

    //WICED_BT_TRACE("[%s] req_samples : %d, bytes_per_sample : %d, num_samples : %d\n", __FUNCTION__, req_samples, bytes_per_sample, num_samples);

    if (num_samples == 0 || !left_data || !right_data || !l_data || !r_data)
    {
     //   WICED_BT_TRACE("[%s] l_data : 0x%x, r_data : 0x%x, left_data : 0x%x, right_data : 0x%x\n", __FUNCTION__, l_data, r_data, left_data, right_data);
     //   WICED_BT_TRACE("[%s] num_samples %d\n", __FUNCTION__, num_samples);
        return 0;
    }

    if ((curr_count + req_samples) < num_samples)
    {
        memcpy(l_data, left_data + (curr_count * bytes_per_sample), req_samples * bytes_per_sample);
        memcpy(r_data, right_data + (curr_count * bytes_per_sample), req_samples * bytes_per_sample);
        curr_count += (req_samples);
    }
    else
    {
        int left_over_samples = num_samples - curr_count;
        int remaining_samples = req_samples - left_over_samples;
        memcpy(l_data, left_data + (curr_count * bytes_per_sample), left_over_samples * bytes_per_sample);
        memcpy(r_data, right_data + (curr_count * bytes_per_sample), left_over_samples * bytes_per_sample);
        memcpy(l_data + (left_over_samples * bytes_per_sample), left_data, remaining_samples * bytes_per_sample);
        memcpy(r_data + (left_over_samples * bytes_per_sample), right_data, remaining_samples * bytes_per_sample);
        curr_count = remaining_samples;
    }

    //WICED_BT_TRACE("[%s] : curr_count %d\n", __FUNCTION__, curr_count);
    return req_samples;
}
