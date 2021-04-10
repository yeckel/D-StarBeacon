/********************************************************************************************
 * DSTAR Arduino library
 * Created 8/11/2015
 * Anthony LE CREN f4goh@orange.fr
 * Modified
 * BSD license, all text above must be included in any redistribution
 *
 * Instance :
 *
 * Functions :
 *
 *******************************************************************************************/


#pragma once
#include <Arduino.h>


class DSTAR
{
public:
    static constexpr uint16_t RF_HEADER_SIZE{42u};//in real data(39)+crc(2)+1 padding
    static constexpr uint16_t RF_HEADER_TRANSFER_BITSIZE{660u};
    void convolution(byte* input, byte* output);


    //  void pseudo_random(byte *array);
    void pseudo_random(byte* array, int nbBits);

    void interleave(byte* array);
    void deInterleave(byte* data);

    void add_crc(byte* array);
    boolean check_crc(byte* array);


    void viterbi(byte* output, byte* history, byte* decoded);

    //  void print_data(byte * array);
    //  void print_data_bis(byte * array);

    int size_buffer{RF_HEADER_SIZE};
    unsigned int crc;
    byte acc_error[2][4];
    uint16_t calcCCITTCRC(uint8_t* buffer, int startpos, int length);
private:
    void fcsbit(byte tbyte);
    void compute_crc(byte* array);
    void compute_error_branch(int ptr, byte current_state, byte encoded_input, byte* history);
    byte hamming_distance(byte encoder_channel, byte encoded_input);
    byte set_position(byte value, byte next_state, byte current_state);
};
