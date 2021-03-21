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
 
 
#ifndef DSTAR_H
#define DSTAR_H
#include <Arduino.h>


class DSTAR
{
  public:
    DSTAR();
  
  
   
  void convolution(byte *input, byte *output);
  
  
//  void pseudo_random(byte *array);
  void pseudo_random(byte *array, int nbBits);

  void interleave(byte *array);
  void deInterleave(byte *data);
  
  void add_crc(byte *array);
  boolean check_crc(byte *array);

   
  void viterbi(byte* output,byte* history,byte* decoded);

//  void print_data(byte * array);
//  void print_data_bis(byte * array);

  int size_buffer;
  unsigned int crc;
  byte acc_error[2][4];
  
  private:
  
  void fcsbit(byte tbyte);
  void compute_crc(byte *array);
  void compute_error_branch(int ptr,byte current_state, byte encoded_input,byte* history);
  byte hamming_distance(byte encoder_channel,byte encoded_input);
  byte set_position(byte value, byte next_state, byte current_state);
  
  
};

extern DSTAR Dstar;

#endif
