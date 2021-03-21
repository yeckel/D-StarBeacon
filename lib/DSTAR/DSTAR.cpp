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
 

#include <DSTAR.h>


DSTAR Dstar;


DSTAR::DSTAR(){

}

/********************************************************
 * convolve
 ********************************************************/

 
void DSTAR::convolution(byte *input, byte *output) {      //one byte input, two bytes output
   byte reg =0;
   int n,m;
   byte bits;
   byte mask_in;
   byte bit_out;
   unsigned int enc;
  
   int ptr=0;                            //ptr in encoded table
   for (n =0; n < size_buffer; n ++) {
		mask_in=0x01;                        
        bit_out=15;                      //use for 16 bits bitwrite 
        for (m=0; m < 8; m ++) {
          bits= (input[n] & mask_in) ? 1 : 0;    //1 if > 1, 0 else
		  mask_in<<=1;                      //shift mask to get each bit
          reg = (reg >>1) | bits <<2;      //convol reg
          bitWrite(enc, bit_out--, bitRead(reg, 0) ^ bitRead(reg, 1) ^ bitRead(reg, 2));      //msb poly 7
          bitWrite(enc, bit_out--, bitRead(reg, 0) ^ bitRead(reg, 2));                       //lsb poly 5
        }
        output[ptr++]=(enc>>8);            //one 16bits enc word is completed then store in 2 bytes
        output[ptr++]=(enc);

    }
}



/********************************************************
 * pseudo random generator
 ********************************************************/
//127 cyclic values can be generated
//E,F2,C9,2,26,2E,B6,C,D4,E7,B4,2A,FA,51,B8,FE,1D,E5,92,4,4C,5D,6C,19,A9,CF,68,55,F4,A3,71,FC,
//3B,CB,24,8,98,BA,D8,33,53,9E,D0,AB,E9,46,E3,F8,77,96,48,11,31,75,B0,66,A7,3D,A1,57,D2,8D,C7,
//F0,EF,2C,90,22,62,EB,60,CD,4E,7B,42,AF,A5,1B,8F,E1,DE,59,20,44,C5,D6,C1,9A,9C,F6,85,5F,4A,37,
//1F,C3,BC,B2,40,89,8B,AD,83,35,39,ED,A,BE,94,6E,3F,87,79,64,81,13,17,5B,6,6A,73,DA,15,7D,28,DC,7F,

void DSTAR::pseudo_random(byte *array, int nbBits)
{
  byte reg=0xff;
  byte pseudo;
  int n,b,BitsCount;
 n=0; 
 BitsCount=0;
 b=7;
 byte mask=0x00;
 while(BitsCount<nbBits)
{
  while ((b>=0) && (BitsCount<660))
  {
            bitWrite(reg,0,bitRead(reg,4)^bitRead(reg,7));
            bitWrite(pseudo,b,bitRead(reg,0));
            reg<<=1;
			bitWrite(mask,b,1);
		b--;
        BitsCount++;
  }
  b=7;  
  array[n]=(array[n]^pseudo)&mask;
  n++;
  mask=0x00;
 }
}


//buffer need to be a multiple of 8 bytes

/********************************************************
 * interleaving
 ********************************************************/

void DSTAR::interleave(byte *array)
{
static const int INTERLEAVE_TABLE[] PROGMEM = {
	0, 28, 56, 84, 112, 140, 168, 196, 224, 252, 280, 308, 336, 363, 390, 417, 444,
	471, 498, 525, 552, 579, 606, 633, 1, 29, 57, 85, 113, 141, 169, 197, 225, 253,
	281, 309, 337, 364, 391, 418, 445, 472, 499, 526, 553, 580, 607, 634, 2, 30, 58,
	86, 114, 142, 170, 198, 226, 254, 282, 310, 338, 365, 392, 419, 446, 473, 500,
	527, 554, 581, 608, 635, 3, 31, 59, 87, 115, 143, 171, 199, 227, 255, 283, 311,
	339, 366, 393, 420, 447, 474, 501, 528, 555, 582, 609, 636, 4, 32, 60, 88, 116,
	144, 172, 200, 228, 256, 284, 312, 340, 367, 394, 421, 448, 475, 502, 529, 556,
	583, 610, 637, 5, 33, 61, 89, 117, 145, 173, 201, 229, 257, 285, 313, 341, 368,
	395, 422, 449, 476, 503, 530, 557, 584, 611, 638, 6, 34, 62, 90, 118, 146, 174,
	202, 230, 258, 286, 314, 342, 369, 396, 423, 450, 477, 504, 531, 558, 585, 612,
	639, 7, 35, 63, 91, 119, 147, 175, 203, 231, 259, 287, 315, 343, 370, 397, 424,
	451, 478, 505, 532, 559, 586, 613, 640, 8, 36, 64, 92, 120, 148, 176, 204, 232,
	260, 288, 316, 344, 371, 398, 425, 452, 479, 506, 533, 560, 587, 614, 641, 9,
	37, 65, 93, 121, 149, 177, 205, 233, 261, 289, 317, 345, 372, 399, 426, 453, 480,
	507, 534, 561, 588, 615, 642, 10, 38, 66, 94, 122, 150, 178, 206, 234, 262, 290,
	318, 346, 373, 400, 427, 454, 481, 508, 535, 562, 589, 616, 643, 11, 39, 67, 95,
	123, 151, 179, 207, 235, 263, 291, 319, 347, 374, 401, 428, 455, 482, 509, 536,
	563, 590, 617, 644, 12, 40, 68, 96, 124, 152, 180, 208, 236, 264, 292, 320, 348,
	375, 402, 429, 456, 483, 510, 537, 564, 591, 618, 645, 13, 41, 69, 97, 125, 153,
	181, 209, 237, 265, 293, 321, 349, 376, 403, 430, 457, 484, 511, 538, 565, 592,
	619, 646, 14, 42, 70, 98, 126, 154, 182, 210, 238, 266, 294, 322, 350, 377, 404,
	431, 458, 485, 512, 539, 566, 593, 620, 647, 15, 43, 71, 99, 127, 155, 183, 211,
	239, 267, 295, 323, 351, 378, 405, 432, 459, 486, 513, 540, 567, 594, 621, 648,
	16, 44, 72, 100, 128, 156, 184, 212, 240, 268, 296, 324, 352, 379, 406, 433, 460,
	487, 514, 541, 568, 595, 622, 649, 17, 45, 73, 101, 129, 157, 185, 213, 241, 269,
	297, 325, 353, 380, 407, 434, 461, 488, 515, 542, 569, 596, 623, 650, 18, 46, 74,
	102, 130, 158, 186, 214, 242, 270, 298, 326, 354, 381, 408, 435, 462, 489, 516,
	543, 570, 597, 624, 651, 19, 47, 75, 103, 131, 159, 187, 215, 243, 271, 299, 327,
	355, 382, 409, 436, 463, 490, 517, 544, 571, 598, 625, 652, 20, 48, 76, 104, 132,
	160, 188, 216, 244, 272, 300, 328, 356, 383, 410, 437, 464, 491, 518, 545, 572,
	599, 626, 653, 21, 49, 77, 105, 133, 161, 189, 217, 245, 273, 301, 329, 357, 384,
	411, 438, 465, 492, 519, 546, 573, 600, 627, 654, 22, 50, 78, 106, 134, 162, 190,
	218, 246, 274, 302, 330, 358, 385, 412, 439, 466, 493, 520, 547, 574, 601, 628,
	655, 23, 51, 79, 107, 135, 163, 191, 219, 247, 275, 303, 331, 359, 386, 413, 440,
	467, 494, 521, 548, 575, 602, 629, 656, 24, 52, 80, 108, 136, 164, 192, 220, 248,
	276, 304, 332, 360, 387, 414, 441, 468, 495, 522, 549, 576, 603, 630, 657, 25, 53,
	81, 109, 137, 165, 193, 221, 249, 277, 305, 333, 361, 388, 415, 442, 469, 496, 523,
	550, 577, 604, 631, 658, 26, 54, 82, 110, 138, 166, 194, 222, 250, 278, 306, 334,
	362, 389, 416, 443, 470, 497, 524, 551, 578, 605, 632, 659, 27, 55, 83, 111, 139,
	167, 195, 223, 251, 279, 307, 335}; 
byte out[84];
int nBits=0;
int b=7;
int k=0;

for (k=0;k<84;k++) out[k]=0;
k=0;
while(nBits<660)
{
  while ((b>=0) && (nBits<660))
  {
        k=int(pgm_read_word(&INTERLEAVE_TABLE[nBits]));
        bitWrite(out[k/8],7-(k%8), bitRead(array[nBits/8], 7-(nBits%8)));   
        b--;
        nBits++;
  }
  b=7;
  
}

for (k=0;k<84;k++) array[k]=out[k];
}

/********************************************************
 * de-interleaving
 ********************************************************/

void DSTAR::deInterleave(byte *data)
{
byte out[84];
int nBits=0;
int b=7;
int k=0;

for (k=0;k<84;k++) out[k]=0;
k=0;
while(nBits<660)
{
  while ((b>=0) && (nBits<660))
  {
        bitWrite(out[k/8],7-(k%8), bitRead(data[nBits/8], 7-(nBits%8)));   
	k += 24;
        b--;
        nBits++;
	if (k >= 672) {
		k -= 671;
	} else if (k >= 660) {
		k -= 647;
	};
  }
  b=7;
}

for (k=0;k<84;k++) data[k]=out[k];


/*
//print binary table
while(nBits<660)
{
  while ((b>=0) && (nBits<660))
  {
        Serial.print(bitRead(data[nBits/8], 7-(nBits%8)));   
        nBits++;
        if (nBits%28==0) Serial.println();
        b--;
  }
  b=7;
  
}
*/
}
 

/********************************************************
 * CRC calculator
 ********************************************************/

void DSTAR::fcsbit(byte tbyte)
{
  crc ^= tbyte;
  if (crc & 1)
    crc = (crc >> 1) ^ 0x8408;  // X-modem CRC poly
  else
    crc = crc >> 1;
 
}

void DSTAR::compute_crc(byte *array)
{
crc=0xffff;  
for (int n =0; n < (size_buffer-3); n ++)
    {
      for (int m=0;m<8;m++)  {                //each bit must be calculated separatly
                  DSTAR::fcsbit(bitRead(array[n],m));
      }
    }
crc^=0xffff;
}

void DSTAR::add_crc(byte *array)          //add crc to inupt array
{
DSTAR::compute_crc(array);
array[size_buffer-2]=(crc>>8);
array[size_buffer-3]=(crc);
}

boolean DSTAR::check_crc(byte *array)    //check decoded array if crc is good
{                                 //because viterbi decoding doesn't know if decoded data is good
DSTAR::compute_crc(array);
unsigned int crc_decoded;
crc_decoded=(array[size_buffer-2]<<8)+array[size_buffer-3];			//inversion msb lsb
if(crc_decoded==crc) return true; else return false;
}


 /********************************************************
 * THE Viterbi decoder
 ********************************************************/

void DSTAR::viterbi(byte* output,byte* history,byte* decoded)                         //viterbi decode
{                                      //first compute error and history table, then find decoded data
int n,m;
byte dual;
byte mask;
static byte state_transitions_table[4][4]={{0,4,1,4},  //4 number as x (never used)
										   {0,4,1,4},
										   {4,0,4,1},
										   {4,0,4,1}};

   //compute error and history table
    for (n =0; n <2; n ++) for (m =0; m <4; m ++) acc_error[n][m]=0;   //clear acc_error_shift_table

   // building the accumulated error metric and state history table
   DSTAR::compute_error_branch(0, 0, output[0]>>6,history);                           // ptr, current_state, encoded_input
   DSTAR::compute_error_branch(1, 0, (output[0]>>4)&3,history);
   DSTAR::compute_error_branch(1, 2, (output[0]>>4)&3,history);
   for (m =0; m <4; m ++) DSTAR::compute_error_branch(2, m, (output[0]>>2)&3,history);
   for (m =0; m <4; m ++) DSTAR::compute_error_branch(3, m, output[0]&3,history);
   
   for (n=4; n < (size_buffer*8) -1; n ++) for (m =0; m <4; m ++) {
                                                                   switch(n&3)      //select pair of bit in a byte
                                                                   {
                                                                   case 0 : dual=output[n/4]>>6;break;
                                                                   case 1 : dual=(output[n/4]>>4)&3;break;
                                                                   case 2 : dual=(output[n/4]>>2)&3;break;
                                                                   case 3 : dual=output[n/4]&3;break;
                                                                   }
                                                                   DSTAR::compute_error_branch(n, m, dual,history);
                                                                   }
   //find decoded data
   // Traceback : Working backward through the state history and recreate the original message 
   byte current_state =0;
   byte predecessor_state;
   m=size_buffer-1;
     mask=1;
   for (n = (size_buffer*8) -1; n >0; n --) {   
      switch(current_state) {                    //found exact state in byte
         case 0 : predecessor_state = (history[n] &0x03);
         break;
         case 1 : predecessor_state = (history[n] &0x0c) >>2;
         break;
         case 2 : predecessor_state = (history[n] &0x30) >>4;
         break;
         case 3 : predecessor_state = (history[n] &0xc0) >>6;
         break;
      }
        bitWrite(decoded[m],7-mask,state_transitions_table[predecessor_state][current_state]);		//modif 7-mask afin d'éviter de permuter les bits
        mask++;
       if (mask==8) {
                    mask=0;
                    m--;
                }
      current_state = predecessor_state;
   }
}

void DSTAR::compute_error_branch(int ptr,byte current_state, byte encoded_input,byte* history)
{
  static byte transition[]={0,0,3,2, 3,0,0,2, 2,1,1,3, 1,1,2,3};    //table to find transition and next state for channel A and B
   byte encoder_channel_A = transition[current_state *4];
   byte next_state_A = transition[current_state *4 +1];
   byte encoder_channel_B = transition[current_state *4 +2];
   byte next_state_B = transition[current_state *4 +3];
   byte errorA = DSTAR::hamming_distance(encoded_input, encoder_channel_A);
   byte errorB = DSTAR::hamming_distance(encoded_input, encoder_channel_B);

    if (current_state ==0) {                                          //shift accumulated error matrix
      for (int n =0; n <4; n ++) acc_error[0][n] = acc_error[1][n];
   }

   if ((current_state &1) ==0) {                                      //if state=0 or 2 then compute error and history
      acc_error[1][next_state_A] = errorA + acc_error[0][current_state];
      acc_error[1][next_state_B] = errorB + acc_error[0][current_state];
      history[ptr +1] = set_position(history[ptr +1], next_state_A, current_state);
      history[ptr +1] = set_position(history[ptr +1], next_state_B, current_state);
   } else {                                                          //else (state=1 or 3) then swapp error and history if necessary
            if (acc_error[1][next_state_A] > (errorA + acc_error[0][current_state])) {
                                                                                   acc_error[1][next_state_A] = errorA + acc_error[0][current_state];
                                                                                   history[ptr +1] = DSTAR::set_position(history[ptr +1], next_state_A, current_state);
                                                                                      }
            if (acc_error[1][next_state_B] > (errorB + acc_error[0][current_state])) {
                                                                                   acc_error[1][next_state_B] = errorB + acc_error[0][current_state];
                                                                                   history[ptr +1] = DSTAR::set_position(history[ptr +1], next_state_B, current_state);
                                                                                  }
   }
}

byte DSTAR::hamming_distance(byte encoder_channel,byte encoded_input)
{
  
static byte hamming_table[4][4]={{0,1,1,2},        //compute hamming distance by table (more easy)
								 {1,0,2,1},
								 {1,2,0,1},
								 {2,1,1,0}};

return hamming_table[encoder_channel][encoded_input];

//or 
//return   (((encoder_channel>>1)-(encoded_input>>1)) ? 1 : 0) + (((encoder_channel&1)-(encoded_input&1)) ? 1 : 0);
}

byte DSTAR::set_position(byte value, byte next_state, byte current_state) {    //set state position in byte with & and | mask
   switch(next_state) {
      case 0 : value = value &0xfc | current_state;
      break;
      case 1 : value = value &0xf3 | (current_state <<2);
      break;
      case 2 : value = value &0xcf | (current_state <<4);
      break;
      case 3 : value = value &0x3f | (current_state <<6);
      break;
   }
   return  value;
}


 /********************************************************
 * Print array routines optional
 ********************************************************/
/*
void DSTAR::print_data(byte * array) {
   for (int n =0; n < size_buffer; n ++) {
      Serial.print(array[n],HEX);
      Serial.print(",");
   }
   Serial.println();
}

void DSTAR::print_data_bis(byte * array) {
   for (int n =0; n < size_buffer*2; n ++) {
      Serial.print(array[n],HEX);
      Serial.print(",");
   }
   Serial.println();
}
*/

/*
//reverse bits if i need it
   for (k=0;k<BUFSIZE;k++) {                       
                       temp=bufferTxRx[k];
                       for (b=0;b<8;b++) bitWrite(bufferTxRx[k],7-b, bitRead(temp, b));  
                     }
*/   
