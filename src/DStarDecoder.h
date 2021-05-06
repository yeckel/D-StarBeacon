#pragma once
#include <cstdint>

extern "C" {
#include "mbelib.h"
}

class DStarDecoder
{
public:
    DStarDecoder();
    void process_dstar(unsigned char* d);

private:
    mbe_parameters* m_cur_mp;
    mbe_parameters* m_prev_mp;
    mbe_parameters* m_prev_mp_enhanced;

    float m_audio_out_temp_buf[160];   //!< output of decoder
    int m_errs{0};
    int m_errs2{0};
    char m_err_str[64];
    char ambe_d[49];

    const int* w, *x;

    const int dW[72] = {0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2, 0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0,
                        3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2, 0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 3, 2, 1, 0, 0, 3, 3,
                       };

    const int dX[72] = {10, 22, 11, 9, 10, 22, 11, 23, 8, 20, 9, 21, 10, 8, 9, 21, 8, 6, 7, 19, 8, 20, 9, 7, 6, 18, 7, 5, 6, 18, 7, 19, 4, 16, 5, 17, 6,
                        4, 5, 17, 4, 2, 3, 15, 4, 16, 5, 3, 2, 14, 3, 1, 2, 14, 3, 15, 0, 12, 1, 13, 2, 0, 1, 13, 0, 12, 10, 11, 0, 12, 1, 13,
                       };

    void mbe_initMbeParms(mbe_parms* cur_mp, mbe_parms* prev_mp, mbe_parms* prev_mp_enhanced);
    void mbe_moveMbeParms(mbe_parms* cur_mp, mbe_parms* prev_mp);
    void writeAudio(int16_t* pcm, uint16_t len);
    uint8_t reverse(uint8_t b);
};
