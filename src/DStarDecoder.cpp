#include "DStarDecoder.h"
#include <Arduino.h>
#include <Streaming.h>

DStarDecoder::DStarDecoder()
{
    m_cur_mp = (mbe_parms*) malloc(sizeof(mbe_parms));
    m_prev_mp  = (mbe_parms*) malloc(sizeof(mbe_parms));
    m_prev_mp_enhanced  = (mbe_parms*) malloc(sizeof(mbe_parms));
    memset(ambe_d, 0, 49);
    memset(m_err_str, 0, 64);
    mbe_initMbeParms(m_cur_mp, m_prev_mp, m_prev_mp_enhanced);
}


void DStarDecoder::writeAudio(int16_t* pcm, uint16_t len)
{
    //todo
}

void DStarDecoder::process_dstar(unsigned char* d)
{
    for(auto i = 0; i < 9; i++)
    {
        *(d + i) =  reverse(*(d + i));
    }
    char ambe_fr[4][24];

    memset(ambe_fr, 0, 96);
    w = dW;
    x = dX;

    for(int i = 0; i < 9; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            ambe_fr[*w][*x] = (1 & (d[i] >> j));
            w++;
            x++;
        }
    }

    mbe_processAmbe3600x2400Framef(m_audio_out_temp_buf, &m_errs, &m_errs2, m_err_str, ambe_fr, ambe_d, m_cur_mp, m_prev_mp, m_prev_mp_enhanced, 1);
    Serial << "m_errs:" << m_errs << " m_errs2:" << m_errs2 << " m_err_str:" << m_err_str << endl;

    auto m_audio_out_temp_buf_p = m_audio_out_temp_buf;
    float m_volume = 1.0f;
    short m_audio_out_buf[160];
    short* m_audio_out_buf_p  = m_audio_out_buf;

    for(uint n = 0; n < sizeof(m_audio_out_buf); n++)
    {
        *m_audio_out_temp_buf_p *= m_volume;
        if(*m_audio_out_temp_buf_p > static_cast<float>(32760))
        {
            *m_audio_out_temp_buf_p = static_cast<float>(32760);
        }
        else if(*m_audio_out_temp_buf_p < static_cast<float>(-32760))
        {
            *m_audio_out_temp_buf_p = static_cast<float>(-32760);
        }

        *m_audio_out_buf_p = static_cast<short>(*m_audio_out_temp_buf_p);
        //        qDebug() << " o:" << *m_audio_out_temp_buf_p << " n:" << *m_audio_out_buf_p;
        m_audio_out_buf_p++;
        m_audio_out_temp_buf_p++;
    }
    writeAudio(m_audio_out_buf, sizeof(m_audio_out_buf));
}

void DStarDecoder::mbe_moveMbeParms(mbe_parms* cur_mp, mbe_parms* prev_mp)
{

    int l;

    prev_mp->w0 = cur_mp->w0;
    prev_mp->L = cur_mp->L;
    prev_mp->K = cur_mp->K;       // necessary?
    prev_mp->Ml[0] = (float) 0;
    prev_mp->gamma = cur_mp->gamma;
    prev_mp->repeat = cur_mp->repeat;
    for(l = 0; l <= 56; l++)
    {
        prev_mp->Ml[l] = cur_mp->Ml[l];
        prev_mp->Vl[l] = cur_mp->Vl[l];
        prev_mp->log2Ml[l] = cur_mp->log2Ml[l];
        prev_mp->PHIl[l] = cur_mp->PHIl[l];
        prev_mp->PSIl[l] = cur_mp->PSIl[l];
    }
}

void DStarDecoder::mbe_initMbeParms(mbe_parms* cur_mp, mbe_parms* prev_mp, mbe_parms* prev_mp_enhanced)
{

    int l;

    prev_mp->w0 = 0.09378;
    prev_mp->L = 30;
    prev_mp->K = 10;
    prev_mp->gamma = (float) 0;
    for(l = 0; l <= 56; l++)
    {
        prev_mp->Ml[l] = (float) 0;
        prev_mp->Vl[l] = 0;
        prev_mp->log2Ml[l] = (float) 0;   // log2 of 1 == 0
        prev_mp->PHIl[l] = (float) 0;
        prev_mp->PSIl[l] = (M_PI / (float) 2);
    }
    prev_mp->repeat = 0;
    mbe_moveMbeParms(prev_mp, cur_mp);
    mbe_moveMbeParms(prev_mp, prev_mp_enhanced);
}

uint8_t DStarDecoder::reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}
