#pragma once
#include <Stream.h>
#include <Streaming.h>
#if 0
#define LOGRX  Serial
#else
#define LOGRX m_noOut
#endif

#if 0
#define LOGTX  Serial
#else
#define LOGTX m_noOut
#endif

#if 0
#define LOGWEB  Serial
#else
#define LOGWEB m_noOut
#endif
class NoOut: public Stream
{
    // Print interface
public:
    size_t write(uint8_t) override
    {
        return 0;
    };
    // Stream interface
public:
    int available() override
    {
        return 0;
    };
    int read() override
    {
        return 0;
    };
    int peek() override
    {
        return 0;
    };
    void flush() override {};
};

extern NoOut m_noOut;
