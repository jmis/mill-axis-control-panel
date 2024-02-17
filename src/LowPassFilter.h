#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif