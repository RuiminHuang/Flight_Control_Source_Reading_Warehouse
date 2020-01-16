//
/// @file LowPassFilter.cpp
/// @brief  A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain
#include "util.h"
#include "aq.h"
#include "LowPassFilterFloat.h"

////////////////////////////////////////////////////////////////////////////////////////////
// DigitalLPF
////////////////////////////////////////////////////////////////////////////////////////////

static float apply(LowPassFilterFloat *lpf, const float sample) {
    lpf->_output += (sample - lpf->_output) * lpf->alpha;
    return lpf->_output;
}


static void compute_alpha(LowPassFilterFloat *lpf, float sample_freq, float cutoff_freq) {
    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        lpf->alpha = 1.0;
    } else {
        float dt = 1.0/sample_freq;  
        float rc = 1.0f/(M_2PI*cutoff_freq);
        lpf->alpha = constrainFloat(dt/(dt+rc), 0.0f, 1.0f);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter
////////////////////////////////////////////////////////////////////////////////////////////

static void set_cutoff_frequency(LowPassFilterFloat *lpf, float sample_freq, float cutoff_freq) {
    lpf->_cutoff_freq = cutoff_freq;
    compute_alpha(lpf, sample_freq, cutoff_freq);
}

void LowPassFilterFloat_init(LowPassFilterFloat *lpf, float sample_freq, float cutoff_freq)
{
    set_cutoff_frequency(lpf, sample_freq, cutoff_freq);
}


float LowPassFilterFloat_apply(LowPassFilterFloat *lpf, float sample) {
    return apply(lpf, sample);
}




