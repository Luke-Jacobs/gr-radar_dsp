/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PULSE_ALIGN_IMPL_H
#define INCLUDED_RADAR_DSP_PULSE_ALIGN_IMPL_H

#include <gnuradio/radar_dsp/pulse_align.h>

namespace gr {
namespace radar_dsp {

class pulse_align_impl : public pulse_align
{
private:
    std::vector<gr_complex> d_buf;
    int d_input_buffer_len, d_samps_per_sym;
    float d_samp_rate;

public:
    pulse_align_impl(int input_buffer_len, float samp_rate, int samps_per_sym);
    ~pulse_align_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PULSE_ALIGN_IMPL_H */
