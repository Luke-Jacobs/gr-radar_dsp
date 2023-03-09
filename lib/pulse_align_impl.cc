/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "pulse_align_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/filter/firdes.h>
#include <armadillo>
#include "usrp_tx_rx_impl.h"  // TODO Hopefully the linking with the usrp_tx_rx block won't be a problem

namespace gr {
namespace radar_dsp {

/*
 * Downsampling
 */
std::vector<gr_complex> downsample(const std::vector<gr_complex>::iterator in_start, 
                                   const std::vector<gr_complex>::iterator in_end, 
                                   int downsampling_rate) {
    assert(downsampling_rate > 0);
    double input_len = std::distance(in_start, in_end);
    std::vector<gr_complex> out(std::ceil(input_len / downsampling_rate));

    for (int i = 0; i < out.size(); i++) {
        out[i] = *(in_start + i*downsampling_rate);
    }
    return out;
}

pulse_align::sptr pulse_align::make(int input_buffer_len, float samp_rate, int samps_per_sym)
{
    return gnuradio::make_block_sptr<pulse_align_impl>(input_buffer_len, samp_rate, samps_per_sym);
}

/*
 * The private constructor
 */
pulse_align_impl::pulse_align_impl(int input_buffer_len, float samp_rate, int samps_per_sym)
    : gr::block("pulse_align",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, input_buffer_len*sizeof(gr_complex)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(gr_complex))),
                d_input_buffer_len(input_buffer_len),
                d_samps_per_sym(samps_per_sym),
                d_samp_rate(samp_rate)
{
    assert(d_samps_per_sym <= d_input_buffer_len);
}

/*
 * Our virtual destructor.
 */
pulse_align_impl::~pulse_align_impl() {}

void pulse_align_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

int pulse_align_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    auto in = static_cast<const gr_complex *>(input_items[0]);
    auto out = static_cast<gr_complex *>(output_items[0]);
    int vec_i, out_written = 0;

    std::cout << "PulseAlign ninput_items[0] = " << ninput_items[0] << std::endl;
    for (vec_i = 0; vec_i < ninput_items[0]; vec_i++) {  // TODO Check to see if ninput_items treats a input_buffer_len length vector as 1 item
        // Convolve buffer with matched filter
        std::vector<gr_complex> buf(in+vec_i*d_input_buffer_len, in+(vec_i+1)*d_input_buffer_len);        
        buf = rrc_filter(buf, d_samp_rate, d_samps_per_sym, 11*d_samps_per_sym);

        // Compare the average energy output of decimation at each possible shift (0...N-1)
        float best_avg_energy = 0.0;
        int best_shift = 0;

        for (int shift = 0; shift < d_samps_per_sym; shift++) {
            auto dec_buf = downsample(buf.begin()+shift, buf.end(), d_samps_per_sym);
            float energy = 0.0;
            std::for_each(dec_buf.begin(), dec_buf.end(), [&energy](gr_complex samp){energy += std::abs(samp);});
            float avg_energy = energy / dec_buf.size();
            // Update the best shift record if this is a better shift than the previous best
            if (avg_energy > best_avg_energy) {
                best_avg_energy = avg_energy;
                best_shift = shift;
                // std::cout << "Found better shift " << best_shift << " with energy " << avg_energy << std::endl;
            }
        }

        auto out_vec = downsample(buf.begin()+best_shift, buf.end(), d_samps_per_sym);
        std::copy(out_vec.begin(), out_vec.end(), out+out_written);
        out_written += out_vec.size();
    }

    // Tell runtime system how many input items we consumed on each input stream.
    // std::cout << "Wrote " << out_written << " items\n";
    consume(0, vec_i);

    // Tell runtime system how many output samples we produced.
    return out_written;
}

} /* namespace radar_dsp */
} /* namespace gr */
