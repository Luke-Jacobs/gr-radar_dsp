/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "cfo_estimator_impl.h"
#include <gnuradio/io_signature.h>
#include <armadillo>

namespace gr {
namespace radar_dsp {

using input_type = gr_complex;
using output_type = float;

static const double PI = 3.14159265358979323;  // std::acos(-1);

cfo_estimator::sptr cfo_estimator::make(int training_seq_len, float samp_rate)
{
   return gnuradio::make_block_sptr<cfo_estimator_impl>(training_seq_len, samp_rate);
}


/*
 * The private constructor
 */
cfo_estimator_impl::cfo_estimator_impl(int training_seq_len, float samp_rate)
    : gr::sync_block("cfo_estimator",
                     gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)*training_seq_len),
                     gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
      d_training_seq_len(training_seq_len),  // training_seq_len is the length of the whole synchronization sequence (both halves)
      d_samp_rate(samp_rate)
{
}

/*
 * Our virtual destructor.
 */
cfo_estimator_impl::~cfo_estimator_impl() {}

int cfo_estimator_impl::work(int noutput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    /* Estimate the frequency offset that needs to be added to TX's carrier to get RX's carrier.
       This block can be used to "unspin" the components of a vector. */
    for (int vec_i = 0; vec_i < noutput_items; vec_i++) {
        const gr_complex *vec_start = in+vec_i*d_training_seq_len;
        arma::cx_fvec first_half(std::vector<gr_complex>(vec_start, vec_start+d_training_seq_len/2)),
                      second_half(std::vector<gr_complex>(vec_start+d_training_seq_len/2, vec_start+d_training_seq_len));
        float cfo_est = std::arg(arma::sum(first_half.t() * second_half)) * d_samp_rate / (2*PI*d_training_seq_len/2);
        out[vec_i] = cfo_est;
        // std::cout << cfo_est << "\n";
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace radar_dsp */
} /* namespace gr */
