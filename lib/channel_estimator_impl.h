/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_CHANNEL_ESTIMATOR_IMPL_H
#define INCLUDED_RADAR_DSP_CHANNEL_ESTIMATOR_IMPL_H

#include <gnuradio/radar_dsp/channel_estimator.h>
#include <armadillo>

namespace gr {
  namespace radar_dsp {

    class channel_estimator_impl : public channel_estimator
    {
     private:
      int d_training_seq_len;
      float d_samp_rate;
      arma::cx_fvec d_ts_buf;

     public:
      channel_estimator_impl(int training_seq_len, float samp_rate, const std::vector<gr_complex>& buf);
      ~channel_estimator_impl();

      // Where all the action really happens
      int work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
      
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_CHANNEL_ESTIMATOR_IMPL_H */
