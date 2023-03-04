/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "channel_estimator_impl.h"

namespace gr {
  namespace radar_dsp {

  static const double PI = 3.14159265358979323;  // std::acos(-1);

    channel_estimator::sptr channel_estimator::make(int training_seq_len, float samp_rate, const std::vector<gr_complex>& ts_buf)
    {
      return gnuradio::make_block_sptr<channel_estimator_impl>(training_seq_len, samp_rate, ts_buf);
    }


    /*
     * The private constructor
     */
    channel_estimator_impl::channel_estimator_impl(int training_seq_len, float samp_rate, const std::vector<gr_complex>& ts_buf)
      : gr::sync_block("channel_estimator",
                       gr::io_signature::make2(2 /* min inputs */, 2 /* max inputs */, sizeof(gr_complex)*training_seq_len, sizeof(float)),
                       gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(gr_complex))),
        d_training_seq_len(training_seq_len),
        d_samp_rate(samp_rate)
      {
        d_ts_buf = arma::cx_fvec(ts_buf);
      }

    /*
     * Our virtual destructor.
     */
    channel_estimator_impl::~channel_estimator_impl() {}

    int
    channel_estimator_impl::work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {
      auto rx_ts_in = static_cast<const gr_complex *>(input_items[0]);
      auto cfo_est_in = static_cast<const float *>(input_items[1]);
      auto out = static_cast<gr_complex *>(output_items[0]);
      
      for (int vec_i = 0; vec_i < noutput_items; vec_i++) {
        // Undo CFO with the given CFO estimate
        std::vector<gr_complex> rx_ts_buf(rx_ts_in+vec_i*d_training_seq_len, rx_ts_in+(vec_i+1)*d_training_seq_len);
        for (int i = 0; i < d_training_seq_len; i++) {
          rx_ts_buf[i] *= std::exp(std::complex(0.0, 2.0*PI*cfo_est_in[vec_i]*i/d_samp_rate));  // The CFO estimate needs to have the correct sign or else this will not work
        }

        // Correlate the received training sequence against the transmitted copy
        arma::cx_fvec rx_ts_vec(rx_ts_buf);
        gr_complex corr = arma::sum(rx_ts_vec.t() * d_ts_buf);  // TODO Check the order of the dot product
        gr_complex unit_rotation = corr / std::abs(corr);
        out[vec_i] = unit_rotation;
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar_dsp */
} /* namespace gr */
