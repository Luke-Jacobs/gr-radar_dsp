/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_IMPL_H
#define INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_IMPL_H

#include <gnuradio/radar_dsp/selfref_framesync.h>
#include <complex>

namespace gr {
  namespace radar_dsp {

    class selfref_framesync_impl : public selfref_framesync
    {
     private:
      int d_training_seq_len, d_packet_len, d_items_to_write;
      float d_threshold, d_prev_corr;
      std::vector<gr_complex> d_packet_buf, d_training_buf;
      bool d_packet_detected;

     public:
      selfref_framesync_impl(int training_seq_len, int packet_len, float threshold);
      ~selfref_framesync_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_IMPL_H */
