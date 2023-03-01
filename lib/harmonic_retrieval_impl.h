/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_IMPL_H
#define INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_IMPL_H

#include <gnuradio/radar_dsp/harmonic_retrieval.h>

namespace gr {
  namespace radar_dsp {

    class harmonic_retrieval_impl : public harmonic_retrieval
    {
     private:
      int d_samp_rate, d_fft_size;

      inline float beta(float k);

     public:
      harmonic_retrieval_impl(int samp_rate, int fft_size);
      ~harmonic_retrieval_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_IMPL_H */
