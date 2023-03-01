/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_IMPL_H
#define INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_IMPL_H

#include <gnuradio/radar_dsp/phase_measurement.h>

namespace gr {
  namespace radar_dsp {

    class phase_measurement_impl : public phase_measurement
    {
     private:
      std::vector<float> d_tone_frequencies;
      int d_n_tones, d_vector_length;
      float d_samp_rate;

     public:
      phase_measurement_impl(std::vector<float>& tone_frequencies, int n_tones, int vector_length, float samp_rate);
      ~phase_measurement_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_IMPL_H */
