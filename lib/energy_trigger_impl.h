/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_ENERGY_TRIGGER_IMPL_H
#define INCLUDED_RADAR_DSP_ENERGY_TRIGGER_IMPL_H

#include <gnuradio/radar_dsp/energy_trigger.h>

namespace gr {
  namespace radar_dsp {

    class energy_trigger_impl : public energy_trigger
    {
     private:
      int d_buf_len_before, d_buf_len_at_after;

     public:
      energy_trigger_impl(float energy_threshold, int buf_len_before, int buf_len_at_after);
      ~energy_trigger_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_ENERGY_TRIGGER_IMPL_H */
