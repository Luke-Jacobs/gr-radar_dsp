/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_LLS_IMPL_H
#define INCLUDED_RADAR_DSP_LLS_IMPL_H

#include <gnuradio/radar_dsp/lls.h>
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>


namespace gr {
  namespace radar_dsp {

    class lls_impl : public lls
    {
     private:
      float d_f_c;
      int d_n_tones;
      arma::fcolvec d_f_i;

     public:
      lls_impl(float f_c, std::vector<float>& f_i, int n_tones);
      ~lls_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_LLS_IMPL_H */
