/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "lls_impl.h"
#include <cassert>

namespace gr {
  namespace radar_dsp {

    using input_type = float;
    using output_type = float;

    static const float PI = 3.14159265358979323;  // std::acos(-1);
    static const float C = 299792458;

    lls::sptr lls::make(float f_c, std::vector<float>& f_i, int n_tones)
    {
      return gnuradio::make_block_sptr<lls_impl>(f_c, f_i, n_tones);
    }

    /*
     * The private constructor
     */
    lls_impl::lls_impl(float f_c, std::vector<float>& f_i, int n_tones)
      : gr::block("lls",
                  gr::io_signature::make2(2, 2, sizeof(input_type), sizeof(input_type)*f_i.size()),
                  gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      assert(n_tones == f_i.size());
      d_f_c = f_c;
      d_f_i = arma::fcolvec(f_i);
      d_n_tones = n_tones;
    }

    /*
     * Our virtual destructor.
     */
    lls_impl::~lls_impl()
    {
    }

    void
    lls_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
      ninput_items_required[1] = noutput_items;
    }

    int
    lls_impl::general_work (int noutput_items,
                            gr_vector_int &ninput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
    {
      auto in_cfo = static_cast<const input_type*>(input_items[0]);
      auto in_phases = static_cast<const input_type*>(input_items[1]);
      auto out_dist = static_cast<output_type*>(output_items[0]);

      arma::fmat A(d_n_tones, 2, arma::fill::ones);  // Rows are the same length as f vector, only 2 columns

      // Produce a distance estimate for each pair of cfo and phase vector
      for (int samp_i = 0; samp_i < ninput_items[0]; samp_i++) {
        // Fill A vector with f_c, f_CFO, f_i values
        float f_cfo = in_cfo[samp_i];  // We do not assume CFO is constant
        arma::fvec x_vals = -2*PI*(2*d_f_c + f_cfo + d_f_i);
        // Make phases into a vector
        arma::fvec phases(d_n_tones);  // We expect as many phase values as values of f_i
        for (int i = 0; i < d_n_tones; i++) {
          phases(i) = in_phases[samp_i*d_f_i.size() + i];
        }
        // Run LLS
        arma::fvec solution = arma::polyfit(x_vals, phases, 1);
        float tau = solution(0);
        float theta = solution(1);

        // Write distance estimate to output
        // std::cout << "Tau is: " << tau << " Theta is: " << theta << "\n";
        out_dist[samp_i] = tau * C;
      }

      // Tell runtime system how many input items we consumed on each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar_dsp */
} /* namespace gr */
