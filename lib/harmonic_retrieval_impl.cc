/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include <vector>
#include <complex>
#include <iterator>
#include "harmonic_retrieval_impl.h"

const float PI = 3.14159265358979323; // std::acos(-1);
const gr_complex I(0, 1);

namespace gr {
  namespace radar_dsp {

    using input_type = gr_complex;
    using output_type = float;

    harmonic_retrieval::sptr
    harmonic_retrieval::make(int samp_rate, int fft_size)
    {
      return gnuradio::make_block_sptr<harmonic_retrieval_impl>(samp_rate, fft_size);
    }


    /*
     * The private constructor
     */
    harmonic_retrieval_impl::harmonic_retrieval_impl(int samp_rate, int fft_size)
      : gr::block("harmonic_retrieval",
                  gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type) * fft_size),
                  gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
        d_samp_rate(samp_rate),
        d_fft_size(fft_size)
    {}

    /*
     * Our virtual destructor.
     */
    harmonic_retrieval_impl::~harmonic_retrieval_impl()
    {
    }

    void
    harmonic_retrieval_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      //#pragma message("implement a forecast that fills in how many items on each input you need to produce noutput_items and remove this warning")
      ninput_items_required[0] = noutput_items;
    }

    inline float harmonic_retrieval_impl::beta(float k) {
      return 2.0 * PI * k / (float)d_fft_size;
    }

    int
    harmonic_retrieval_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const gr_complex *>(input_items[0]);
      auto out = static_cast<float *>(output_items[0]);
      int n_packets = ninput_items[0];
      std::vector<gr_complex> fft(d_fft_size);

      for (int fft_i = 0; fft_i < n_packets; fft_i++) {
        // Grab the first fft and put it into a vector
        fft.clear();
        for (int i = fft_i*d_fft_size; i < (fft_i+1)*d_fft_size; i++) {
          fft.push_back(in[i]);
        }

        auto r_k_1 = std::exp(-I * (float)2.0 * PI / (float)d_fft_size);
        auto iter_argmax = std::max_element(fft.begin(), fft.end(), [](gr_complex &a, gr_complex &b) {
          return abs(a) < abs(b);
        });
        // int k = std::distance(fft.begin(), iter_argmax);
        int k = 0;

        // cos_alpha_num = -z_k[k-1]*np.cos(beta_k(k-1)) + (1+r_k_1)*z_k[k]*np.cos(beta_k(k)) 
        //               - r_k_1*z_k[k+1]*np.cos(beta_k(k+1))
        // cos_alpha_den = -z_k[k-1] + (1+r_k_1)*z_k[k] - r_k_1*z_k[k+1]
        
        // TODO Check this left/right logic
        int left = (k == 0) ? d_fft_size-1 : k-1;  // Loop around the fft[k-1] lookup
        int right = (k == d_fft_size-1) ? 0 : k+1;  // Loop around the fft[k+1] lookup
        gr_complex cos_alpha_num = -fft[left]*std::cos(beta(k-1.0)) + ((float)1.0+r_k_1)*fft[k]*std::cos(beta(k)) - r_k_1*fft[right]*std::cos(beta(k+1.0));
        gr_complex cos_alpha_den = -fft[left] + ((float)1.0+r_k_1)*fft[k] - r_k_1*fft[right];

        // f = np.arccos((cos_alpha_num / cos_alpha_den).real) / np.pi * self.samp_rate / 2
        float freq = std::acos((cos_alpha_num / cos_alpha_den).real()) / PI * d_samp_rate / (float)2.0;

        // Output many copies of this float
        for (int i = 0; i < noutput_items; i++) {
          out[i] = freq;
        }
      }

      consume(0, noutput_items);

      // Tell runtime system how many output items we produced
      return noutput_items;
    }

  } /* namespace radar_dsp */
} /* namespace gr */
