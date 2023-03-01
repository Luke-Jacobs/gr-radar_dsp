/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "phase_measurement_impl.h"
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>

namespace gr {
  namespace radar_dsp {

    static const gr_complexd I(0.0, 1.0);
    static const double PI = 3.14159265358979323;  // std::acos(-1);

    phase_measurement::sptr phase_measurement::make(std::vector<float>& tone_frequencies, int n_tones, int vector_length, float samp_rate)
    {
      return gnuradio::make_block_sptr<phase_measurement_impl>(tone_frequencies, n_tones, vector_length, samp_rate);
    }

    /*
     * The private constructor
     */
    phase_measurement_impl::phase_measurement_impl(std::vector<float>& tone_frequencies, int n_tones, int vector_length, float samp_rate)
      : gr::block("phase_measurement",
                  gr::io_signature::make2(2, 2, sizeof(gr_complex)*vector_length, sizeof(float)),
                  gr::io_signature::make(1, 1, sizeof(float)*n_tones))
    {
      d_tone_frequencies = tone_frequencies;
      d_n_tones = n_tones;
      d_vector_length = vector_length;
      d_samp_rate = samp_rate;
    }

    /*
     * Our virtual destructor.
     */
    phase_measurement_impl::~phase_measurement_impl()
    {
    }

    void
    phase_measurement_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
      ninput_items_required[1] = noutput_items;
    }

    int
    phase_measurement_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in_samples = static_cast<const gr_complex *>(input_items[0]);
      auto in_cfo = static_cast<const float *>(input_items[1]);
      auto out_phases = static_cast<float *>(output_items[0]);

      // For each pair of samples vector and cfo, correlate the vector with complex
      // exponentials of frequency equal to the elements of `tone_frequencies` plus
      // cfo
      for (int samp_i = 0; samp_i < ninput_items[0]; samp_i++) {
        // Take the samples from the input as a vector
        arma::cx_dvec rx_buf(d_vector_length);
        for (int i = 0; i < d_vector_length; i++) {
          rx_buf(i) = in_samples[samp_i*d_vector_length + i];
        }

        arma::cx_dvec arange(d_vector_length);
        for (int i = 0; i < d_vector_length; i++) arange(i) = i;

        // For each tone frequency, compute the initial phase of that tone given the
        // RX buffer
        for (int tone_i = 0; tone_i < d_n_tones; tone_i++) {
          float freq_with_cfo = d_tone_frequencies[tone_i] - in_cfo[samp_i];
          arma::cx_dvec complex_tone = arma::exp(I * 2.0 * PI * arange * freq_with_cfo / d_samp_rate);  // TODO Check type conversions
          gr_complexd corr_sum = arma::sum(complex_tone.t() * rx_buf);
          float phase_i = std::arg(corr_sum);
          out_phases[samp_i*d_n_tones + tone_i] = phase_i;  // We write the phases in "vectors" of length d_n_tones
        }
      }

      // TODO Correct phases if one is looped around (holding to the short-distance assumption)

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (ninput_items[0]);

      // Tell runtime system how many output items we produced.
      return ninput_items[0];
    }

  } /* namespace radar_dsp */
} /* namespace gr */
