/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PACKETEXTRACTOR_IMPL_H
#define INCLUDED_RADAR_DSP_PACKETEXTRACTOR_IMPL_H

#include <gnuradio/radar_dsp/packetExtractor.h>
#include <vector>

namespace gr {
  namespace radar_dsp {

    class packetExtractor_impl : public packetExtractor
    {
     private:
      int _energy_factor_threshold, _sample_history_len, _n_collected, _packet_len, _initial_wait;
      bool _collecting;
      std::vector<gr_complex> _sample_history, _packet_buf;

     public:
      packetExtractor_impl(int sample_history_len, int energy_factor_threshold, int packet_len);
      ~packetExtractor_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PACKETEXTRACTOR_IMPL_H */
