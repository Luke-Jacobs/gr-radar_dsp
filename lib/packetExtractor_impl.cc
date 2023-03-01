/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "packetExtractor_impl.h"
#include <complex>

namespace gr {
namespace radar_dsp {

using input_type = gr_complex;
using output_type = gr_complex;

packetExtractor::sptr packetExtractor::make(int sample_history_len, int energy_factor_threshold, int packet_len)
{
  return gnuradio::make_block_sptr<packetExtractor_impl>(sample_history_len, energy_factor_threshold, packet_len);
}

/*
  * The private constructor
  */
packetExtractor_impl::packetExtractor_impl(int sample_history_len, int energy_factor_threshold, int packet_len)
  : gr::block("packetExtractor",
              gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
              gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type) * packet_len)),
    _sample_history_len(sample_history_len),
    _energy_factor_threshold(energy_factor_threshold),
    _packet_len(packet_len),
    _n_collected(0),
    _collecting(false),
    _initial_wait(sample_history_len)
{
  // Initialize sample history with all 0's
  for (int i = 0; i < sample_history_len; i++)
    _sample_history.push_back(gr_complex(0.0, 0.0));
  // Empty packet buffer
  _packet_buf.clear();
}

/*
  * Our virtual destructor.
  */
packetExtractor_impl::~packetExtractor_impl()
{}

void
packetExtractor_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  ninput_items_required[0] = _packet_len*noutput_items;
}

int
packetExtractor_impl::general_work (int noutput_items,
                                    gr_vector_int &ninput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items) 
{
  auto in = static_cast<const input_type*>(input_items[0]);
  auto out = static_cast<output_type*>(output_items[0]);
  int samp_i = 0;  // Keeps track of how many samples we have consumed
  int n_in = ninput_items[0];

  for (samp_i = 0; samp_i < n_in; samp_i++) {
    // Compute average energy
    float avg_energy = 0.0;
    for (int hist_i = 0; hist_i < _sample_history_len; hist_i++) {
      avg_energy += std::norm(_sample_history.at(hist_i));
    }
    avg_energy /= _sample_history_len;

    // Detect peak if we are not waiting nor collecting
    if ((_initial_wait < 1) && !_collecting && (std::norm(in[samp_i]) > _energy_factor_threshold * avg_energy)) {
      // unsigned long writeIndex = nitems_read(0) + samp_i; // The order that we are given input samples is oldest->newest
      // add_item_tag(0, writeIndex, pmt::mp("packet_len"), pmt::from_long(_packet_len));
      _n_collected = 0;
      _collecting = true;
      // std::cout << "Packet starting on index " << writeIndex;
    }

    if (_initial_wait > 0)
      _initial_wait--;

    // Add sample to shift register
    std::rotate(_sample_history.rbegin(), _sample_history.rbegin() + 1, _sample_history.rend());
    _sample_history.at(0) = in[samp_i];
    
    // Output sample if we are writing a packet
    if (_collecting) {
      if (_n_collected == _packet_len) {
        _collecting = false;
        break;  // Break when we have taken one packet from the input buffer so that we do not get 2 packets in one loop
      } else {
        _packet_buf.push_back(in[samp_i]);
        _n_collected++;
      }
    }
  }

  // Tell runtime system how many inputs we consumed
  consume(0, samp_i);

  // Assumes that noutput_items > 0
  if (!_collecting && _n_collected == _packet_len) {
    for (int i = 0; i < _packet_len; i++) {
      out[i] = _packet_buf[i];
    }
    _packet_buf.clear();
    _n_collected = 0;
    produce(0, 1);
  } else {
    produce(0, 0);
  }

  return WORK_CALLED_PRODUCE;
}

} /* namespace radar_dsp */
} /* namespace gr */
