/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "energy_trigger_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace radar_dsp {

energy_trigger::sptr
energy_trigger::make(float energy_threshold, int buf_len_before, int buf_len_at_after)
{
    return gnuradio::make_block_sptr<energy_trigger_impl>(energy_threshold, buf_len_before, buf_len_at_after);
}

/*
 * The private constructor
 */
energy_trigger_impl::energy_trigger_impl(float energy_threshold,
                                         int buf_len_before,
                                         int buf_len_at_after)
    : gr::block("energy_trigger",
                gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(gr_complex)),
                gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, (buf_len_before+buf_len_at_after)*sizeof(gr_complex))),
      d_buf_len_before(buf_len_before),
      d_buf_len_at_after(buf_len_at_after),
      d_energy_threshold(energy_threshold),
      d_collecting(false)
{
  set_history(d_buf_len_before + 1);  // Stores buf_len_before number of samples plus at least one new one
  d_buf.clear();
}

/*
 * Our virtual destructor.
 */
energy_trigger_impl::~energy_trigger_impl() {}

void energy_trigger_impl::forecast(int noutput_items,
                                   gr_vector_int& ninput_items_required)
{
  ninput_items_required[0] = noutput_items*(d_buf_len_before + d_buf_len_at_after);
}

int energy_trigger_impl::general_work(int noutput_items,
                                      gr_vector_int& ninput_items,
                                      gr_vector_const_void_star& input_items,
                                      gr_vector_void_star& output_items)
{
  auto in = static_cast<const gr_complex *>(input_items[0]);
  auto out = static_cast<gr_complex *>(output_items[0]);

  int n_new_items = ninput_items[0] - d_buf_len_before;
  for (int samp_i = 0; samp_i < n_new_items; samp_i++) {
    if (!d_collecting && (std::abs(in[samp_i+d_buf_len_before]) > d_energy_threshold)) {
      std::cout << "Triggered\n";
      d_buf.insert(d_buf.end(), in+samp_i, in+samp_i+d_buf_len_before);  // Copy d_buf_len_before number of samples into the buffer
      d_collecting = true;
      d_samps_to_write = d_buf_len_at_after;
    }
    // If in writing mode, write this sample to output buffer
    if (d_collecting && (d_samps_to_write > 0)) {
      d_buf.push_back(in[samp_i+d_buf_len_before]);
      d_samps_to_write--;
    }
    // If finished writing
    if (d_collecting && (d_samps_to_write < 1)) {
      d_collecting = false;
      std::copy(d_buf.begin(), d_buf.end(), out);
      d_buf.clear();
      consume(0, samp_i);
      return 1;
    }
  }

  // Tell runtime system how many input items we consumed on each input stream.
  consume(0, n_new_items);

  // Tell runtime system how many output items we produced.
  return 0;
}

} /* namespace radar_dsp */
} /* namespace gr */