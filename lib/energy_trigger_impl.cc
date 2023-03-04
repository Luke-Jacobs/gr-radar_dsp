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
      d_buf_len_at_after(buf_len_at_after)
{
  set_history(buf_len_before+1);  // Stores buf_len_before number of samples plus at least one new one
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

  // for (int samp_i = 0; samp_i < )

  // Tell runtime system how many input items we consumed on each input stream.
  consume_each(ninput_items[0] - d_buf_len_before);

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace radar_dsp */
} /* namespace gr */