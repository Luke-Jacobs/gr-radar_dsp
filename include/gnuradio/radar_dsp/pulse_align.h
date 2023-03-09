/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PULSE_ALIGN_H
#define INCLUDED_RADAR_DSP_PULSE_ALIGN_H

#include <gnuradio/block.h>
#include <gnuradio/radar_dsp/api.h>

namespace gr {
namespace radar_dsp {

/*!
 * \brief <+description of block+>
 * \ingroup radar_dsp
 *
 */
class RADAR_DSP_API pulse_align : virtual public gr::block
{
public:
    typedef std::shared_ptr<pulse_align> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of radar_dsp::pulse_align.
     *
     * To avoid accidental use of raw pointers, radar_dsp::pulse_align's
     * constructor is in a private implementation
     * class. radar_dsp::pulse_align::make is the public interface for
     * creating new instances.
     */
    static sptr make(int input_buffer_len, float samp_rate, int samps_per_sym);
};

} // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PULSE_ALIGN_H */
