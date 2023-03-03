/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_CFO_ESTIMATOR_H
#define INCLUDED_RADAR_DSP_CFO_ESTIMATOR_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace radar_dsp {

/*!
 * \brief <+description of block+>
 * \ingroup radar_dsp
 *
 */
class RADAR_DSP_API cfo_estimator : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<cfo_estimator> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of radar_dsp::cfo_estimator.
     *
     * To avoid accidental use of raw pointers, radar_dsp::cfo_estimator's
     * constructor is in a private implementation
     * class. radar_dsp::cfo_estimator::make is the public interface for
     * creating new instances.
     */
    static sptr make(int training_seq_len, float samp_rate);
};

} // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_CFO_ESTIMATOR_H */
