/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_H
#define INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API phase_measurement : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<phase_measurement> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::phase_measurement.
       *
       * To avoid accidental use of raw pointers, radar_dsp::phase_measurement's
       * constructor is in a private implementation
       * class. radar_dsp::phase_measurement::make is the public interface for
       * creating new instances.
       */
      static sptr make(std::vector<float>& tone_frequencies, int n_tones, int vector_length, float samp_rate);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PHASE_MEASUREMENT_H */
