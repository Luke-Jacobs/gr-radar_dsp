/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_PACKETEXTRACTOR_H
#define INCLUDED_RADAR_DSP_PACKETEXTRACTOR_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API packetExtractor : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<packetExtractor> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::packetExtractor.
       *
       * To avoid accidental use of raw pointers, radar_dsp::packetExtractor's
       * constructor is in a private implementation
       * class. radar_dsp::packetExtractor::make is the public interface for
       * creating new instances.
       */
      static sptr make(int sample_history_len, int energy_factor_threshold, int packet_len);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_PACKETEXTRACTOR_H */
