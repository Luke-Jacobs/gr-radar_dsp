/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_H
#define INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API selfref_framesync : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<selfref_framesync> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::selfref_framesync.
       *
       * To avoid accidental use of raw pointers, radar_dsp::selfref_framesync's
       * constructor is in a private implementation
       * class. radar_dsp::selfref_framesync::make is the public interface for
       * creating new instances.
       */
      static sptr make(int training_seq_len, int packet_len, float threshold);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_SELFREF_FRAMESYNC_H */
