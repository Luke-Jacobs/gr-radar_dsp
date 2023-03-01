/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_H
#define INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API harmonic_retrieval : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<harmonic_retrieval> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::harmonic_retrieval.
       *
       * To avoid accidental use of raw pointers, radar_dsp::harmonic_retrieval's
       * constructor is in a private implementation
       * class. radar_dsp::harmonic_retrieval::make is the public interface for
       * creating new instances.
       */
      static sptr make(int samp_rate, int fft_size);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_HARMONIC_RETRIEVAL_H */
