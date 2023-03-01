/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_LLS_H
#define INCLUDED_RADAR_DSP_LLS_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API lls : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<lls> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::lls.
       *
       * To avoid accidental use of raw pointers, radar_dsp::lls's
       * constructor is in a private implementation
       * class. radar_dsp::lls::make is the public interface for
       * creating new instances.
       */
      static sptr make(float f_c, std::vector<float>& f_i, int n_tones);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_LLS_H */
