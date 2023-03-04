/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_ENERGY_TRIGGER_H
#define INCLUDED_RADAR_DSP_ENERGY_TRIGGER_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API energy_trigger : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<energy_trigger> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::energy_trigger.
       *
       * To avoid accidental use of raw pointers, radar_dsp::energy_trigger's
       * constructor is in a private implementation
       * class. radar_dsp::energy_trigger::make is the public interface for
       * creating new instances.
       */
      static sptr make(float energy_threshold, int buf_len_before, int buf_len_at_after);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_ENERGY_TRIGGER_H */
