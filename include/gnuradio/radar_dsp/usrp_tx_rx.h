/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_USRP_TX_RX_H
#define INCLUDED_RADAR_DSP_USRP_TX_RX_H

#include <gnuradio/radar_dsp/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar_dsp {

    /*!
     * \brief <+description of block+>
     * \ingroup radar_dsp
     *
     */
    class RADAR_DSP_API usrp_tx_rx : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<usrp_tx_rx> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar_dsp::usrp_tx_rx.
       *
       * To avoid accidental use of raw pointers, radar_dsp::usrp_tx_rx's
       * constructor is in a private implementation
       * class. radar_dsp::usrp_tx_rx::make is the public interface for
       * creating new instances.
       */
      static sptr make(int channel, float carrier_freq, float sampling_rate, int samps_per_sym, float gain, int packet_len, bool start_tx, const std::vector<gr_complex>& ts_buf);
    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_USRP_TX_RX_H */
