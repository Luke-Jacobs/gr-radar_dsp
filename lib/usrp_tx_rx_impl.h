/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_DSP_USRP_TX_RX_IMPL_H
#define INCLUDED_RADAR_DSP_USRP_TX_RX_IMPL_H

#include <gnuradio/radar_dsp/usrp_tx_rx.h>
#include <vector>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

namespace gr {
  namespace radar_dsp {

    class usrp_tx_rx_impl : public usrp_tx_rx
    {
     private:
      float d_carrier_freq, d_sampling_rate, d_gain;
      int d_samps_per_sym;
      bool d_tx_mode;
      int d_packet_len;
      const std::vector<gr_complex> d_ts_buf;
      std::vector<gr_complex> d_ts_mf_buf;
      uhd::rx_streamer::sptr d_rx_stream;
      uhd::tx_streamer::sptr d_tx_stream;
      uhd::usrp::multi_usrp::sptr d_usrp;

     public:
      usrp_tx_rx_impl(float carrier_freq, float sampling_rate, int samps_per_sym, float gain, int packet_len, bool start_tx, const std::vector<gr_complex>& ts_buf);
      ~usrp_tx_rx_impl();

      bool start();

      // Sends packet_len number of samples stored in `in`
      void usrp_tx_rx_impl::send_packet(const std::vector<gr_complex>& buf, int repetition);

      // Wait for a packet until one comes
      size_t usrp_tx_rx_impl::recv_into_buf(std::vector<gr_complex> &buf, const size_t n);

      void usrp_tx_rx_impl::start_continuous_streaming();

      void usrp_tx_rx_impl::stop_continuous_streaming();

      // Where all the action really happens
      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace radar_dsp
} // namespace gr

#endif /* INCLUDED_RADAR_DSP_USRP_TX_RX_IMPL_H */
