/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "selfref_framesync_impl.h"
#include <cassert>
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>

namespace gr {
  namespace radar_dsp {

    using input_type = gr_complex;
    using output_type = gr_complex;

    selfref_framesync::sptr
    selfref_framesync::make(int training_seq_len, int packet_len, float threshold)
    {
      return gnuradio::make_block_sptr<selfref_framesync_impl>(training_seq_len, packet_len, threshold);
    }

    /*
     * The private constructor
     */
    selfref_framesync_impl::selfref_framesync_impl(int training_seq_len, int packet_len, float threshold)
      : gr::block("selfref_framesync",
                  gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                  gr::io_signature::make3(3 /* min outputs */, 3 /*max outputs */, 
                                          sizeof(output_type)*packet_len, sizeof(float), sizeof(output_type)*training_seq_len)),
        d_training_seq_len(training_seq_len),
        d_packet_len(packet_len),
        d_threshold(threshold),
        d_items_to_write(0),
        d_prev_corr(0.0),
        d_packet_detected(false)
    {
      set_history(d_training_seq_len+1);  // Ensures that if we just get 1 new sample, we will still have access to the previous N-1 samples
      assert(training_seq_len % 2 == 0);
      d_packet_buf.clear();
      d_training_buf.assign(d_training_seq_len, gr_complex(0.0, 0.0));
      std::cout << d_training_buf.size() << "\n";
    }

    /*
     * Our virtual destructor.
     */
    selfref_framesync_impl::~selfref_framesync_impl()
    {
    }

    void
    selfref_framesync_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items*d_packet_len;
    }

    int
    selfref_framesync_impl::general_work (int noutput_items,
                                          gr_vector_int &ninput_items,
                                          gr_vector_const_void_star &input_items,
                                          gr_vector_void_star &output_items)
    {
      auto in = static_cast<const gr_complex *>(input_items[0]);
      auto out_packet = static_cast<gr_complex *>(output_items[0]);
      auto out_corr = static_cast<float *>(output_items[1]);
      auto out_rx_ts = static_cast<gr_complex *>(output_items[2]);
      int half_len = d_training_seq_len/2, samp_i = 0;
      
      // Perform frame self-referencing starting with the first new sample
      int newinput_items = ninput_items[0] - d_training_seq_len;
      for (samp_i = 0; samp_i < newinput_items; samp_i++) {
        // Take the two halves of the training sequence and correlate them together
        const gr_complex *tr_seq_start = in+samp_i+1;  // The +1 ensures that the correlation happens one sample after the longest-stored sample
        arma::cx_fvec first_half = arma::cx_fvec(std::vector<gr_complex>(tr_seq_start, tr_seq_start+half_len));
        arma::cx_fvec second_half = arma::cx_fvec(std::vector<gr_complex>(tr_seq_start+half_len, tr_seq_start+d_training_seq_len));
        float seq_power = arma::sum(arma::abs(first_half.t() * first_half)) + arma::sum(arma::abs(second_half.t() * second_half));
        float corr = std::abs(arma::sum(first_half.t() * second_half)) / seq_power * 2.0;

        out_corr[samp_i] = corr;
        
        // Trigger immediate packet writing, since we are 1 sample beyond the training sequence correlation peak
        if (!d_packet_detected && (d_prev_corr > d_threshold) && (corr < d_prev_corr)) {
          d_items_to_write = d_packet_len;  // This is our "timer" for the number of packet samples that we still need to write
          d_packet_detected = true;
          // Assume that the first d_training_seq_len symbols in the buffer are the received training sequence
          for (int i = 0; i < d_training_seq_len; i++) {
            out_rx_ts[i] = in[i+samp_i];
          }
          produce(2, 1);
        // If we are not at the start of a packet
        } else {
          produce(2, 0);
        }

        d_prev_corr = corr;  // Update correlation history

        // Add to sample buffer if we are currently capturing a packet
        if (d_packet_detected) {
          d_packet_buf.push_back(in[samp_i+d_training_seq_len]);
          d_items_to_write--;
          if (d_items_to_write < 1) {
            d_packet_detected = false;
            break;  // We do not want to start another packet if we have just finished one
          }
        }
      }
      
      assert(samp_i > 0);
      produce(1, samp_i);  // Produce this many correlation values
      consume(0, samp_i);  // This records only the new samples we are reading

      // Dump the packet to output if we have collected it fully
      if (!d_packet_detected && (d_packet_buf.size() == d_packet_len)) {
        // std::cout << "D";
        // memcpy_s(out_packet, noutput_items*d_packet_len*sizeof(gr_complex), d_packet_buf.data(), d_packet_len*sizeof(gr_complex));
        for (int write_i = 0; write_i < d_packet_len; write_i++) {
          out_packet[write_i] = d_packet_buf[write_i];
        }
        d_packet_buf.clear();
        produce(0, 1);
        return WORK_CALLED_PRODUCE;  // Produce 1 packet
      }

      // Tell runtime system how many output items we produced.
      produce(0, 0);
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace radar_dsp */
} /* namespace gr */
