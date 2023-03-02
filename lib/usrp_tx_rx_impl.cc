/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "usrp_tx_rx_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>

namespace gr {
namespace radar_dsp {

usrp_tx_rx::sptr usrp_tx_rx::make(float carrier_freq, float sampling_rate, float gain, int packet_len, bool start_tx, const std::vector<gr_complex>& ts_buf)
{
    return gnuradio::make_block_sptr<usrp_tx_rx_impl>(carrier_freq, sampling_rate, gain, packet_len, start_tx, ts_buf);
}

/*
 * The private constructor
 */
usrp_tx_rx_impl::usrp_tx_rx_impl(float carrier_freq, float sampling_rate, float gain, int packet_len, bool start_tx, const std::vector<gr_complex>& ts_buf)
    : // the input is a sample vector (a single packet, a training sequence)
      // the output is a sample stream from the USRP, given to self-ref framesync
      gr::block("usrp_tx_rx",
                gr::io_signature::make(0 /* min inputs */, 0 /* max inputs */, 0),
                gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(gr_complex))),
      d_carrier_freq(carrier_freq),
      d_sampling_rate(sampling_rate),
      d_gain(gain),
      d_tx_mode(start_tx),
      d_packet_len(packet_len),
      d_ts_buf(ts_buf)
{
  std::cout << "Carrier freq:" << carrier_freq << " Sampling rate:" << sampling_rate << " Gain:" << gain << "Start in TX Mode:" << start_tx << " Packet length:" << packet_len << "\n";

  // Setup message passing
  message_port_register_in(pmt::intern("msg"));
}

/*
 * Our virtual destructor.
 */
usrp_tx_rx_impl::~usrp_tx_rx_impl() {}

bool usrp_tx_rx_impl::start() {
  uhd::set_thread_priority_safe();

  uhd::device_addr_t device_args("");
  std::string subdev("A:0 B:0");
  std::string ref("internal");

  // Create a usrp device
  std::cout << "Searching for all USRP devices..." << std::endl;
  d_usrp = uhd::usrp::multi_usrp::make(device_args);

  // Lock mboard clocks
  std::cout << "Lock mboard clocks: " << ref << std::endl;
  d_usrp->set_clock_source(ref);
  
  // Always select the subdevice first, the channel mapping affects the other settings
  std::cout << "subdev set to: " << subdev << std::endl;
  d_usrp->set_rx_subdev_spec(subdev);
  std::cout << "Using Device: " << d_usrp->get_pp_string() << std::endl;

  // Set the sample rate
  if (d_sampling_rate <= 0.0) {
      std::cerr << "Please specify a valid sample rate" << std::endl;
      return false;
  }

  // Set the TX gain
  d_usrp->set_tx_gain(d_gain, 1);

  // Set sample rate
  d_usrp->set_rx_rate(d_sampling_rate, 0);
  d_usrp->set_tx_rate(d_sampling_rate, 1);
  std::cout << "Actual RX Rate: " << d_usrp->get_rx_rate() / 1e6 << " Msps..." << std::endl;
  d_usrp->set_clock_source("internal");
  d_usrp->set_time_source("internal");

  // Reset timing
  d_usrp->set_time_next_pps(uhd::time_spec_t(0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Use the timed command interface to send a timed command to both channels
  d_usrp->clear_command_time();
  d_usrp->set_command_time(d_usrp->get_time_now() + uhd::time_spec_t(0.1));
  uhd::tune_request_t tune_request(d_carrier_freq);
  d_usrp->set_rx_freq(tune_request, 0);  // this command will be sent synchronously
  d_usrp->set_tx_freq(tune_request, 1);  // this command will be sent synchronously
  std::this_thread::sleep_for(std::chrono::milliseconds(110));  // sleep 110ms (~10ms after retune occurs) to allow LO to lock
  d_usrp->clear_command_time();

  // create streams
  uhd::stream_args_t stream_args("fc32"); // complex floats
  stream_args.channels = {0};  // RF 0 is RX
  d_rx_stream = d_usrp->get_rx_stream(stream_args);
  stream_args.channels = {1};  // RF 1 is TX
  d_tx_stream = d_usrp->get_tx_stream(stream_args);
  std::cout << "Set up TX and RX streams\n";

  // setup continous streaming if put into receive mode
  if (!d_tx_mode) {
    start_continuous_streaming();
  }

  return true;
}

void usrp_tx_rx_impl::send_packet(const std::vector<gr_complex>& buf, int repetition = 1) {
  uhd::tx_metadata_t md;
  md.start_of_burst = false;
  md.end_of_burst = false;
  md.has_time_spec = true;
  md.time_spec = d_usrp->get_time_now() + uhd::time_spec_t(0.1);
  // std::cout << ".";
  size_t num_sent_samps = 0;
  for (int i = 0; i < repetition; i++)
    num_sent_samps += d_tx_stream->send(&buf.front(), buf.size(), md, 1);
  if (num_sent_samps != buf.size()*repetition) {
    throw std::runtime_error("The number of sent samples does not equal the desired packet length");
  }
  md.time_spec = d_usrp->get_time_now() + uhd::time_spec_t(0.1);
  md.end_of_burst = true;
  d_tx_stream->send("", 0, md);
}

size_t usrp_tx_rx_impl::recv_into_buf(std::vector<gr_complex> &buf, const size_t n) {
  uhd::rx_metadata_t md;

  // ===== Only used for multi-channel reception =====
  // std::vector<gr_complex *> buffs_ptrs(1);
  // buffs_ptrs.push_back(&buf.front());

  const size_t num_rx_samps = d_rx_stream->recv(&buf.front(), n, md, 3.0);

  // Error handling
  if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
    std::cout << "USRP Error: timeout while streaming\n" << std::flush;
    return 0;
  }
  if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
    std::cout << "USRP Error: overflow\n" << std::flush;
    return 0;
  }
  if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
    // NOTE: This may also be due to a timeout
    std::cout << "USRP Error: " << md.error_code << "\n" << std::flush;
    throw std::runtime_error("USRP Error: receiver error");
  }

  return num_rx_samps;
}

void usrp_tx_rx_impl::start_continuous_streaming() {
  // Send continuous streaming command
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.num_samps = 0;
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(d_usrp->get_time_now() + uhd::time_spec_t(0.5));
  d_rx_stream->issue_stream_cmd(stream_cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// This function should only be needed on exiting the program, not when switching between TX & RX
void usrp_tx_rx_impl::stop_continuous_streaming() {
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
  d_rx_stream->issue_stream_cmd(stream_cmd);
}

void usrp_tx_rx_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
  ninput_items_required[0] = noutput_items;
}

int usrp_tx_rx_impl::general_work(int noutput_items,
                                  gr_vector_int& ninput_items,
                                  gr_vector_const_void_star& input_items,
                                  gr_vector_void_star& output_items)
{
  // auto in = static_cast<const gr_complex *>(input_items[0]);
  auto out = static_cast<gr_complex *>(output_items[0]);

  if (d_tx_mode) {
    // Send internal training sequence
    send_packet(d_ts_buf, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    produce(0, 0);

    // Switch off to receive mode
    // d_tx_mode = false;
    // std::cout << "Switching to RX mode\n";
    // start_continuous_streaming();
  } else {
    // Pull samples from the USRP and write them to the temporary vector, then into the output
    std::vector<gr_complex> buf(noutput_items);
    size_t num_recv_samps = recv_into_buf(buf, noutput_items);

    int n_written_samps = std::min((int)num_recv_samps, noutput_items);
    std::copy(buf.begin(), buf.begin() + n_written_samps, out);
    
    produce(0, n_written_samps);
  }

  // Tell runtime system how many output items we produced.
  return WORK_CALLED_PRODUCE;
}

} /* namespace radar_dsp */
} /* namespace gr */
