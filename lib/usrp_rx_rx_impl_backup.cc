/* -*- c++ -*- */
/*
 * Copyright 2023 Luke Jacobs.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "usrp_tx_rx_impl.h"
#include <gnuradio/io_signature.h>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
// #include <boost/program_options.hpp>
// #include <boost/format.hpp>
// #include <boost/thread.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

namespace gr {
namespace radar_dsp {

using input_type = gr_complex;
using output_type = gr_complex;

usrp_tx_rx::sptr usrp_tx_rx::make()
{
    return gnuradio::make_block_sptr<usrp_tx_rx_impl>();
}


/*
 * The private constructor
 */
usrp_tx_rx_impl::usrp_tx_rx_impl()
    : gr::block("usrp_tx_rx",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
{
}

/*
 * Our virtual destructor.
 */
usrp_tx_rx_impl::~usrp_tx_rx_impl() {}

bool usrp_tx_rx_impl::start() {
  uhd::set_thread_priority_safe();

  uhd::device_addr_t device_args("");
  std::string subdev("A:0 B:0");
  // std::string ant("TX/RX");
  std::string ref("internal");

  double rate(1e6);
  double freq(2e9);
  double gain(10);
  double bw(1e6);

  //create a usrp device
  std::cout << "Searching for all USRP devices..." << std::endl;
  uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);

  // Lock mboard clocks
  std::cout << "Lock mboard clocks: " << ref << std::endl;
  usrp->set_clock_source(ref);
  
  //always select the subdevice first, the channel mapping affects the other settings
  std::cout << "subdev set to: " << subdev << std::endl;
  usrp->set_rx_subdev_spec(subdev);
  std::cout << "Using Device: " << usrp->get_pp_string() << std::endl;

  //set the sample rate
  if (rate <= 0.0) {
      std::cerr << "Please specify a valid sample rate" << std::endl;
      return false;
  }

  // set sample rate
  usrp->set_rx_rate(rate);
  std::cout << "Actual RX Rate: " << usrp->get_rx_rate() / 1e6 << " Msps..." << std::endl;

  // set the rf gain
  // usrp->set_rx_gain(gain);
  // std::cout << "Actual RX Gain: " << usrp->get_rx_gain() << " dB..." << std::endl << std::endl;

  // set the IF filter bandwidth
  // usrp->set_rx_bandwidth(bw);
  // std::cout << "Actual RX Bandwidth: " << usrp->get_rx_bandwidth() / 1e6 << "MHz..." << std::endl << std::endl;

  // set the antenna
  // usrp->set_rx_antenna(ant);
  // std::cout << "Actual RX Antenna: " << usrp->get_rx_antenna() << std::endl << std::endl;

  // 
  usrp->set_clock_source("internal");
  usrp->set_time_source("internal");

  // Reset timing
  usrp->set_time_next_pps(uhd::time_spec_t(0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Use the timed command interface to send a timed command to both RX2 channels
  usrp->clear_command_time();
  usrp->set_command_time(usrp->get_time_now() + uhd::time_spec_t(0.1));
  uhd::tune_request_t tune_request(freq);
  usrp->set_rx_freq(tune_request, 0);  // this command will be sent synchronously
  usrp->set_rx_freq(tune_request, 1);  // this command will be sent synchronously
  std::this_thread::sleep_for(std::chrono::milliseconds(110));  // sleep 110ms (~10ms after retune occurs) to allow LO to lock
  usrp->clear_command_time();

  // create a receive streamer
  uhd::stream_args_t stream_args("fc32"); // complex floats
  stream_args.channels             = std::vector<uint64_t>({0,1});
  uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

  // setup streaming
  std::cout << "Setting up RX streamer\n";
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
  stream_cmd.num_samps  = 1000;
  stream_cmd.stream_now = false;
  stream_cmd.time_spec  = uhd::time_spec_t(usrp->get_time_now() + uhd::time_spec_t(0.5));
  rx_stream->issue_stream_cmd(stream_cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // receive num_samps samples
  uhd::rx_metadata_t md;
  std::vector<std::complex<float>> buff_ch0(stream_cmd.num_samps), buff_ch1(stream_cmd.num_samps);
  std::vector<std::complex<float> *> both_buffs;  //(2, std::vector<std::complex<float>>(stream_cmd.num_samps));
  both_buffs.push_back(&buff_ch0.front());
  both_buffs.push_back(&buff_ch1.front());
  std::cout << "RX streamer has " << rx_stream->get_num_channels() << " channels\n";
  for (int i = 0; i < 1; i++) {
    const size_t num_rx_samps = rx_stream->recv(both_buffs, stream_cmd.num_samps, md, 3.0);
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
      throw std::runtime_error("Receiver error");
    }
    std::cout << "Received " << num_rx_samps << " samples\n";
  }

  // write samples to file
  std::ofstream outfile;
  outfile.open("output_samples.bin", std::ofstream::binary);
  if (outfile.is_open()) {
    std::cout << "Writing " << buff_ch0.size() << " samples to both channels...\n";
    outfile.write((const char *)both_buffs[0], buff_ch0.size() * sizeof(std::complex<float>));
    outfile.write((const char *)both_buffs[1], buff_ch1.size() * sizeof(std::complex<float>));
    outfile.close();
    std::cout << "Written to file\n";
  } else {
    std::cout << "Problem writing output file!\n";
  }

  return true;
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
  auto in = static_cast<const input_type*>(input_items[0]);
  auto out = static_cast<output_type*>(output_items[0]);

  // Tell runtime system how many input items we consumed on
  // each input stream.
  consume_each(noutput_items);

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace radar_dsp */
} /* namespace gr */
