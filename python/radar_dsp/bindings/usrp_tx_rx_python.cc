/*
 * Copyright 2023 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(usrp_tx_rx.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(21d6e2dd2dec2aed561a9a98b97db448)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/radar_dsp/usrp_tx_rx.h>
// pydoc.h is automatically generated in the build directory
#include <usrp_tx_rx_pydoc.h>

void bind_usrp_tx_rx(py::module& m)
{

    using usrp_tx_rx    = gr::radar_dsp::usrp_tx_rx;


    py::class_<usrp_tx_rx, gr::block, gr::basic_block,
        std::shared_ptr<usrp_tx_rx>>(m, "usrp_tx_rx", D(usrp_tx_rx))

        .def(py::init(&usrp_tx_rx::make),
           py::arg("carrier_freq"),
           py::arg("sampling_rate"),
           py::arg("samps_per_sym"),
           py::arg("gain"),
           py::arg("packet_len"),
           py::arg("start_tx"),
           py::arg("ts_buf"),
           D(usrp_tx_rx,make)
        )
        



        ;




}








