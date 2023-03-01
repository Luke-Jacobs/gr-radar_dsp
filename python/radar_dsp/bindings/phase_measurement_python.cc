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
/* BINDTOOL_HEADER_FILE(phase_measurement.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(1d45936e87baf84d9f99cbf225732e36)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/radar_dsp/phase_measurement.h>
// pydoc.h is automatically generated in the build directory
#include <phase_measurement_pydoc.h>

void bind_phase_measurement(py::module& m)
{

    using phase_measurement    = gr::radar_dsp::phase_measurement;


    py::class_<phase_measurement, gr::block, gr::basic_block,
        std::shared_ptr<phase_measurement>>(m, "phase_measurement", D(phase_measurement))

        .def(py::init(&phase_measurement::make),
           D(phase_measurement,make)
        )
        



        ;




}








