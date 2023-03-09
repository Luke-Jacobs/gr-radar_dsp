/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/pybind11.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace py = pybind11;

// Headers for binding functions
/**************************************/
// The following comment block is used for
// gr_modtool to insert function prototypes
// Please do not delete
/**************************************/
// BINDING_FUNCTION_PROTOTYPES(
    void bind_lls(py::module& m);
    void bind_packetExtractor(py::module& m);
    void bind_harmonic_retrieval(py::module& m);
    void bind_phase_measurement(py::module& m);
    void bind_selfref_framesync(py::module& m);
    void bind_usrp_tx_rx(py::module& m);
    void bind_cfo_estimator(py::module& m);
    void bind_channel_estimator(py::module& m);
    void bind_energy_trigger(py::module& m);
    void bind_pulse_align(py::module& m);
// ) END BINDING_FUNCTION_PROTOTYPES


// We need this hack because import_array() returns NULL
// for newer Python versions.
// This function is also necessary because it ensures access to the C API
// and removes a warning.
void* init_numpy()
{
    import_array();
    return NULL;
}

PYBIND11_MODULE(radar_dsp_python, m)
{
    // Initialize the numpy C API
    // (otherwise we will see segmentation faults)
    init_numpy();

    // Allow access to base block methods
    py::module::import("gnuradio.gr");

    /**************************************/
    // The following comment block is used for
    // gr_modtool to insert binding function calls
    // Please do not delete
    /**************************************/
    // BINDING_FUNCTION_CALLS(
    bind_lls(m);
    bind_packetExtractor(m);
    bind_harmonic_retrieval(m);
    bind_phase_measurement(m);
    bind_selfref_framesync(m);
    bind_usrp_tx_rx(m);
    bind_cfo_estimator(m);
    bind_channel_estimator(m);
    bind_energy_trigger(m);
    bind_pulse_align(m);
    // ) END BINDING_FUNCTION_CALLS
}