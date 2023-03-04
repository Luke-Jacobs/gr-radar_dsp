# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT radar_dsp_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT radar_dsp_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND radar_dsp_python_files
    lls_python.cc
    packetExtractor_python.cc
    harmonic_retrieval_python.cc
    phase_measurement_python.cc
    selfref_framesync_python.cc
    usrp_tx_rx_python.cc
    cfo_estimator_python.cc
    channel_estimator_python.cc
    energy_trigger_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(radar_dsp
   ../../..
   gr::radar_dsp
   "${radar_dsp_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(TARGET radar_dsp_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:radar_dsp_python>
    ${PROJECT_BINARY_DIR}/test_modules/gnuradio/radar_dsp/
)

install(TARGETS radar_dsp_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/radar_dsp COMPONENT pythonapi)
