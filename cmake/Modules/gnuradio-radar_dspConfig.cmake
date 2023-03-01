find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_RADAR_DSP gnuradio-radar_dsp)

FIND_PATH(
    GR_RADAR_DSP_INCLUDE_DIRS
    NAMES gnuradio/radar_dsp/api.h
    HINTS $ENV{RADAR_DSP_DIR}/include
        ${PC_RADAR_DSP_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_RADAR_DSP_LIBRARIES
    NAMES gnuradio-radar_dsp
    HINTS $ENV{RADAR_DSP_DIR}/lib
        ${PC_RADAR_DSP_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-radar_dspTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_RADAR_DSP DEFAULT_MSG GR_RADAR_DSP_LIBRARIES GR_RADAR_DSP_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_RADAR_DSP_LIBRARIES GR_RADAR_DSP_INCLUDE_DIRS)
