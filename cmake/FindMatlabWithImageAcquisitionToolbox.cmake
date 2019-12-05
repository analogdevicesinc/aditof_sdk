# This cmake script will try to find Matlab using the find_package command
# and also set some extra variables to point to the mwimaqmex lib and the
# imaqadaptors kit include directory:
# Matlab_MWIMAQMEX_INCLUDES and Matlab_MWIMAQMEX_LIBRARY

find_package(Matlab REQUIRED COMPONENTS MX_LIBRARY)

# find_package is able to find the mx library and set Matlab_MX_LIBRARY to point at it,
# but not the mwimaqmex lib. The only difference in the path is the name of the lib
# so we just replace libmx with libmwimaqmex in Matlab_MX_LIBRARY. In this way we also
# preserve the suffix (.so, .dll, .dylib) and the architecture (glnxa64, maci64 ... ) w/o
# hardcoding anything

if(WIN32)
    string(REPLACE "libmx"
        "imaqmex" Matlab_MWIMAQMEX_LIBRARY ${Matlab_MX_LIBRARY})
    string(REPLACE "extern"
        "toolbox/imaq/imaqadaptors/kit" Matlab_MWIMAQMEX_LIBRARY ${Matlab_MWIMAQMEX_LIBRARY})
    string(REPLACE "microsoft"
        "release" Matlab_MWIMAQMEX_LIBRARY ${Matlab_MWIMAQMEX_LIBRARY})

else()
    string(REPLACE "libmx"
        "libmwimaqmex" Matlab_MWIMAQMEX_LIBRARY ${Matlab_MX_LIBRARY})
endif()

# There is no problem when hardcoding this include dir, because it stays the same on every
# platform (Windows, Mac, Linux)
set(Matlab_MWIMAQMEX_INCLUDES "${Matlab_ROOT_DIR}/toolbox/imaq/imaqadaptors/kit/include")
