AM_CPPFLAGS = -Wall -O2 -I. -I./librapidradio

lib_LIBRARIES = librapidradio.a
librapidradio_a_SOURCES = librapidradio/librapidradio.cpp

include_HEADERS = librapidradio/rapidradio.h

bin_PROGRAMS = rapidradio
rapidradio_SOURCES = tools/rapidradio/rapidradio.cpp
rapidradio_LDFLAGS = -L.
rapidradio_LDADD = -lrapidradio


