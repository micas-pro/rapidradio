#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT(rapidradio, 1.0.1, bugs@rapidradio.pl)
AM_INIT_AUTOMAKE(rapidradio, 1.0.1)
AC_OUTPUT(Makefile src/Makefile scripts/Makefile)
AC_CONFIG_SRCDIR([src/tools/librapidradio/librapidradio.cpp])
AC_CONFIG_SRCDIR([src/tools/rapidradio/rapidradio.cpp])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_RANLIB
AC_CHECK_PROG(PERL, perl, perl)

# Checks for libraries.
AC_CHECK_LIB(bcm2835, bcm2835_init)

# Checks for header files.
AC_CHECK_HEADERS([inttypes.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_INT32_T
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_CHECK_FUNCS([strerror])

AC_OUTPUT
