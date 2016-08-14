AM_CFLAGS = -std=gnu99 $(UHD_CFLAGS) -g
AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CXXFLAGS = $(UHD_CFLAGS) -std=gnu++11 -g

SIGPROC_LA = $(top_builddir)/src/sigproc/libsigproc.la
OPENPHY_LTE_LA = $(top_builddir)/src/libopenphy_lte.la
OPENPHY_IO_LA = $(top_builddir)/src/libopenphy_io.la

bin_PROGRAMS = lte_decode
noinst_PROGRAMS = dcitest

dcitest_SOURCES = dcitest.c
dcitest_LDADD = $(OPENPHY_LTE_LA)

lte_decode_SOURCES = io_subframe.cc lte_decode.cc sync.cc rx_proc.cc rrc.cc
lte_decode_LDADD = $(OPENPHY_LTE_LA) $(OPENPHY_IO_LA) $(SIGPROC_LA) $(FFTWF_LIBS) $(UHD_LIBS) $(OPENFEC_LIBS) -lboost_system
lte_decode_LDFLAGS = -pthread
