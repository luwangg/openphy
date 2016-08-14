AM_CFLAGS = -Wall -std=c99 -fvisibility=hidden $(OPENFEC_CFLAGS) $(UHD_CFLAGS) -pthread -I$(top_srcdir)/include $(WIRESHARK_CFLAGS) -g -march=native
AM_CPPFLAGS = $(UHD_CFLAGS)
AM_CXXFLAGS = $(UHD_CFLAGS) -Wall -std=gnu++11 -I$(top_srcdir)/include -g -march=native

SIGPROC_LA = $(top_builddir)/src/sigproc/libsigproc.la

SUBDIRS = sigproc

noinst_LTLIBRARIES = libopenphy_lte.la libopenphy_io.la

libopenphy_lte_la_SOURCES = \
	lte.cc \
	pss.c \
	sss.c \
	ref.c \
	crc.c \
	pbch.c \
	pbch_block.c \
	pcfich.c \
	phich.c \
	pdcch.c \
	pdcch_block.c \
	pdcch_interleave.c \
	qam.c \
	precode.c \
	ofdm.c \
	slot.c \
	gold.c \
	sync.c \
	sync_pss.c \
	dci.c \
	dci_formats.c \
	scramble.c \
	expand.c \
	pdsch.c \
	pdsch_vrb.c \
	pdsch_riv.c \
	pdsch_tbs.c \
	pdsch_block.c \
	log.c \
	wireshark.c

libopenphy_io_la_SOURCES = \
	Resampler.cc \
	uhd.cc \
	io.cc \
	buffer.cc
