# Makefile.am - Automake script for gps GNSS shippable test app
#

ACLOCAL_AMFLAGS = -I m4

AM_CFLAGS = \
    -DDEBUG \
    -I$(WORKSPACE)/hardware/qcom/gps/loc_api/libloc_api_50001 \
    $(LOC_CFLAGS) \
    $(LOCVZW_CFLAGS) \
    $(GTI_CFLAGS) \
    $(LOCXTRA_CFLAGS) \
    $(LOCATIONSERVICE_CFLAGS) \
    -DTEST_ULP

required_libs = \
    $(LOCHAL_LIBS)

gnss_c_sources = gnss_test.cpp

gnss_test_SOURCES = $(gnss_c_sources)

if USE_GLIB
gnss_test_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
gnss_test_LDFLAGS = -lstdc++ -g -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
gnss_test_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
gnss_test_CFLAGS = $(AM_CFLAGS)
gnss_test_LDFLAGS = -lpthread -shared -version-info 1:0:0
gnss_test_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

gnss_test_LDADD = -lcutils -ldl $(LOC_LIBS) $(required_libs)

bin_PROGRAMS = gnss_test

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = gnss-test.pc
EXTRA_DIST = $(pkgconfig_DATA)
