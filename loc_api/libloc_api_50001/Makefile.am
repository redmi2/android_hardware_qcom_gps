AM_CFLAGS = -I${WORKSPACE}/hardware/libhardware/include/ \
            -I${WORKSPACE}/system/core/include/ \
            -I../../utils \
            -DHAVE_LIBLOG

requiredlibs = ../../utils/libgps_utils_so.la
h_sources = \
   LocApiAdapter.h \
   loc.h \
   loc_eng.h \
   loc_eng_xtra.h \
   loc_eng_ni.h \
   loc_eng_msg.h \
   loc_eng_msg_id.h \
   loc_eng_log.h

c_sources = loc_eng_log.cpp \
            LocApiAdapter.cpp

library_includedir = $(pkgincludedir)
library_include_HEADERS = $(h_sources)

libloc_adapter_so_la_SOURCES = $(c_sources) $(h_sources)

if USE_GLIB
libloc_adapter_so_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libloc_adapter_so_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libloc_adapter_so_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libloc_adapter_so_la_CFLAGS = $(AM_CFLAGS)
libloc_adapter_so_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libloc_adapter_so_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libloc_adapter_so_la_LIBADD = $(requiredlibs) -lstdc++

#Create and Install librariese
lib_LTLIBRARIES = libloc_adapter_so.la
