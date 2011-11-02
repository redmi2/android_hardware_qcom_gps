AM_CFLAGS = -Wundef \
         -Wno-trigraphs \
         -g -O0 \
         -fno-inline \
         -fno-short-enums \
         -fpic

libgps_utils_so_la_h_sources = log_util.h \
            msg_q.h \
            linked_list.h \
            loc_cfg.h \
            loc_log.h

libgps_utils_so_la_c_sources = linked_list.c \
            msg_q.c \
            loc_cfg.cpp \
            loc_log.cpp

library_includedir = $(pkgincludedir)

libgps_utils_so_la_SOURCES = $(libgps_utils_so_la_c_sources)

if USE_GLIB
libgps_utils_so_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libgps_utils_so_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libgps_utils_so_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libgps_utils_so_la_CFLAGS = $(AM_CFLAGS)
libgps_utils_so_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libgps_utils_so_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libgps_utils_so_la_LIBADD = $(requiredlibs) -lstdc++

#Create and Install libraries
lib_LTLIBRARIES = libgps_utils_so.la
