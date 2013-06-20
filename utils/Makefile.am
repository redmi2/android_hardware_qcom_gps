AM_CFLAGS = -Wundef \
         -MD \
         -Wno-trigraphs \
         -g -O0 \
         -fno-inline \
         -fno-short-enums \
         -fpic \
	 -I../platform_lib_abstractions

libgps_utils_so_la_h_sources = loc_log.h \
	loc_cfg.h \
	log_util.h \
	linked_list.h \
	msg_q.h \
	loc_target.h \
	../platform_lib_abstractions/platform_lib_includes.h \
	../platform_lib_abstractions/platform_lib_time.h \
	../platform_lib_abstractions/platform_lib_macros.h

libgps_utils_so_la_c_sources = loc_log.cpp \
	loc_cfg.cpp \
	msg_q.c \
	linked_list.c \
	loc_target.cpp \
	../platform_lib_abstractions/elapsed_millis_since_boot.cpp

library_includedir = $(pkgincludedir)/utils

library_include_HEADERS = $(libgps_utils_so_la_h_sources)

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

libgps_utils_so_la_LIBADD = -lstdc++ -lcutils

#Create and Install libraries
lib_LTLIBRARIES = libgps_utils_so.la
