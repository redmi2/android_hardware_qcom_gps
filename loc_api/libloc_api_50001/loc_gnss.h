/* Copyright (c) 2016 The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LE_INCLUDE_HARDWARE_GNSS_H
#define LE_INCLUDE_HARDWARE_GNSS_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define MAX_CONSTELLATIONS  6

typedef uint32_t     location_flags_t;

#define LOCATION_IS_LATITUDE_VALID      (1 << 0)
#define LOCATION_IS_LONGITUDE_VALID     (1 << 1)
#define LOCATION_IS_ALTITUDE_VALID      (1 << 2)
#define LOCATION_IS_ACCURACY_VALID      (1 << 3)
#define LOCATION_IS_SPEED_VALID         (1 << 4)
#define LOCATION_IS_SPEED_UNC_VALID     (1 << 5)
#define LOCATION_IS_HEADING_VALID       (1 << 6)
#define LOCATION_IS_HEADING_UNC_VALID   (1 << 7)
#define LOCATION_IS_VEL_EAST_VALID      (1 << 8)
#define LOCATION_IS_VEL_EAST_UNC_VALID  (1 << 9)
#define LOCATION_IS_VEL_NORTH_VALID     (1 << 10)
#define LOCATION_IS_VEL_NORTH_UNC_VALID (1 << 11)
#define LOCATION_IS_VEL_UP_VALID        (1 << 12)
#define LOCATION_IS_VEL_UP_UNC_VALID    (1 << 13)
#define LOCATION_IS_PDOP_VALID          (1 << 14)
#define LOCATION_IS_HDOP_VALID          (1 << 15)
#define LOCATION_IS_VDOP_VALID          (1 << 16)
#define LOCATION_IS_USED_IN_FIX_VALID   (1 << 17)
#define LOCATION_IS_TIMESTAMP_VALID     (1 << 18)

typedef struct {
    size_t              size;
    double              latitude_deg;
    double              longitude_deg;
    double              altitude_m;
    double              accuracy_m;  /* expected accuracy in meters */
    double              speed_mps;
    double              speed_unc_mps;
    double              heading_deg;
    double              heading_unc_deg;
    double              vel_east_mps;
    double              vel_east_unc_mps;
    double              vel_north_mps;
    double              vel_north_unc_mps;
    double              vel_up_mps;
    double              vel_up_unc_mps;
    double              pdop;
    double              hdop;
    double              vdop;
    uint64_t            used_in_fix[MAX_CONSTELLATIONS]; /* for each constellation the used_in_fix has one bit per sv
                                                            (1 if the svid was used in computing the fix, 0 otherwise) */
    uint64_t            timestamp_ms; /* timestamp of the fix in ms */
    location_flags_t    location_flags;
} Location;

typedef enum {
   GNSS_CONSTELLATION_UNKNOWN = 0,
   GNSS_CONSTELLATION_GPS = 1,
   GNSS_CONSTELLATION_SBAS = 2,
   GNSS_CONSTELLATION_GLONASS = 3,
   GNSS_CONSTELLATION_QZSS = 4,
   GNSS_CONSTELLATION_BEIDOU = 5,
   GNSS_CONSTELLATION_GALILEO = 6
} GnssConstellation_t;

typedef uint16_t    svid_t;
typedef uint8_t     sv_info_flags_t;

#define SV_INFO_IS_CN0_VALID        (1 << 0)
#define SV_INFO_IS_ELEVATION_VALID  (1 << 1)
#define SV_INFO_IS_AZIMUTH_VALID    (1 << 2)
#define SV_INFO_HAS_EPHEMERIS       (1 << 3)
#define SV_INFO_HAS_ALMANAC         (1 << 4)
#define SV_INFO_IS_BEING_TRACKED    (1 << 5)

typedef struct {
    size_t                  size;
    svid_t                  svid;
    GnssConstellation_t     constellation;
    float                   cN0_dbHz;
    float                   elevation_deg;
    float                   azimuth_deg;
    sv_info_flags_t         sv_info_flags;
} SV_Info;

typedef struct {
    void(*location_cb)(const Location* location);
    void(*sv_info_cb)(const SV_Info* sv_info, uint32_t num_of_svs);
    void(*get_XTRA_data_cb)();
} LocationCallbacks;

typedef enum {
    ERROR_NO_ERROR = 0,
    ERROR_SESSION_IN_PROGRESS = -1,
    ERROR_CANNOT_INITIALIZE = -2,
} ErrorCodes_t;

int gnss_start(LocationCallbacks* loc_cbs, uint32_t fix_rate_ms);
int gnss_stop();
int gnss_inject_XTRA_data(const char* XTRA_data, uint32_t XTRA_length);
int gnss_delete_aiding_data(uint32_t delete_aiding_mask);


#endif /* LE_INCLUDE_HARDWARE_GNSS_H */

