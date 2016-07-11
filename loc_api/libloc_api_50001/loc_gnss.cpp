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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <loc_gnss.h>
#include <pthread.h>
#include <loc_eng.h>
#include "loc_api_v02_client.h"

bool isInitialized = false;
bool isSessionInProgress = false;
static LocationCallbacks* my_loc_cbs = NULL;
static loc_eng_data_s_type loc_gnss_data;
static uint32_t    num_of_svs;

static void* loc_gnss_parse_loc_ext(void* locExt)
{
    qmiLocEventPositionReportIndMsgT_v02* lE = (qmiLocEventPositionReportIndMsgT_v02*)locExt;
    Location* pGnssLocation;

    if (NULL == locExt)
    {
        LOC_LOGE("locExt pointer is NULL, returning!");
        return NULL;
    }

    pGnssLocation = (Location*)malloc(sizeof(Location));
    memset(pGnssLocation, 0, sizeof(Location));
    pGnssLocation->size = sizeof(Location);

    if ((true == lE->latitude_valid) &&
        (true == lE->longitude_valid) &&
        (lE->latitude != 0 || lE->longitude != 0))
    {
        if (true == lE->latitude_valid)
        {
            pGnssLocation->latitude_deg = lE->latitude;
            pGnssLocation->location_flags |= LOCATION_IS_LATITUDE_VALID;
        }
        if (true == lE->longitude_valid)
        {
            pGnssLocation->longitude_deg = lE->longitude;
            pGnssLocation->location_flags |= LOCATION_IS_LONGITUDE_VALID;
        }
        if (true == lE->altitudeWrtEllipsoid_valid)
        {
            pGnssLocation->altitude_m = lE->altitudeWrtEllipsoid;
            pGnssLocation->location_flags |= LOCATION_IS_ALTITUDE_VALID;
        }
        if (true == lE->horUncCircular_valid)
        {
            pGnssLocation->accuracy_m = lE->horUncCircular;
            pGnssLocation->location_flags |= LOCATION_IS_ACCURACY_VALID;
        }
        else if (true == lE->horUncEllipseSemiMinor_valid &&
            true == lE->horUncEllipseSemiMajor_valid)
        {
            pGnssLocation->accuracy_m =
                sqrt((lE->horUncEllipseSemiMinor *
                    lE->horUncEllipseSemiMinor) +
                    (lE->horUncEllipseSemiMajor *
                        lE->horUncEllipseSemiMajor));
            pGnssLocation->location_flags |= LOCATION_IS_ACCURACY_VALID;
        }
        if (true == lE->speedHorizontal_valid)
        {
            pGnssLocation->speed_mps = lE->speedHorizontal;
            pGnssLocation->location_flags |= LOCATION_IS_SPEED_VALID;
        }
        if (true == lE->speedUnc_valid)
        {
            pGnssLocation->speed_unc_mps = lE->speedUnc;
            pGnssLocation->location_flags |= LOCATION_IS_SPEED_UNC_VALID;
        }
        if (true == lE->heading_valid)
        {
            pGnssLocation->heading_deg = lE->heading;
            pGnssLocation->location_flags |= LOCATION_IS_HEADING_VALID;
        }
        if (true == lE->headingUnc_valid)
        {
            pGnssLocation->heading_unc_deg = lE->headingUnc;
            pGnssLocation->location_flags |= LOCATION_IS_HEADING_UNC_VALID;
        }
        if (true == lE->velEnu_valid)
        {
            pGnssLocation->vel_east_mps = lE->velEnu[0];
            pGnssLocation->vel_north_mps = lE->velEnu[1];
            pGnssLocation->vel_up_mps = lE->velEnu[2];
            pGnssLocation->location_flags |= LOCATION_IS_VEL_EAST_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_VEL_NORTH_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_VEL_UP_VALID;
        }
        if (true == lE->velUncEnu_valid)
        {
            pGnssLocation->vel_east_unc_mps = lE->velUncEnu[0];
            pGnssLocation->vel_north_unc_mps = lE->velUncEnu[1];
            pGnssLocation->vel_up_unc_mps = lE->velUncEnu[2];
            pGnssLocation->location_flags |= LOCATION_IS_VEL_EAST_UNC_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_VEL_NORTH_UNC_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_VEL_UP_UNC_VALID;
        }
        if (true == lE->DOP_valid)
        {
            pGnssLocation->pdop = lE->DOP.PDOP;
            pGnssLocation->hdop = lE->DOP.HDOP;
            pGnssLocation->vdop = lE->DOP.VDOP;
            pGnssLocation->location_flags |= LOCATION_IS_PDOP_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_HDOP_VALID;
            pGnssLocation->location_flags |= LOCATION_IS_VDOP_VALID;
        }
        if (true == lE->gnssSvUsedList_valid)
        {
            int svId;

            for (int i = 0; i < lE->gnssSvUsedList_len; i++)
            {
                svId = lE->gnssSvUsedList[i];
//                LOC_LOGD("lE->gnssSvUsedList[%d]=%d", i, svId);
                if (svId >= 1 && svId <= 32) // GPS
                {
                    pGnssLocation->used_in_fix[GNSS_CONSTELLATION_GPS - 1] |= (1 << (svId - 1));
                }
                else if (svId >= 65 && svId <= 96) // GLONASS
                {
                    pGnssLocation->used_in_fix[GNSS_CONSTELLATION_GLONASS - 1] |= (1 << (svId - 65));
                }
                else if (svId >= 193 && svId <= 197) // QZSS
                {
                    pGnssLocation->used_in_fix[GNSS_CONSTELLATION_QZSS - 1] |= (1 << (svId - 193));
                }
                else if (svId >= 201 && svId <= 237) // BEIDOU
                {
                    pGnssLocation->used_in_fix[GNSS_CONSTELLATION_BEIDOU - 1] |= (1 << (svId - 201));
                }
                else if (svId >= 301 && svId <= 336) // GALILEO
                {
                    pGnssLocation->used_in_fix[GNSS_CONSTELLATION_GALILEO - 1] |= (1 << (svId - 301));
                }
            }
            pGnssLocation->location_flags |= LOCATION_IS_USED_IN_FIX_VALID;
        }

        if (true == lE->timestampUtc_valid)
        {
            pGnssLocation->timestamp_ms = lE->timestampUtc;
            pGnssLocation->location_flags |= LOCATION_IS_TIMESTAMP_VALID;
        }
    }
    return (void*)pGnssLocation;
}

static void* loc_gnss_parse_sv_ext(void* locExt)
{
    qmiLocEventGnssSvInfoIndMsgT_v02* lE = (qmiLocEventGnssSvInfoIndMsgT_v02*)locExt;
    qmiLocSvInfoStructT_v02 *sv_info_ptr;
    SV_Info* pGnssSvInfo;
    SV_Info* pGnssSvInfoSat;
    int i;
    int slen;

    if (NULL == locExt)
    {
        LOC_LOGE("locExt pointer is NULL, returning!");
        return NULL;
    }

    if (false == lE->svList_valid || 0 == lE->svList_len)
    {
        LOC_LOGE("List is invalid or empty, returning!");
        return NULL;
    }
    slen = sizeof(SV_Info);
    num_of_svs = lE->svList_len;
    pGnssSvInfo = (SV_Info*)malloc(num_of_svs*slen);
    memset(pGnssSvInfo, 0, num_of_svs*slen);

    for (i = 0, pGnssSvInfoSat = pGnssSvInfo; i < num_of_svs; i++, pGnssSvInfoSat++)
    {
        sv_info_ptr = &(lE->svList[i]);

        pGnssSvInfoSat->size = sizeof(SV_Info);

        pGnssSvInfoSat->svid = sv_info_ptr->gnssSvId;

        if ((sv_info_ptr->validMask & QMI_LOC_SV_INFO_MASK_VALID_SYSTEM_V02) &&
            (sv_info_ptr->validMask & QMI_LOC_SV_INFO_MASK_VALID_GNSS_SVID_V02)
            && (sv_info_ptr->gnssSvId != 0))
        {
            switch (sv_info_ptr->system)
            {
               case eQMI_LOC_SV_SYSTEM_GPS_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_GPS;
                    break;

                case eQMI_LOC_SV_SYSTEM_GALILEO_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_GALILEO;
                    break;

                case eQMI_LOC_SV_SYSTEM_SBAS_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_SBAS;
                    break;

                case eQMI_LOC_SV_SYSTEM_GLONASS_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_GLONASS;
                    break;

                case eQMI_LOC_SV_SYSTEM_BDS_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_BEIDOU;
                    break;

                case eQMI_LOC_SV_SYSTEM_QZSS_V02:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_QZSS;
                    break;

                default:
                    pGnssSvInfoSat->constellation = GNSS_CONSTELLATION_UNKNOWN;
                    break;
            }
            if (sv_info_ptr->validMask & QMI_LOC_SV_INFO_MASK_VALID_SNR_V02)
            {
                pGnssSvInfoSat->cN0_dbHz = sv_info_ptr->snr;
                pGnssSvInfoSat->sv_info_flags |= SV_INFO_IS_CN0_VALID;
            }
            if (sv_info_ptr->validMask & QMI_LOC_SV_INFO_MASK_VALID_ELEVATION_V02)
            {
                pGnssSvInfoSat->elevation_deg = sv_info_ptr->elevation;
                pGnssSvInfoSat->sv_info_flags |= SV_INFO_IS_ELEVATION_VALID;
            }
            if (sv_info_ptr->validMask & QMI_LOC_SV_INFO_MASK_VALID_AZIMUTH_V02)
            {
                pGnssSvInfoSat->azimuth_deg = sv_info_ptr->azimuth;
                pGnssSvInfoSat->sv_info_flags |= SV_INFO_IS_AZIMUTH_VALID;
            }
            if (sv_info_ptr->validMask &
                QMI_LOC_SV_INFO_MASK_VALID_SVINFO_MASK_V02)
            {
                if (sv_info_ptr->svInfoMask &
                    QMI_LOC_SVINFO_MASK_HAS_EPHEMERIS_V02)
                {
                    pGnssSvInfoSat->sv_info_flags |= SV_INFO_HAS_EPHEMERIS;
                }
                if (sv_info_ptr->svInfoMask &
                    QMI_LOC_SVINFO_MASK_HAS_ALMANAC_V02)
                {
                    pGnssSvInfoSat->sv_info_flags |= SV_INFO_HAS_ALMANAC;
                }
            }
            if ((sv_info_ptr->validMask &
                QMI_LOC_SV_INFO_MASK_VALID_PROCESS_STATUS_V02)
                &&
                (sv_info_ptr->svStatus == eQMI_LOC_SV_STATUS_TRACK_V02))
            {
                pGnssSvInfoSat->sv_info_flags |= SV_INFO_IS_BEING_TRACKED;
            }
        }
    }

    LOC_LOGD("exit loc_gnss_parse_loc_ext pGnssSvInfo=%p", pGnssSvInfo);
    return (void*)pGnssSvInfo;
}

void loc_gnss_location_cb(GpsLocation* location, void* locExt)
{
    Location* pGnssLocation;

    if (NULL == location)
    {
        LOC_LOGE("location pointer is NULL, returning!");
        return;
    }
    if (NULL == locExt)
    {
        LOC_LOGE("locExt pointer is NULL, returning!");
        return;
    }
    pGnssLocation = (Location*)locExt;

    if (NULL != my_loc_cbs && true == isSessionInProgress)
    {
        my_loc_cbs->location_cb(pGnssLocation);
    }
    free(pGnssLocation);
}

void loc_gnss_sv_status_cb(GpsSvStatus *sv_info, void* svExt)
{
    char* temp;
    SV_Info* pGnssSvInfo;
    int slen;

    if (NULL == sv_info)
    {
        LOC_LOGE("sv_info pointer is NULL, returning!");
        return;
    }
    if (NULL == svExt)
    {
        LOC_LOGE("svExt pointer is NULL, returning!");
        return;
    }

    slen = sizeof(SV_Info);

    pGnssSvInfo = (SV_Info*)svExt;

    if (NULL != my_loc_cbs && true == isSessionInProgress)
    {
        my_loc_cbs->sv_info_cb(pGnssSvInfo, num_of_svs);
    }
    free(pGnssSvInfo);
}

typedef void(*ThreadStart) (void *);
struct tcreatorData {
    ThreadStart pfnThreadStart;
    void* arg;
};

void *my_thread_fn(void *tcd)
{
    tcreatorData* local_tcd = (tcreatorData*)tcd;
    LOC_LOGD("my_thread_fn local_tcd=0x04X", local_tcd);
    if (NULL != local_tcd) {
        LOC_LOGD("my_thread_fn calling deferred thread");
        local_tcd->pfnThreadStart(local_tcd->arg);
        free(local_tcd);
    }

    return NULL;
}

pthread_t test_gps_create_thread_cb(const char *name, void(*start) (void *),
    void *arg)
{
    pthread_t thread_id = -1;

    tcreatorData* tcd = (tcreatorData*)malloc(sizeof(*tcd));

    if (NULL != tcd) {
        tcd->pfnThreadStart = start;
        tcd->arg = arg;

        if (0 > pthread_create(&thread_id, NULL, my_thread_fn, (void*)tcd)) {
            LOC_LOGE("pthread_create() fail!");
            free(tcd);
        }
        else {
            LOC_LOGD("Thread created successfully");
        }
    }


    return thread_id;
}

static int loc_gnss_init(LocationCallbacks* loc_cbs)
{
    int retVal = -1;
    LOC_API_ADAPTER_EVENT_MASK_T event;
    LocCallbacks clientCallbacks = {
        loc_gnss_location_cb, /* location_cb */
        NULL, /* status_cb */
        loc_gnss_sv_status_cb, /* sv_status_cb */
        NULL, /* nmea_cb */
        NULL, /* set_capabilities_cb */
        NULL, /* acquire_wakelock_cb */
        NULL, /* release_wakelock_cb */
        test_gps_create_thread_cb, /* create_thread_cb */
        loc_gnss_parse_loc_ext, /* location_ext_parser */
        loc_gnss_parse_sv_ext, /* sv_ext_parser */
        NULL, /* request_utc_time_cb */
    };
    GpsXtraCallbacks myGpsXtraCallbacks = {
        loc_cbs->get_XTRA_data_cb,
        test_gps_create_thread_cb,
    };

    ENTRY_LOG();

    event = LOC_API_ADAPTER_BIT_PARSED_POSITION_REPORT |
        LOC_API_ADAPTER_BIT_SATELLITE_REPORT |
        LOC_API_ADAPTER_BIT_LOCATION_SERVER_REQUEST |
        LOC_API_ADAPTER_BIT_STATUS_REPORT;

    retVal = loc_eng_init(loc_gnss_data, &clientCallbacks, event, NULL);
    if (retVal) {
        LOC_LOGE("loc_eng_init() fail!");
        goto err;
    }
    LOC_LOGD("loc_eng_init() success!");

    retVal = loc_eng_xtra_init(loc_gnss_data, &myGpsXtraCallbacks);
    if (retVal) {
        LOC_LOGE("loc_eng_xtra_init() fail!");
        goto err;
    }
    LOC_LOGD("loc_eng_xtra_init() success!");

err:
    EXIT_LOG(%d, retVal);
    return retVal;
}

int gnss_start(LocationCallbacks* loc_cbs, uint32_t fix_rate_ms)
{
    loc_eng_read_config();

    LOC_LOGD("enter gnss_start loc_cbs=%p fix_rate_ms=%d", loc_cbs, fix_rate_ms);

    if (true == isSessionInProgress)
    {
        return ERROR_SESSION_IN_PROGRESS;
    }
    isSessionInProgress = true;
    if (false == isInitialized)
    {
        int retVal;
        retVal = loc_gnss_init(loc_cbs);
        if (0 != retVal)
        {
            return ERROR_CANNOT_INITIALIZE;
        }

        isInitialized = true;
    }

    my_loc_cbs = loc_cbs;

    LocPosMode params(LOC_POSITION_MODE_STANDALONE,
                    GPS_POSITION_RECURRENCE_PERIODIC,
                    fix_rate_ms,
                    100,
                    fix_rate_ms,
                    NULL,
                    NULL);

    loc_eng_set_position_mode(loc_gnss_data, params);
    loc_eng_start(loc_gnss_data);

    return 0;
}

int gnss_stop()
{
    LOC_LOGD("enter gnss_stop");
    if (true == isSessionInProgress)
    {
        isSessionInProgress = false;

        LOC_LOGD("gnss_stop calling loc_eng_stop");
        loc_eng_stop(loc_gnss_data);
    }
    else
    {
        LOC_LOGD("gnss_stop called when session is not in progress, ignored");
    }
    return 0;
}

int gnss_inject_XTRA_data(const char* XTRA_data, uint32_t XTRA_length)
{
    loc_eng_xtra_inject_data(loc_gnss_data, (char*)XTRA_data, XTRA_length);
    return 0;
}

int gnss_delete_aiding_data(uint32_t delete_aiding_mask)
{
    if (false == isSessionInProgress)
    {
        LOC_LOGD("gnss_delete_aiding_data calling loc_eng_delete_aiding_data");
        loc_eng_delete_aiding_data(loc_gnss_data, (uint16_t)delete_aiding_mask);
    }
    else
    {
        LOC_LOGD("gnss_stop called when session is in progress, ignored");
    }
    return 0;
}


