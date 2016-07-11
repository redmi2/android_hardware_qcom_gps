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

#include "loc_gnss.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define XTRA_FILE_NAME "xtra2.bin"

static Location*   pLocation;
static SV_Info*    pSvInfo;
static uint32_t    num_of_svs;

typedef enum
{
    ACTION_NONE,
    ACTION_QUIT,
    ACTION_LOCATION,
    ACTION_SV_INFO,
    ACTION_XTRA_REQUEST
} gnss_thread_action_e_type;

pthread_mutex_t gnss_thread_mutex;
pthread_cond_t gnss_thread_cond;

gnss_thread_action_e_type gnss_thread_action;

static void mutex_init()
{
    pthread_mutex_init(&gnss_thread_mutex, NULL);
    pthread_cond_init(&gnss_thread_cond, NULL);
}

static void mutex_destroy()
{
    pthread_mutex_destroy(&gnss_thread_mutex);
    pthread_cond_destroy(&gnss_thread_cond);
}

static int XTRA_helper()
{
    FILE* XTRA_file;
    char* XTRA_data;
    int XTRA_length;
    int retVal = ERROR_NO_ERROR;

    XTRA_file = fopen(XTRA_FILE_NAME, "rb");

    if (NULL == XTRA_file)
    {
        printf("XTRA file %s cannot be opened", XTRA_FILE_NAME);
        return ERROR_CANNOT_INITIALIZE;
    }

    fseek(XTRA_file, 0, SEEK_END);
    XTRA_length = ftell(XTRA_file);
    fseek(XTRA_file, 0, SEEK_SET);

    XTRA_data = (char*)malloc(XTRA_length*sizeof(char));

    fread(XTRA_data, sizeof(char), XTRA_length, XTRA_file);
    fclose(XTRA_file);

    printf("Injecting XTRA with length = %d\n", XTRA_length);

    retVal = gnss_inject_XTRA_data(XTRA_data, XTRA_length);
    free(XTRA_data);

    return retVal;
}

static void *gnss_thread(void *args)
{
    uint32_t sv;
    uint32_t constellation;
    const char* constellationString[] = { "Unknown", "GPS", "SBAS", "GLONASS", "QZSS", "BEIDOU", "GALILEO" };

    printf("Enter gnss_thread\n");

    pthread_mutex_lock(&gnss_thread_mutex);
    do
    {
        pthread_cond_wait(&gnss_thread_cond, &gnss_thread_mutex);
        printf("gnss thread unblocked, action = ");
        if (gnss_thread_action == ACTION_QUIT)
        {
            printf(" ACTION_QUIT\n");
            break;
        }
        switch (gnss_thread_action)
        {
            case ACTION_LOCATION:
                printf(" ACTION_LOCATION\n");
                if (pLocation != NULL)
                {
                    printf("Printing location components:\n");
                    if (0 == pLocation->location_flags)
                    {
                        printf("\tInvalid Location\n");
                    }
                    else
                    {
                        if (LOCATION_IS_TIMESTAMP_VALID == (pLocation->location_flags & LOCATION_IS_TIMESTAMP_VALID))
                        {
                            printf("\tTimestamp = %lld\n", pLocation->timestamp_ms);
                        }
                        else
                        {
                            printf("\tInvalid Timestamp\n");
                        }
                        if (LOCATION_IS_LATITUDE_VALID == (pLocation->location_flags & LOCATION_IS_LATITUDE_VALID))
                        {
                            printf("\tLatitude = %g\n", pLocation->latitude_deg);
                        }
                        else
                        {
                            printf("\tInvalid Latitude\n");
                        }
                        if (LOCATION_IS_LONGITUDE_VALID == (pLocation->location_flags & LOCATION_IS_LONGITUDE_VALID))
                        {
                            printf("\tLongitude = %g\n", pLocation->longitude_deg);
                        }
                        else
                        {
                            printf("\tInvalid Longitude\n");
                        }
                        if (LOCATION_IS_ALTITUDE_VALID == (pLocation->location_flags & LOCATION_IS_ALTITUDE_VALID))
                        {
                            printf("\tAltitude = %g\n", pLocation->altitude_m);
                        }
                        else
                        {
                            printf("\tInvalid Altitude\n");
                        }
                        if (LOCATION_IS_ACCURACY_VALID == (pLocation->location_flags & LOCATION_IS_ACCURACY_VALID))
                        {
                            printf("\tAccuracy = %g\n", pLocation->accuracy_m);
                        }
                        else
                        {
                            printf("\tInvalid Accuracy\n");
                        }
                        if (LOCATION_IS_SPEED_VALID == (pLocation->location_flags & LOCATION_IS_SPEED_VALID))
                        {
                            printf("\tSpeed = %g\n", pLocation->speed_mps);
                        }
                        else
                        {
                            printf("\tInvalid Speed\n");
                        }
                        if (LOCATION_IS_SPEED_UNC_VALID == (pLocation->location_flags & LOCATION_IS_SPEED_UNC_VALID))
                        {
                            printf("\tSpeed Unc = %g\n", pLocation->speed_unc_mps);
                        }
                        else
                        {
                            printf("\tInvalid Speed Unc\n");
                        }
                        if (LOCATION_IS_HEADING_VALID == (pLocation->location_flags & LOCATION_IS_HEADING_VALID))
                        {
                            printf("\tHeading = %g\n", pLocation->heading_deg);
                        }
                        else
                        {
                            printf("\tInvalid Heading\n");
                        }
                        if (LOCATION_IS_HEADING_UNC_VALID == (pLocation->location_flags & LOCATION_IS_HEADING_UNC_VALID))
                        {
                            printf("\tHeading Unc = %g\n", pLocation->heading_unc_deg);
                        }
                        else
                        {
                            printf("\tInvalid Heading Unc\n");
                        }
                        if (LOCATION_IS_VEL_EAST_VALID == (pLocation->location_flags & LOCATION_IS_VEL_EAST_VALID))
                        {
                            printf("\tVelocity East = %g\n", pLocation->vel_east_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity East\n");
                        }
                        if (LOCATION_IS_VEL_EAST_UNC_VALID == (pLocation->location_flags & LOCATION_IS_VEL_EAST_UNC_VALID))
                        {
                            printf("\tVelocity East Unc = %g\n", pLocation->vel_east_unc_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity East Unc\n");
                        }
                        if (LOCATION_IS_VEL_NORTH_VALID == (pLocation->location_flags & LOCATION_IS_VEL_NORTH_VALID))
                        {
                            printf("\tVelocity North = %g\n", pLocation->vel_north_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity North\n");
                        }
                        if (LOCATION_IS_VEL_NORTH_UNC_VALID == (pLocation->location_flags & LOCATION_IS_VEL_NORTH_UNC_VALID))
                        {
                            printf("\tVelocity North Unc= %g\n", pLocation->vel_north_unc_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity North Unc\n");
                        }
                        if (LOCATION_IS_VEL_UP_VALID == (pLocation->location_flags & LOCATION_IS_VEL_UP_VALID))
                        {
                            printf("\tVelocity Up = %g\n", pLocation->vel_up_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity Up\n");
                        }
                        if (LOCATION_IS_VEL_UP_UNC_VALID == (pLocation->location_flags & LOCATION_IS_VEL_UP_UNC_VALID))
                        {
                            printf("\tVelocity Up Unc = %g\n", pLocation->vel_up_unc_mps);
                        }
                        else
                        {
                            printf("\tInvalid Velocity Up Unc\n");
                        }
                        if (LOCATION_IS_PDOP_VALID == (pLocation->location_flags & LOCATION_IS_PDOP_VALID))
                        {
                            printf("\tPDOP = %g\n", pLocation->pdop);
                        }
                        else
                        {
                            printf("\tInvalid PDOP\n");
                        }
                        if (LOCATION_IS_HDOP_VALID == (pLocation->location_flags & LOCATION_IS_HDOP_VALID))
                        {
                            printf("\tHDOP = %g\n", pLocation->hdop);
                        }
                        else
                        {
                            printf("\tInvalid HDOP\n");
                        }
                        if (LOCATION_IS_VDOP_VALID == (pLocation->location_flags & LOCATION_IS_VDOP_VALID))
                        {
                            printf("\tVDOP = %g\n", pLocation->vdop);
                        }
                        else
                        {
                            printf("\tInvalid VDOP\n");
                        }
                        if (LOCATION_IS_USED_IN_FIX_VALID == (pLocation->location_flags & LOCATION_IS_USED_IN_FIX_VALID))
                        {
                            for (constellation = GNSS_CONSTELLATION_GPS; constellation <= GNSS_CONSTELLATION_GALILEO; constellation++)
                            {
                                if (pLocation->used_in_fix[constellation - 1] != 0)
                                {
                                    printf("\tSatellites used for %*s: 0x%08X\n", 13, constellationString[constellation], pLocation->used_in_fix[constellation]);
                                }
                            }
                        }
                        else
                        {
                            printf("\tInvalid Used In Fix\n");
                        }
                    }
                    free(pLocation);
                    pLocation = NULL;
                }
                else
                {
                    printf("pLocation is NULL!:\n");
                }
                break;

            case ACTION_SV_INFO:
                printf(" ACTION_SV_INFO\n");
                if (pSvInfo != NULL)
                {
                    SV_Info* pSvInfoSat;
                    printf("Printing sv_info components for %d satellites:\n", num_of_svs);

                    for (sv = 0, pSvInfoSat = pSvInfo; sv < num_of_svs; sv++, pSvInfoSat++)
                    {
                        printf("\tSV ID = %d\n", pSvInfoSat->svid);
                        printf("\tConstellation = %s\n", constellationString[pSvInfoSat->constellation]);
                        if (SV_INFO_IS_CN0_VALID == (pSvInfoSat->sv_info_flags & SV_INFO_IS_CN0_VALID))
                        {
                            printf("\tCN0 = %f\n", pSvInfoSat->cN0_dbHz);
                        }
                        else
                        {
                            printf("\tInvalid CN0\n");
                        }
                        if (SV_INFO_IS_ELEVATION_VALID == (pSvInfoSat->sv_info_flags & SV_INFO_IS_ELEVATION_VALID))
                        {
                            printf("\tElevation = %f\n", pSvInfoSat->elevation_deg);
                        }
                        else
                        {
                            printf("\tInvalid Elevation\n");
                        }
                        if (SV_INFO_IS_AZIMUTH_VALID == (pSvInfoSat->sv_info_flags & SV_INFO_IS_AZIMUTH_VALID))
                        {
                            printf("\tAzimuth = %f\n", pSvInfoSat->azimuth_deg);
                        }
                        else
                        {
                            printf("\tInvalid Azimuth\n");
                        }
                        if (SV_INFO_HAS_EPHEMERIS == (pSvInfoSat->sv_info_flags & SV_INFO_HAS_EPHEMERIS))
                        {
                            printf("\tSatellite has ephemeris information\n");
                        }
                        else
                        {
                            printf("\tSatellite does not have ephemeris information\n");
                        }
                        if (SV_INFO_HAS_ALMANAC == (pSvInfoSat->sv_info_flags & SV_INFO_HAS_ALMANAC))
                        {
                            printf("\tSatellite has almanac information\n");
                        }
                        else
                        {
                            printf("\tSatellite does not have almanac information\n");
                        }
                        if (SV_INFO_IS_BEING_TRACKED == (pSvInfoSat->sv_info_flags & SV_INFO_IS_BEING_TRACKED))
                        {
                            printf("\tSatellite is being tracked\n");
                        }
                        else
                        {
                            printf("\tSatellite is not being tracked\n");
                        }
                    }
                    free(pSvInfo);
                    pSvInfo = NULL;
                }
                else
                {
                    printf("pSvInfo is NULL!:\n");
                }
                break;

            case ACTION_XTRA_REQUEST:
                printf(" ACTION_XTRA_REQUEST\n");
                XTRA_helper();
                break;

            default:
                break;
        }
        gnss_thread_action = ACTION_NONE;
    } while (1);
    pthread_mutex_unlock(&gnss_thread_mutex);

    printf("Exit gnss_thread\n");
    return NULL;
}

void location_callback(const Location* location)
{
    pLocation = (Location*)malloc(sizeof(Location));
    memcpy(pLocation, location, sizeof(Location));
    pthread_mutex_lock(&gnss_thread_mutex);
    gnss_thread_action = ACTION_LOCATION;
    pthread_cond_signal(&gnss_thread_cond);
    pthread_mutex_unlock(&gnss_thread_mutex);
}

static void sv_info_callback(const SV_Info* sv_info, uint32_t rcv_num_of_svs)
{
    num_of_svs = rcv_num_of_svs;
    pSvInfo = (SV_Info*)malloc(num_of_svs*sizeof(SV_Info));
    memcpy(pSvInfo, sv_info, num_of_svs*sizeof(SV_Info));
    pthread_mutex_lock(&gnss_thread_mutex);
    gnss_thread_action = ACTION_SV_INFO;
    pthread_cond_signal(&gnss_thread_cond);
    pthread_mutex_unlock(&gnss_thread_mutex);
}

void get_XTRA_data_callback()
{
    pthread_mutex_lock(&gnss_thread_mutex);
    gnss_thread_action = ACTION_XTRA_REQUEST;
    pthread_cond_signal(&gnss_thread_cond);
    pthread_mutex_unlock(&gnss_thread_mutex);
}

int main (int argc, char *argv[])
{
    char buf[16], *p, *r;
    bool exit_loop = false;
    LocationCallbacks location_callbacks;
    uint32_t fix_rate_ms = 0;
    int ret_status = -1;
    pthread_t tid;
    void *ignored;

    pLocation = NULL;
    pSvInfo = NULL;
    // start app thread to process received data
    mutex_init();
    pthread_create(&tid, NULL, gnss_thread, NULL);

    while(!exit_loop)
    {
        printf("\n\n");
        printf ("1: Start GNSS session\n"
                "2: Stop GNSS session\n"
                "3: Inject XTRA data (%s)\n"
                "4: Delete aiding data\n"
                "q: quit\n"
                "\nEnter Command:", XTRA_FILE_NAME);
        fflush (stdout);
        p = fgets (buf, 16, stdin);

        switch(p[0])
        {
            case '1':
                printf("Please choose the rate (min. 100 ms) ");
                fflush(stdout);
                r = fgets(buf, 16, stdin);
                fix_rate_ms = atoi(r);

                if (fix_rate_ms < 100)
                {
                    printf("Rate must be at least 100ms!\n");
                    break;
                }
                location_callbacks.location_cb = location_callback;
                location_callbacks.sv_info_cb = sv_info_callback;
                location_callbacks.get_XTRA_data_cb = get_XTRA_data_callback;
                ret_status = gnss_start(&location_callbacks, fix_rate_ms);
                if (ERROR_NO_ERROR == ret_status)
                {
                    printf("GNSS start returned success!\n");
                }
                else
                {
                    printf("GNSS start returned ERROR!\n");
                }
                break;

            case '2':
                ret_status = gnss_stop();
                if (ERROR_NO_ERROR == ret_status)
                {
                    printf("GNSS stop returned success!\n");
                }
                else
                {
                    printf("GNSS stop returned ERROR!\n");
                }
                break;

            case '3':
                ret_status = XTRA_helper();
                if (ERROR_NO_ERROR == ret_status)
                {
                    printf("GNSS inject XTRA returned success!\n");
                }
                else
                {
                    printf("GNSS inject XTRA returned ERROR!\n");
            }
                break;

            case '4':
                ret_status = gnss_delete_aiding_data(0xFFFFFFFF);
                if (ERROR_NO_ERROR == ret_status)
                {
                    printf("GNSS delete aiding data returned success!\n");
                }
                else
                {
                    printf("GNSS delete aiding data returned ERROR!\n");
                }
                break;

            case 'q':
                printf("Bye bye!\n");
                exit_loop = true;
                break;

            default:
                printf("\ninvalid command\n");
        }
    }

    pthread_mutex_lock(&gnss_thread_mutex);
    gnss_thread_action = ACTION_QUIT;
    pthread_cond_signal(&gnss_thread_cond);
    pthread_mutex_unlock(&gnss_thread_mutex);

    pthread_join(tid, &ignored);
    mutex_destroy();
}
