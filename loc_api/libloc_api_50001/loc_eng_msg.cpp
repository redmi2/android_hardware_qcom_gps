/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
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
 *
 */
#include <stdlib.h>
#include <fcntl.h>
#include "loc_eng_msg.h"
#include "loc_eng_dmn_conn_glue_msg.h"
#include <msg_q.h>

#ifdef _ANDROID_

#define LOC_ENG_MSG_REQ_Q_PATH "/data/misc/gpsone_d/loc_eng_msg_req_q"

#else

#define LOC_ENG_MSG_REQ_Q_PATH "/tmp/loc_eng_msg_req_q"

#endif
static void free_msg(void* msg)
{
    free(msg);
}

int loc_eng_msgget(void ** p_req_msgq)
{
    return (eMSG_Q_SUCCESS == msg_q_init(p_req_msgq) ? 0 : -1);
}

int loc_eng_msgremove(void** p_req_msgq)
{
    return (eMSG_Q_SUCCESS == msg_q_destroy(p_req_msgq) ? 0 : -1);
}

int loc_eng_msgsnd(void* msgqid, void * msgp, unsigned int size)
{
    int result = 0;
    void* msg = malloc(size);
    memcpy(msg, msgp, size);
    if (eMSG_Q_SUCCESS != msg_q_snd(msgqid, msg, free)) {
        free(msg);
        result = -1;
    }

    return result;
}

int loc_eng_msgrcv(void* msgqid, void **msgp)
{
    int result = 0;

    *msgp = NULL;
    if (eMSG_Q_SUCCESS != msg_q_rcv(msgqid, msgp)) {
        if (NULL != *msgp) {
            free(*msgp);
            *msgp = NULL;
        }
        result = -1;
    }

    return result;
}

int loc_eng_msgflush(void* msgqid)
{
    return (eMSG_Q_SUCCESS == msg_q_flush(msgqid) ? 0 : -1);
}

int loc_eng_msgunblock(void* msgqid)
{
    return (eMSG_Q_SUCCESS == msg_q_unblock(msgqid) ? 0 : -1);
}
