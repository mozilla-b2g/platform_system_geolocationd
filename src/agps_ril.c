/*
 * Copyright (C) 2015  Mozilla Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "agps_ril.h"
#include <assert.h>
#include <fdio/task.h>
#include <hardware/gps.h>
#include <pdu/pdubuf.h>
#include "gps.h"
#include "log.h"
#include "pdu.h"
#include "thread.h"

enum {
  /* commands/responses */
  OPCODE_SET_REF_LOCATION = 0x01,
  OPCODE_SET_SET_ID = 0x02,
  OPCODE_NI_MESSAGE = 0x03,
  OPCODE_UPDATE_NETWORK_STATE = 0x04,
  OPCODE_UPDATE_NETWORK_AVAILABILITY = 0x05,
  /* notifications */
  OPCODE_REQUEST_SET_ID_NTF = 0x81,
  OPCODE_REQUEST_REF_LOCATION_NTF = 0x82
};

static void (*send_pdu)(struct pdu_wbuf* wbuf);
static const AGpsRilInterface* agps_ril_interface;

static enum ioresult
send_ntf_pdu(void* data)
{
  /* send notification on I/O thread */
  if (!send_pdu) {
    ALOGE("send_pdu is NULL");
    return IO_OK;
  }
  send_pdu(data);
  return IO_OK;
}

/*
 * Protocol helpers
 */

static long
read_AGpsRefLocationCellID(const struct pdu* pdu, unsigned long offset,
                           AGpsRefLocationCellID* cellid)
{
  return read_pdu_at(pdu, offset, "SSSSI", &cellid->type,
                                           &cellid->mcc,
                                           &cellid->mnc,
                                           &cellid->lac,
                                           &cellid->cid);
}

static long
read_AGpsRefLocationMac(const struct pdu* pdu, unsigned long offset,
                        AGpsRefLocationMac* mac)
{
  return read_pdu_at(pdu, offset, "CCCCCC", mac->mac + 0,
                                            mac->mac + 1,
                                            mac->mac + 2,
                                            mac->mac + 3,
                                            mac->mac + 4,
                                            mac->mac + 5);
}

static long
read_AGpsRefLocation(const struct pdu* pdu, unsigned long offset,
                     AGpsRefLocation* refloc)
{
  long off;

  off = read_pdu_at(pdu, offset, "S", &refloc->type);
  if (off < 0) {
    return -1;
  }
  switch (refloc->type) {
    case AGPS_REF_LOCATION_TYPE_GSM_CELLID:
    case AGPS_REF_LOCATION_TYPE_UMTS_CELLID:
      return read_AGpsRefLocationCellID(pdu, offset, &refloc->u.cellID);
    case AGPS_REG_LOCATION_TYPE_MAC:
      return read_AGpsRefLocationMac(pdu, offset, &refloc->u.mac);
    default:
      ALOGE("invalid location type %d", refloc->type);
  }
  return -1;
}

static long
read_AGpsSetIDType(const struct pdu* pdu, unsigned long offset,
                   AGpsSetIDType* type)
{
  return read_pdu_at(pdu, offset, "S", (uint16_t*)type);
}

/*
 * Notifications
 */

static void
request_setid_cb(uint32_t flags)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(4, /* flags */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_AGPS_RIL, OPCODE_REQUEST_SET_ID_NTF);
  if (append_to_pdu(&wbuf->buf.pdu, "I", flags) < 0) {
    goto cleanup;
  }
  if (run_task(send_ntf_pdu, wbuf) < 0) {
    goto cleanup;
  }
  return;
cleanup:
  destroy_pdu_wbuf(wbuf);
}

static void
request_refloc_cb(uint32_t flags)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(4, /* flags */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_AGPS_RIL, OPCODE_REQUEST_REF_LOCATION_NTF);
  if (append_to_pdu(&wbuf->buf.pdu, "I", flags) < 0) {
    goto cleanup;
  }
  if (run_task(send_ntf_pdu, wbuf) < 0) {
    goto cleanup;
  }
  return;
cleanup:
  destroy_pdu_wbuf(wbuf);
}

static pthread_t
create_thread_cb(const char* name, void (*start)(void*), void* arg)
{
  return create_thread(name, start, arg);
}

/*
 * Commands/Responses
 */

static int
set_ref_location(const struct pdu* cmd)
{
  long off;
  AGpsRefLocation agps_reflocation;
  struct pdu_wbuf* wbuf;

  assert(agps_ril_interface);
  assert(agps_ril_interface->set_ref_location);

  off = read_AGpsRefLocation(cmd, 0, &agps_reflocation);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  agps_ril_interface->set_ref_location(&agps_reflocation,
                                       sizeof(agps_reflocation));

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
}

static int
set_set_id(const struct pdu* cmd)
{
  long off;
  char* setid;
  AGpsSetIDType type;
  struct pdu_wbuf* wbuf;

  assert(agps_ril_interface);
  assert(agps_ril_interface->set_set_id);

  setid = NULL;

  off = read_AGpsSetIDType(cmd, 0, &type);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "0", &setid) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    free(setid);
    return ERROR_NOMEM;
  }

  agps_ril_interface->set_set_id(type, setid);

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(setid);

  return ERROR_NONE;
}

static int
ni_message(const struct pdu* cmd)
{
  long off;
  uint16_t len;
  uint8_t* msg;
  struct pdu_wbuf* wbuf;

  assert(agps_ril_interface);
  assert(agps_ril_interface->ni_message);

  msg = NULL;

  off = read_pdu_at(cmd, 0, "S", &len);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "m", &msg, len) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    free(msg);
    return ERROR_NOMEM;
  }

  agps_ril_interface->ni_message(msg, len);

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(msg);

  return ERROR_NONE;
}

static int
update_network_state(const struct pdu* cmd)
{
  int32_t type;
  uint8_t connected, roaming;
  char* info;
  struct pdu_wbuf* wbuf;

  assert(agps_ril_interface);
  assert(agps_ril_interface->update_network_state);

  info = NULL;

  if (read_pdu_at(cmd, 0, "iCC0", &type, &connected, &roaming, &info) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    free(info);
    return ERROR_NOMEM;
  }

  agps_ril_interface->update_network_state(connected, type, roaming, info);

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(info);

  return ERROR_NONE;
}

static int
update_network_availability(const struct pdu* cmd)
{
  uint8_t available;
  char* apn;
  struct pdu_wbuf* wbuf;

  assert(agps_ril_interface);
  assert(agps_ril_interface->update_network_availability);

  apn = NULL;

  if (read_pdu_at(cmd, 0, "C0", &available, &apn) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    free(apn);
    return ERROR_NOMEM;
  }

  agps_ril_interface->update_network_availability(available, apn);

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(apn);

  return ERROR_NONE;
}

static int
agps_ril_handler(const struct pdu* cmd)
{
  static int (* const handler[256])(const struct pdu*) = {
    [OPCODE_SET_REF_LOCATION] = set_ref_location,
    [OPCODE_SET_SET_ID] = set_set_id,
    [OPCODE_NI_MESSAGE] = ni_message,
    [OPCODE_UPDATE_NETWORK_STATE] = update_network_state,
    [OPCODE_UPDATE_NETWORK_AVAILABILITY] = update_network_availability
  };

  return handle_pdu_by_opcode(cmd, handler);
}

int
(*register_agps_ril(void (*send_pdu_cb)(struct pdu_wbuf*)))(const struct pdu*)
{
  static AGpsRilCallbacks agps_ril_callbacks = {
    .request_setid = request_setid_cb,
    .request_refloc = request_refloc_cb,
    .create_thread_cb = create_thread_cb
  };

  if (agps_ril_interface) {
    ALOGE("AGPS-RIL interface already set up");
    return NULL;
  }

  agps_ril_interface = get_gps_extension(AGPS_RIL_INTERFACE);
  if (!agps_ril_interface) {
    ALOGE("get_gps_extension(AGPS_RIL_INTERFACE) failed");
    return NULL;
  }

  assert(agps_ril_interface->init);
  agps_ril_interface->init(&agps_ril_callbacks);

  send_pdu = send_pdu_cb;

  return agps_ril_handler;
}

int
unregister_agps_ril()
{
  assert(agps_ril_interface);

  send_pdu = NULL;
  agps_ril_interface = NULL;

  return 0;
}
