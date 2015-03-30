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

#include "agps.h"
#include <assert.h>
#include <fdio/task.h>
#include <hardware/gps.h>
#include <netinet/in.h>
#include <pdu/pdubuf.h>
#include "gps.h"
#include "log.h"
#include "pdu.h"
#include "thread.h"

#ifndef QCOM_FEATURE_IPV6
#ifdef AGPS_TYPE_INVALID
#define QCOM_FEATURE_IPV6 1
#endif
#endif

enum {
  /* commands/responses */
  OPCODE_DATA_CONN_OPEN = 0x01,
  OPCODE_DATA_CONN_CLOSED = 0x02,
  OPCODE_DATA_CONN_FAILED = 0x03,
  OPCODE_SET_SERVER = 0x04,
  OPCODE_DATA_CONN_OPEN_WITH_APN_IP_TYPE = 0x05,
  /* notifications */
  OPCODE_STATUS_NTF = 0x81
};

static void (*send_pdu)(struct pdu_wbuf* wbuf);
static const AGpsInterface* agps_interface;

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
append_AGpsType(struct pdu* pdu, const AGpsType* type)
{
  return append_to_pdu(pdu, "S", (uint16_t)*type);
}

static long
append_AGpsStatusValue(struct pdu* pdu, const AGpsStatusValue* value)
{
  return append_to_pdu(pdu, "S", (uint16_t)*value);
}

#if ANDROID_VERSION >= 22
static long
append_in_addr(struct pdu* pdu, const struct in_addr* addr)
{
  return append_to_pdu(pdu, "I", addr->s_addr);
}

static long
append_sockaddr_in(struct pdu* pdu, const struct sockaddr_in* addr)
{
  if (append_to_pdu(pdu, "SS", (uint16_t)addr->sin_family,
                               (uint16_t)addr->sin_port)) {
    return -1;
  }
  return append_in_addr(pdu, &addr->sin_addr);
}

static long
append_in6_addr(struct pdu* pdu, const struct in6_addr* addr)
{
  return append_to_pdu(pdu, "IIII", addr->s6_addr32[0],
                                    addr->s6_addr32[1],
                                    addr->s6_addr32[2],
                                    addr->s6_addr32[3]);
}

static long
append_sockaddr_in6(struct pdu* pdu, const struct sockaddr_in6* addr)
{
  if (append_to_pdu(pdu, "SSI", addr->sin6_family,
                                addr->sin6_port,
                                addr->sin6_flowinfo)) {
    return -1;
  }
  if (append_in6_addr(pdu, &addr->sin6_addr) < 0) {
    return -1;
  }
  return append_to_pdu(pdu, "I", &addr->sin6_scope_id);
}

static long
append_sockaddr_storage(struct pdu* pdu, const struct sockaddr_storage* addr)
{
  switch (addr->ss_family) {
    case AF_INET:
      return append_sockaddr_in(pdu, (const struct sockaddr_in*)addr);
    case AF_INET6:
      return append_sockaddr_in6(pdu, (const struct sockaddr_in6*)addr);
  }
  ALOGE("invalid address family %d", addr->ss_family);
  return -1;
}
#endif

static long
append_AGpsStatus(struct pdu* pdu, const AGpsStatus* status)
{
  uint8_t version;
  long off;

#if ANDROID_VERSION >= 22
  if (status->size == sizeof(AGpsStatus_v3)) {
    version = 3;
  } else
#endif
#if ANDROID_VERSION >= 21
  if (status->size == sizeof(AGpsStatus_v2)) {
    version = 2;
  } else if (status->size == sizeof(AGpsStatus_v1)) {
    version = 1;
  } else
#else
  if (status->size == sizeof(AGpsStatus)) {
    version = 1;
  } else
#endif
  {
    ALOGE("invalid size of AGpsStatus");
    return -1;
  }

  /* v1 and later */
  if ((append_to_pdu(pdu, "C", version) < 0) ||
      (append_AGpsType(pdu, &status->type) < 0)) {
    return -1;
  }
  off = append_AGpsStatusValue(pdu, &status->status);
  if ((off < 0) || (version < 2)) {
    return off;
  }

#if ANDROID_VERSION >= 21
  /* v2 and later */
  off = append_to_pdu(pdu, "I", status->ipaddr);
  if ((off < 0) || (version < 3)) {
    return off;
  }
#endif

#if ANDROID_VERSION >= 22
  /* v3 and later */
  off = append_sockaddr_storage(pdu, &status->addr);
  if (off < 0) {
    return off;
  }
#endif

  return off;
}

static long
read_AGpsType(const struct pdu* pdu, unsigned long offset, AGpsType* type)
{
  return read_pdu_at(pdu, offset, "S", (uint16_t*)type);
}

#if ANDROID_VERSION >= 21
static long
read_ApnIpType(const struct pdu* pdu, unsigned long offset, ApnIpType* type)
{
  return read_pdu_at(pdu, offset, "S", (uint16_t*)type);
}
#endif

/*
 * Notifications
 */

static void
status_cb(AGpsStatus* status)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(137, /* max size of AGpsStatus_v3 */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_AGPS, OPCODE_STATUS_NTF);
  if (append_AGpsStatus(&wbuf->buf.pdu, status) < 0) {
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
data_conn_open(const struct pdu* cmd)
{
  char* apn;
  struct pdu_wbuf* wbuf;
  int res;

  assert(agps_interface);
  assert(agps_interface->data_conn_open);

  apn = NULL;

  if (read_pdu_at(cmd, 0, "0", &apn) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
#ifdef QCOM_FEATURE_IPV6
  res = agps_interface->data_conn_open(AGPS_TYPE_SUPL, apn,
                                       AGPS_APN_BEARER_IPV4);
#else
  res = agps_interface->data_conn_open(apn);
#endif
  if (res < 0) {
    goto err_agps_interface_data_conn_open;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(apn);

  return ERROR_NONE;
err_agps_interface_data_conn_open:
  destroy_pdu_wbuf(wbuf);
  free(apn);
  return ERROR_FAIL;
}

static int
data_conn_closed(const struct pdu* cmd)
{
  struct pdu_wbuf* wbuf;
  int res;

  assert(agps_interface);
  assert(agps_interface->data_conn_closed);

  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
#ifdef QCOM_FEATURE_IPV6
  res = agps_interface->data_conn_closed(AGPS_TYPE_SUPL);
#else
  res = agps_interface->data_conn_closed();
#endif
  if (res < 0) {
    goto err_agps_interface_data_conn_closed;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_agps_interface_data_conn_closed:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
data_conn_failed(const struct pdu* cmd)
{
  struct pdu_wbuf* wbuf;
  int res;

  assert(agps_interface);
  assert(agps_interface->data_conn_failed);

  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
#ifdef QCOM_FEATURE_IPV6
  res = agps_interface->data_conn_failed(AGPS_TYPE_SUPL);
#else
  res = agps_interface->data_conn_failed();
#endif
  if (res < 0) {
    goto err_agps_interface_data_conn_failed;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_agps_interface_data_conn_failed:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
set_server(const struct pdu* cmd)
{
  long off;
  AGpsType type;
  uint16_t port;
  char* hostname;
  struct pdu_wbuf* wbuf;
  int res;

  assert(agps_interface);
  assert(agps_interface->data_conn_open);

  hostname = NULL;

  off = read_AGpsType(cmd, 0, &type);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "S0", &port, &hostname) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  res = agps_interface->set_server(type, hostname, port);
  if (res < 0) {
    goto err_agps_interface_set_server;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(hostname);

  return ERROR_NONE;
err_agps_interface_set_server:
  destroy_pdu_wbuf(wbuf);
  free(hostname);
  return ERROR_FAIL;
}

#if ANDROID_VERSION >= 21
static int
data_conn_open_with_apn_ip_type(const struct pdu* cmd)
{
  long off;
  ApnIpType apn_ip_type;
  char* apn;
  struct pdu_wbuf* wbuf;
  int res;

  assert(agps_interface);

  if (agps_interface->size < sizeof(AGpsInterface_v2)) {
    return ERROR_UNSUPPORTED;
  }
  assert(agps_interface->data_conn_open_with_apn_ip_type);

  apn = NULL;

  off = read_ApnIpType(cmd, 0, &apn_ip_type);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "0", &apn) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  res = agps_interface->data_conn_open_with_apn_ip_type(apn, apn_ip_type);
  if (res < 0) {
    goto err_agps_interface_data_conn_open_with_apn_ip_type;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  free(apn);

  return ERROR_NONE;
err_agps_interface_data_conn_open_with_apn_ip_type:
  destroy_pdu_wbuf(wbuf);
  free(apn);
  return ERROR_FAIL;
}
#endif

static int
agps_handler(const struct pdu* cmd)
{
  static int (* const handler[256])(const struct pdu*) = {
    [OPCODE_DATA_CONN_OPEN] = data_conn_open,
    [OPCODE_DATA_CONN_CLOSED] = data_conn_closed,
    [OPCODE_DATA_CONN_FAILED] = data_conn_failed,
    [OPCODE_SET_SERVER] = set_server,
#if ANDROID_VERSION >= 21
    [OPCODE_DATA_CONN_OPEN_WITH_APN_IP_TYPE] = data_conn_open_with_apn_ip_type
#endif
  };

  return handle_pdu_by_opcode(cmd, handler);
}

int
(*register_agps(void (*send_pdu_cb)(struct pdu_wbuf*)))(const struct pdu*)
{
  static AGpsCallbacks agps_callbacks = {
    .status_cb = status_cb,
    .create_thread_cb = create_thread_cb
  };

  if (agps_interface) {
    ALOGE("AGPS interface already set up");
    return NULL;
  }

  agps_interface = get_gps_extension(AGPS_INTERFACE);
  if (!agps_interface) {
    ALOGE("get_gps_extension(AGPS_INTERFACE) failed");
    return NULL;
  }

  assert(agps_interface->init);
  agps_interface->init(&agps_callbacks);

  send_pdu = send_pdu_cb;

  return agps_handler;
}

int
unregister_agps()
{
  assert(agps_interface);

  send_pdu = NULL;
  agps_interface = NULL;

  return 0;
}
