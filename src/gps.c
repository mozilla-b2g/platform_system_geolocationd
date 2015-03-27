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

#include "gps.h"
#include <assert.h>
#include <fdio/task.h>
#include <hardware/gps.h>
#include <hardware_legacy/power.h>
#include <pdu/pdubuf.h>
#include "log.h"
#include "pdu.h"
#include "thread.h"
#include "wakelock.h"

enum {
  /* commands/responses */
  OPCODE_START = 0x01,
  OPCODE_STOP = 0x02,
  OPCODE_INJECT_TIME = 0x03,
  OPCODE_INJECT_LOCATION = 0x04,
  OPCODE_DELETE_AIDING_DATA = 0x05,
  OPCODE_SET_POSITION_MODE = 0x06,
  /* notifications */
  OPCODE_LOCATION_NTF = 0x81,
  OPCODE_STATUS_NTF = 0x82,
  OPCODE_SV_STATUS_NTF = 0x83,
  OPCODE_NMEA_NTF = 0x84,
  OPCODE_SET_CAPABILITIES_NTF = 0x85,
  OPCODE_REQUEST_UTF_TIME_NTF = 0x86
};

static void (*send_pdu)(struct pdu_wbuf* wbuf);
static struct gps_device_t* gps_device;
static const GpsInterface* gps_interface;

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
append_GpsUtcTime(struct pdu* pdu, const GpsUtcTime* time)
{
  return append_to_pdu(pdu, "l", (int64_t)*time);
}

static long
append_GpsLocation(struct pdu* pdu, const GpsLocation* location)
{
  if (append_to_pdu(pdu, "Sdddfff", location->flags,
                                    location->latitude,
                                    location->longitude,
                                    location->altitude,
                                    location->speed,
                                    location->bearing,
                                    location->accuracy) < 0) {
    return -1;
  }
  return append_GpsUtcTime(pdu, &location->timestamp);
}

static long
append_GpsStatusValue(struct pdu* pdu, const GpsStatusValue* status)
{
  return append_to_pdu(pdu, "S", (uint16_t)*status);
}

static long
append_GpsStatus(struct pdu* pdu, const GpsStatus* status)
{
  return append_GpsStatusValue(pdu, &status->status);
}

static long
append_GpsSvInfo(struct pdu* pdu, const GpsSvInfo* svinfo)
{
  return append_to_pdu(pdu, "ifff", (int32_t)svinfo->prn,
                                    svinfo->snr,
                                    svinfo->elevation,
                                    svinfo->azimuth);
}

static long
append_GpsSvStatus(struct pdu* pdu, const GpsSvStatus* svstatus)
{
  long off;
  int i;

  if (svstatus->num_svs < 0) {
    ALOGE("svstatus->num_svs < 0");
    return -1;
  } else if (svstatus->num_svs > (int)UINT8_MAX) {
    ALOGE("svstatus->num_svs > UINT8_MAX");
    return -1;
  }
  off = append_to_pdu(pdu, "IIIC", svstatus->ephemeris_mask,
                                   svstatus->almanac_mask,
                                   svstatus->used_in_fix_mask,
                                   (uint8_t)svstatus->num_svs);
  if (off < 0) {
    return -1;
  }

  for (i = 0; i < svstatus->num_svs; ++i) {
    off = append_GpsSvInfo(pdu, svstatus->sv_list + i);
    if (off < 0) {
      return -1;
    }
  }
  return off;
}

static long
read_GpsUtcTime(const struct pdu* pdu, unsigned long offset, GpsUtcTime* time)
{
  return read_pdu_at(pdu, offset, "l", (int64_t*)time);
}

static long
read_GpsAidingData(const struct pdu* pdu, unsigned long offset,
                   GpsAidingData* data)
{
  return read_pdu_at(pdu, offset, "S", (uint16_t*)data);
}

static long
read_GpsPositionMode(const struct pdu* pdu, unsigned long offset,
                     GpsPositionMode* mode)
{
  return read_pdu_at(pdu, offset, "I", (uint32_t*)mode);
}

static long
read_GpsPositionRecurrence(const struct pdu* pdu, unsigned long offset,
                           GpsPositionRecurrence* recurrence)
{
  return read_pdu_at(pdu, offset, "I", (uint32_t*)recurrence);
}

/*
 * Notifications
 */

static void
location_cb(GpsLocation* location)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(46, /* location */
                         0, NULL);
  if (!wbuf)
    return;

  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_LOCATION_NTF);
  if (append_GpsLocation(&wbuf->buf.pdu, location) < 0) {
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
status_cb(GpsStatus* status)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(2, /* status */
                         0, NULL);
  if (!wbuf)
    return;

  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_STATUS_NTF);
  if (append_GpsStatus(&wbuf->buf.pdu, status) < 0) {
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
sv_status_cb(GpsSvStatus* sv_info)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(4 + /* ephemeris mask */
                         4 + /* almanac mask */
                         4 + /* used-in-fix mask */
                         1 + /* number of SV infos */
                         16 * sv_info->num_svs, /* SV infos */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_SV_STATUS_NTF);
  if (append_GpsSvStatus(&wbuf->buf.pdu, sv_info) < 0) {
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
nmea_cb(GpsUtcTime timestamp, const char* nmea, int length)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(8 + /* timestamp */
                         2 +/* name length */
                         length, /* name */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_NMEA_NTF);
  if ((append_GpsUtcTime(&wbuf->buf.pdu, &timestamp) < 0) ||
      (append_to_pdu(&wbuf->buf.pdu, "Sm", (uint16_t)length,
                                           nmea, (size_t)length) < 0)) {
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
set_capabilities_cb(uint32_t capabilities)
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(4, /* capabilities */
                         0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_SET_CAPABILITIES_NTF);
  if (append_to_pdu(&wbuf->buf.pdu, "I", capabilities) < 0) {
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
acquire_wakelock_cb(void)
{
  acquire_wake_lock(PARTIAL_WAKE_LOCK, WAKE_LOCK_NAME);
}

static void
release_wakelock_cb(void)
{
  release_wake_lock(WAKE_LOCK_NAME);
}

static pthread_t
create_thread_cb(const char* name, void (*start)(void*), void* arg)
{
  return create_thread(name, start, arg);
}

static void
request_utc_time_cb()
{
  struct pdu_wbuf* wbuf;

  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return;
  }
  init_pdu(&wbuf->buf.pdu, SERVICE_GPS, OPCODE_REQUEST_UTF_TIME_NTF);

  if (run_task(send_ntf_pdu, wbuf) < 0) {
    goto cleanup;
  }

  return;
cleanup:
  destroy_pdu_wbuf(wbuf);
}

/*
 * Commands/Responses
 */

static int
start(const struct pdu* cmd)
{
  struct pdu_wbuf* wbuf;
  int err;

  assert(gps_interface);
  assert(gps_interface->start);

  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf)
    return ERROR_NOMEM;

  err = gps_interface->start();
  if (err)
    goto err_gps_interface_start;

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_gps_interface_start:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
stop(const struct pdu* cmd)
{
  struct pdu_wbuf* wbuf;
  int err;

  assert(gps_interface);
  assert(gps_interface->stop);

  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf)
    return ERROR_NOMEM;

  err = gps_interface->stop();
  if (err)
    goto err_gps_interface_stop;

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_gps_interface_stop:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
inject_time(const struct pdu* cmd)
{
  long off;
  GpsUtcTime time;
  int64_t time_ref;
  int32_t uncertainty;
  struct pdu_wbuf* wbuf;
  int err;

  assert(gps_interface);
  assert(gps_interface->inject_time);

  off = read_GpsUtcTime(cmd, 0, &time);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "li", &time_ref, &uncertainty) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  err = gps_interface->inject_time(time, time_ref, uncertainty);
  if (err) {
    goto err_gps_interface_inject_time;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;

err_gps_interface_inject_time:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
inject_location(const struct pdu* cmd)
{
  double latitude, longitude;
  float accuracy;
  struct pdu_wbuf* wbuf;
  int err;

  assert(gps_interface);
  assert(gps_interface->inject_location);

  if (read_pdu_at(cmd, 0, "ddf", &latitude, &longitude, &accuracy) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  err = gps_interface->inject_location(latitude, longitude, accuracy);
  if (err) {
    goto err_gps_interface_inject_location;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_gps_interface_inject_location:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
delete_aiding_data(const struct pdu* cmd)
{
  GpsAidingData data;
  struct pdu_wbuf* wbuf;

  assert(gps_interface);
  assert(gps_interface->inject_time);

  if (read_GpsAidingData(cmd, 0, &data) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  gps_interface->delete_aiding_data(data);

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
}

static int
set_position_mode(const struct pdu* cmd)
{
  long off;
  GpsPositionMode mode;
  GpsPositionRecurrence recurrence;
  uint32_t min_interval, pref_accuracy, pref_time;
  struct pdu_wbuf* wbuf;
  int err;

  assert(gps_interface);
  assert(gps_interface->set_position_mode);

  off = read_GpsPositionMode(cmd, 0, &mode);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  off = read_GpsPositionRecurrence(cmd, off, &recurrence);
  if (off < 0) {
    return ERROR_PARM_INVALID;
  }
  if (read_pdu_at(cmd, off, "III", &min_interval,
                                   &pref_accuracy,
                                   &pref_time) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  err = gps_interface->set_position_mode(mode, recurrence, min_interval,
                                         pref_accuracy, pref_time);
  if (err) {
    goto err_gps_interface_set_position_mode;
  }
  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_gps_interface_set_position_mode:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
gps_handler(const struct pdu* cmd)
{
  static int (* const handler[256])(const struct pdu*) = {
    [OPCODE_START] = start,
    [OPCODE_STOP] = stop,
    [OPCODE_INJECT_TIME] = inject_time,
    [OPCODE_INJECT_LOCATION] = inject_location,
    [OPCODE_DELETE_AIDING_DATA] = delete_aiding_data,
    [OPCODE_SET_POSITION_MODE] = set_position_mode
  };

  return handle_pdu_by_opcode(cmd, handler);
}

int
(*register_gps(void (*send_pdu_cb)(struct pdu_wbuf*)))(const struct pdu*)
{
#define container(_t, _v, _m) \
  ( (_t*)( ((const unsigned char*)(_v)) - offsetof(_t, _m) ) )

  static GpsCallbacks gps_callbacks = {
    .size = sizeof(gps_callbacks),
    .location_cb = location_cb,
    .status_cb = status_cb,
    .sv_status_cb = sv_status_cb,
    .nmea_cb = nmea_cb,
    .set_capabilities_cb = set_capabilities_cb,
    .acquire_wakelock_cb = acquire_wakelock_cb,
    .release_wakelock_cb = release_wakelock_cb,
    .create_thread_cb = create_thread_cb,
    .request_utc_time_cb = request_utc_time_cb
  };

  int err;
  hw_module_t* module;
  hw_device_t* device;

  assert(send_pdu_cb);

  if (gps_device) {
    ALOGE("GPS device already open");
    return NULL;
  }

  if (gps_interface) {
    ALOGE("GPS interface already set up");
    return NULL;
  }

  err = hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
  if (err) {
    ALOGE_ERRNO_NO("hw_get_module", err);
    return NULL;
  }

  err = module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
  if (err) {
    ALOGE_ERRNO_NO("hw_module_t::methods::open", err);
    return NULL;
  }

  gps_device = container(struct gps_device_t, device, common);

  gps_interface = gps_device->get_gps_interface(gps_device);
  if (!gps_interface) {
    ALOGE("gps_device_t::get_gps_interface failed");
    goto err_gps_device_get_gps_interface;
  }

  err = gps_interface->init(&gps_callbacks);
  if (err) {
    ALOGE("gps_interface_t::init failed");
    goto err_gps_interface_init;
  }

  send_pdu = send_pdu_cb;

  return gps_handler;

err_gps_interface_init:
  gps_interface = NULL;
err_gps_device_get_gps_interface:
  gps_device = NULL;
  err = device->close(device);
  if (err)
    ALOGW_ERRNO_NO("hw_device_t::close", err);
  return NULL;
#undef container
}

int
unregister_gps()
{
  int err;

  assert(gps_interface);
  assert(gps_interface->cleanup);
  assert(gps_device);

  gps_interface->cleanup();
  gps_interface = NULL;

  if (gps_device->common.close) {
    err = gps_device->common.close(&gps_device->common);
    if (err)
      ALOGW_ERRNO_NO("gps_device_t::common::close", err);
  }
  gps_device = NULL;

  return 0;
}

const void*
get_gps_extension(const char* name)
{
  assert(gps_interface);
  assert(gps_interface->get_extension);

  return gps_interface->get_extension(name);
}
