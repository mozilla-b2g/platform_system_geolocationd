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

#pragma once

#include <stdint.h>

struct pdu;

/* |PROTOCOL_VERSION| contains the current version number of the IPC
 * protocol. Increment this number when you modify the protocol.
 */
enum {
  PROTOCOL_VERSION = 1
};

enum {
  SERVICE_REGISTRY = 0x00,
  SERVICE_GPS = 0x01,
  SERVICE_AGPS = 0x04,
  SERVICE_AGPS_RIL = 0x07
};

/* No GPS-specific errors, error codes kept in sync with Bluetooth status */
enum {
  ERROR_NONE = 0x0,
  ERROR_FAIL = 0x1,
  ERROR_NOT_READY = 0x2,
  ERROR_NOMEM = 0x3,
  ERROR_BUSY = 0x4,
  ERROR_DONE = 0x5,
  ERROR_UNSUPPORTED = 0x6,
  ERROR_PARM_INVALID = 0x7
};

int
handle_pdu_by_service(const struct pdu* cmd,
                      int (* const handler[256])(const struct pdu*));

int
handle_pdu_by_opcode(const struct pdu* cmd,
                     int (* const handler[256])(const struct pdu*));
