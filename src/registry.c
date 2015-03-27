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

#include "registry.h"
#include <assert.h>
#include <pdu/pdubuf.h>
#include "log.h"
#include "pdu.h"
#include "service.h"

enum {
  /* commands/responses */
  OPCODE_REGISTER_MODULE = 0x01,
  OPCODE_UNREGISTER_MODULE = 0x02,
};

static void (*send_pdu)(struct pdu_wbuf* wbuf);

/*
 * Commands/Responses
 */

static int
register_module(const struct pdu* cmd)
{
  uint8_t service;
  struct pdu_wbuf* wbuf;
  int (*handler)(const struct pdu*);

  if (read_pdu_at(cmd, 0, "C", &service) < 0) {
    return ERROR_PARM_INVALID;
  }
  if (service_handler[service]) {
    ALOGE("service 0x%x already registered", service);
    return ERROR_FAIL;
  }
  if (!register_service[service]) {
    ALOGE("invalid service id 0x%x", service);
    return ERROR_FAIL;
  }

  wbuf = create_pdu_wbuf(4, /* protocol version */
                         0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }

  handler = register_service[service](send_pdu);
  if (!handler) {
    goto err_register_service;
  }
  service_handler[service] = handler;

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);

  if (append_to_pdu(&wbuf->buf.pdu, "I", (uint32_t)PROTOCOL_VERSION) < 0) {
    goto err_append_to_pdu;
  }

  send_pdu(wbuf);

  return ERROR_NONE;
err_append_to_pdu:
err_register_service:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
unregister_module(const struct pdu* cmd)
{
  uint8_t service;
  struct pdu_wbuf* wbuf;

  if (read_pdu_at(cmd, 0, "C", &service) < 0) {
    return ERROR_PARM_INVALID;
  }
  wbuf = create_pdu_wbuf(0, 0, NULL);
  if (!wbuf) {
    return ERROR_NOMEM;
  }
  if (service == SERVICE_REGISTRY) {
    ALOGE("service REGISTRY cannot be unregistered");
    goto err_service_registry;
  }
  if (!unregister_service[service]) {
    ALOGE("service 0x%x not registered", service);
    goto err_not_unregister_service;
  }
  if (unregister_service[service]() < 0) {
    goto err_unregister_service;
  }
  service_handler[service] = NULL;

  init_pdu(&wbuf->buf.pdu, cmd->service, cmd->opcode);
  send_pdu(wbuf);

  return ERROR_NONE;
err_unregister_service:
err_not_unregister_service:
err_service_registry:
  destroy_pdu_wbuf(wbuf);
  return ERROR_FAIL;
}

static int
registry_handler(const struct pdu* cmd)
{
  static int (* const handler[256])(const struct pdu*) = {
    [OPCODE_REGISTER_MODULE] = register_module,
    [OPCODE_UNREGISTER_MODULE] = unregister_module
  };

  return handle_pdu_by_opcode(cmd, handler);
}

int
init_registry(void (*send_pdu_cb)(struct pdu_wbuf*))
{
  send_pdu = send_pdu_cb;
  service_handler[SERVICE_REGISTRY] = registry_handler;

  return 0;
}

void
uninit_registry()
{
  service_handler[SERVICE_REGISTRY] = NULL;
  send_pdu = NULL;
}
