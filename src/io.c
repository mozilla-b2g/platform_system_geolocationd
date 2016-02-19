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

#include "io.h"
#include <assert.h>
#include <cutils/sockets.h>
#include <fcntl.h>
#include <fdio/loop.h>
#include <hardware_legacy/power.h>
#include <pdu/pdubuf.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include "compiler.h"
#include "log.h"
#include "pdu.h"
#include "registry.h"
#include "service.h"
#include "wakelock.h"

#define ARRAY_LENGTH(_array) \
  ( sizeof(_array) / sizeof((_array)[0]) )

/*
 * Socket I/O
 */

static enum ioresult
io_fd0_event(int fd, uint32_t events, void* data);

#if 0
static enum ioresult
io_fd1_event(int fd, uint32_t events, void* data);
#endif

STAILQ_HEAD(pdu_wbuf_stailq, pdu_wbuf);

struct io_state {
  int fd;
  uint32_t epoll_events;
  enum ioresult (*epoll_func)(int, uint32_t, void*);
  struct pdu_rbuf* rbuf;
  struct pdu_wbuf_stailq sendq;
};

static void
io_state_err(struct io_state* io_state)
{
  assert(io_state);

  if (io_state->rbuf) {
    destroy_pdu_rbuf(io_state->rbuf);
    io_state->rbuf = NULL;
  }
  remove_fd_from_epoll_loop(io_state->fd);
  TEMP_FAILURE_RETRY(close(io_state->fd)); /* no error checks here */
  io_state->epoll_events = 0;
  io_state->fd = -1;
}

static void
io_state_hup(struct io_state* io_state)
{
  assert(io_state);

  if (io_state->rbuf) {
    destroy_pdu_rbuf(io_state->rbuf);
    io_state->rbuf = NULL;
  }
  remove_fd_from_epoll_loop(io_state->fd);
  if (TEMP_FAILURE_RETRY(close(io_state->fd)) < 0)
    ALOGW_ERRNO("close");
  io_state->epoll_events = 0;
  io_state->fd = -1;
}

static int
io_state_in(struct io_state* io_state, int (*handle_pdu)(const struct pdu*))
{
  struct iovec iv;
  struct msghdr msg;
  ssize_t res;

  assert(io_state);
  assert(handle_pdu);

  acquire_wake_lock(PARTIAL_WAKE_LOCK, WAKE_LOCK_NAME);

  memset(&iv, 0, sizeof(iv));
  iv.iov_base = io_state->rbuf->buf.raw;
  iv.iov_len = io_state->rbuf->maxlen;

  memset(&msg, 0, sizeof(msg));
  msg.msg_iov = &iv;
  msg.msg_iovlen = 1;

  res = TEMP_FAILURE_RETRY(recvmsg(io_state->fd, &msg, 0));
  if (res < 0) {
    ALOGE_ERRNO("recvmsg");
    goto err_recvmsg;
  } else if (!res) {
    /* stop watching if peer hung up */

    io_state->epoll_events &= ~EPOLLIN;

    if (io_state->epoll_events) {
      res = add_fd_to_epoll_loop(io_state->fd,
                                 io_state->epoll_events,
                                 io_state->epoll_func, io_state);
      if (res < 0)
        goto err_add_fd_to_epoll_loop;
    } else {
      remove_fd_from_epoll_loop(io_state->fd);
    }
  }

  io_state->rbuf->len = res;

  if (pdu_rbuf_has_pdu(io_state->rbuf)) {
    if (handle_pdu(&io_state->rbuf->buf.pdu) < 0)
      goto err_pdu;
    io_state->rbuf->len = 0;
  } else if (pdu_rbuf_is_full(io_state->rbuf)) {
    ALOGE("buffer too small for PDU(0x%x:0x%x)",
          io_state->rbuf->buf.pdu.service, io_state->rbuf->buf.pdu.opcode);
    goto err_pdu;
  }

  release_wake_lock(WAKE_LOCK_NAME);

  return 0;
err_pdu:
err_add_fd_to_epoll_loop:
err_recvmsg:
  release_wake_lock(WAKE_LOCK_NAME);
  return -1;
}

static int
io_state_out(struct io_state* io_state)
{
  struct pdu_wbuf* wbuf;
  int res;

  assert(io_state);

  if (!STAILQ_EMPTY(&io_state->sendq)) {
    /* send next pending PDU */

    wbuf = STAILQ_FIRST(&io_state->sendq);
    STAILQ_REMOVE_HEAD(&io_state->sendq, stailq);

    send_pdu_wbuf(wbuf, io_state->fd, 0);
    destroy_pdu_wbuf(wbuf);
  }

  if (STAILQ_EMPTY(&io_state->sendq)) {
    /* stop watching */

    io_state->epoll_events &= ~EPOLLOUT;

    if (io_state->epoll_events) {
      res = add_fd_to_epoll_loop(io_state->fd,
                                 io_state->epoll_events,
                                 io_state->epoll_func, io_state);
      if (res < 0)
        goto err_add_fd_to_epoll_loop;
    } else {
      remove_fd_from_epoll_loop(io_state->fd);
    }
  }

  return 0;
err_add_fd_to_epoll_loop:
  return -1;
}

static int
io_state_send(struct io_state* io_state, struct pdu_wbuf* wbuf)
{
  uint32_t epoll_events;
  int res;

  STAILQ_INSERT_TAIL(&io_state->sendq, wbuf, stailq);

  if (io_state->epoll_events & EPOLLOUT)
    return 0;

  /* poll file descriptor for writeability */

  epoll_events = io_state->epoll_events | EPOLLOUT;

  res = add_fd_to_epoll_loop(io_state->fd, epoll_events,
                             io_state->epoll_func, io_state);
  if (res < 0)
    goto err_add_fd_to_epoll_loop;

  io_state->epoll_events = epoll_events;

  return 0;
err_add_fd_to_epoll_loop:
  STAILQ_REMOVE(&io_state->sendq, wbuf, pdu_wbuf, stailq);
  destroy_pdu_wbuf(wbuf);
  return -1;
}

#define IO_STATE_INITIALIZER(_io_state, _epoll_func) \
  { \
    .fd = -1, \
    .epoll_events = 0, \
    .epoll_func = (_epoll_func), \
    .rbuf = NULL, \
    STAILQ_HEAD_INITIALIZER((_io_state).sendq) \
  }

static struct io_state io_state[1] = {
  IO_STATE_INITIALIZER(io_state[0], io_fd0_event),
};

static void
send_pdu(struct pdu_wbuf* wbuf)
{
  io_state_send(io_state, wbuf);
}

static int
handle_pdu(const struct pdu* cmd)
{
  int status;
  struct pdu_wbuf* wbuf;

  status = handle_pdu_by_service(cmd, service_handler);
  if (status)
    goto err_handle_pdu_by_service;

  return 0;
err_handle_pdu_by_service:
  /* reply with an error */
  wbuf = create_pdu_wbuf(1, 0, NULL);
  if (!wbuf)
    return -1;
  init_pdu(&wbuf->buf.pdu, cmd->service, 0);
  append_to_pdu(&wbuf->buf.pdu, "C", (uint8_t)status);
  send_pdu(wbuf);
  return 0; /* signal success because we replied with an error */
}

static enum ioresult
io_fd_event_err(int fd ATTRIBS(UNUSED), void* data)
{
  struct io_state* io_state;

  assert(data);

  io_state = data;
  assert(io_state->fd == fd);

  io_state_err(io_state);

  return IO_POLL;
}

static enum ioresult
io_fd_event_hup(int fd ATTRIBS(UNUSED), void* data)
{
  struct io_state* io_state;

  assert(data);

  io_state = data;
  assert(io_state->fd == fd);

  io_state_hup(io_state);

  return IO_EXIT; /* exit with success */
}

static enum ioresult
io_fd_event_in(int fd ATTRIBS(UNUSED), void* data)
{
  struct io_state* io_state;

  assert(data);

  io_state = data;
  assert(io_state->fd == fd);

  if (io_state_in(io_state, handle_pdu) < 0)
    return IO_ABORT;

  return IO_OK;
}

static enum ioresult
io_fd_event_out(int fd ATTRIBS(UNUSED), void* data)
{
  struct io_state* io_state;

  assert(data);

  io_state = data;
  assert(io_state->fd == fd);

  if (io_state_out(io_state) < 0)
    return IO_ABORT;

  return IO_OK;
}

/* Command socket
 */

/* The function |io_fd0_event| handles the command/response file
 * descriptor. It supports IN and OUT events for receiving and
 * sending PDUs. HUP and ERROR are always handled.
 */
static enum ioresult
io_fd0_event(int fd, uint32_t events, void* data)
{
  enum ioresult res;

  if (events & EPOLLERR) {
    res = io_fd_event_err(fd, data);
  } else if (events & EPOLLHUP) {
    res = io_fd_event_hup(fd, data);
  } else if (events & EPOLLIN) {
    res = io_fd_event_in(fd, data);
  } else if (events & EPOLLOUT) {
    res = io_fd_event_out(fd, data);
  } else {
    ALOGW("unsupported event mask: %u", events);
    res = IO_OK;
  }
  return res;
}

#if 0
/* Notification socket
 */

/* The function |io_fd1_event| handles the notification file
 * descriptor. We only send on this file descriptor, so the
 * function only supports OUT events for sending PDUs. HUP
 * and ERROR are always handled.
 */
static enum ioresult
io_fd1_event(int fd, uint32_t events, void* data)
{
  enum ioresult res;

  if (events & EPOLLERR) {
    res = io_fd_event_err(fd, data);
  } else if (events & EPOLLHUP) {
    res = io_fd_event_hup(fd, data);
  } else if (events & EPOLLOUT) {
    res = io_fd_event_out(fd, data);
  } else {
    ALOGW("unsupported event mask: %u", events);
    res = IO_OK;
  }
  return res;
}
#endif

/*
 * Listening socket I/O
 */

static int
connect_socket(const char* socket_name,
               enum ioresult (*func)(int, uint32_t, void*),
               void* data)
{
  static const size_t NAME_OFFSET = 1;

  int fd;
  size_t len, siz;
  struct sockaddr_un addr;
  socklen_t socklen;
  int res;

  assert(socket_name);

  fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (fd < 0) {
    ALOGE_ERRNO("socket");
    goto err_socket;
  }
  if (TEMP_FAILURE_RETRY(fcntl(fd, F_SETFL, O_NONBLOCK)) < 0) {
    ALOGE_ERRNO("fcntl");
    goto err_fcntl;
  }

  len = strlen(socket_name);
  siz = len + 1; /* include trailing '\0' */

  addr.sun_family = AF_UNIX;
  assert(NAME_OFFSET + siz <= sizeof(addr.sun_path));
  memset(addr.sun_path, '\0', NAME_OFFSET); /* abstract socket */
  memcpy(addr.sun_path + NAME_OFFSET, socket_name, siz);

  socklen = offsetof(struct sockaddr_un, sun_path) + NAME_OFFSET + siz;

  res = TEMP_FAILURE_RETRY(connect(fd,
                                   (const struct sockaddr*)&addr,
                                   socklen));
  if (res < 0) {
    ALOGE_ERRNO("connect");
    goto err_connect;
  }

  if (add_fd_to_epoll_loop(fd, EPOLLOUT|EPOLLERR, func, data) < 0)
    goto err_add_fd_to_epoll_loop;

  return fd;
err_add_fd_to_epoll_loop:
err_connect:
err_fcntl:
  if (TEMP_FAILURE_RETRY(close(fd)) < 0)
    ALOGW_ERRNO("close");
err_socket:
  return -1;
}

static enum ioresult
fd_event_err(int fd ATTRIBS(UNUSED), void* data ATTRIBS(UNUSED))
{
  /* We don't attempt to repair a failed connection request. If an
   * error occures, we abort the daemon and let Gecko start a new
   * process.
   */
  return IO_ABORT;
}

/* The first connected socket is for pairs of command/respond PDUs.
 */
static enum ioresult
connected_cmd_socket(int fd)
{
  struct pdu_rbuf* rbuf;

  /* Remove fd from current loop to clear call-back function */
  remove_fd_from_epoll_loop(fd);

  /* Setup socket state */

  rbuf = create_pdu_rbuf(1ul<<16);
  if (!rbuf)
    return IO_ABORT;

  io_state[0].epoll_events = EPOLLERR | EPOLLIN;
  io_state[0].rbuf = rbuf;

  return IO_OK;
}

static int
start_main_loop(void)
{
  int res;

  if (init_registry(send_pdu) < 0)
    return -1;

  res = add_fd_to_epoll_loop(io_state[0].fd, io_state[0].epoll_events,
                             io_fd0_event, io_state + 0);
  if (res < 0)
    goto err_add_fd_to_epoll_loop;

  return 0;
err_add_fd_to_epoll_loop:
  uninit_registry();
  return -1;
}

static enum ioresult
fd_event_out(int fd, void* data)
{
  static enum ioresult (* const connected_socket[])(int) = {
    [0] = connected_cmd_socket
  };

  size_t i;
  unsigned long* remaining_fds;
  enum ioresult res;

  assert(data);

  for (i = 0; i < ARRAY_LENGTH(connected_socket); ++i) {
    if (io_state[i].fd == fd) {
      break;
    }
  }

  if (i == ARRAY_LENGTH(connected_socket)) {
    ALOGE("No state for file descriptor %d", fd);
    return IO_ABORT; /* There should have been a socket in the array. */
  }

  remaining_fds = data;
  (*remaining_fds)--;

  assert(connected_socket[i]);
  res = connected_socket[i](fd);
  if (res == IO_ABORT) {
    return IO_ABORT;
  }

  if (!(*remaining_fds)) {
    if (start_main_loop() < 0) {
      return IO_ABORT;
    }
  }

  return res;
}

/* |fd_event| handles accepted connection requests. ERR is always handled.
 */
static enum ioresult
fd_event(int fd, uint32_t events, void* data)
{
  enum ioresult res;

  if (events & EPOLLERR) {
    res = fd_event_err(fd, data);
  } else if (events & EPOLLOUT) {
    res = fd_event_out(fd, data);
  } else {
    ALOGW("unsupported event mask: %u", events);
    res = IO_OK;
  }

  return res;
}

int
init_io(const char* socket_name)
{
  static unsigned long remaining_fds = (unsigned long)ARRAY_LENGTH(io_state);

  int fd /*, res*/;

  fd = connect_socket(socket_name, fd_event, &remaining_fds);
  if (fd < 0)
    return -1;

  io_state[0].fd = fd;
  io_state[0].epoll_events = 0;
  io_state[0].rbuf = NULL;

  return 0;

#if 0
err_connect_socket:
  res = TEMP_FAILURE_RETRY(close(io_state[0].fd));
  if (res < 0)
    ALOGW_ERRNO("close");
  return -1;
#endif
}

void
uninit_io()
{
  size_t i;

  for (i = 0; i < ARRAY_LENGTH(io_state); ++i) {
    int res;

    if (io_state[i].fd == -1)
      continue;

    remove_fd_from_epoll_loop(io_state[i].fd);

    res = TEMP_FAILURE_RETRY(close(io_state[i].fd));
    if (res < 0) {
      ALOGW_ERRNO("close");
    }
  }

  uninit_registry();
}
