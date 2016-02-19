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

#include "thread.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "log.h"

struct thread_data {
  void (*start)(void*);
  void* arg;
};

static void*
start_thread(void* arg)
{
  struct thread_data* data;

  assert(arg);

  data = arg;

  assert(data->start);
  data->start(data->arg);

  free(data);

  return NULL;
}

pthread_t
create_thread(const char* name, void (*start)(void*), void* arg)
{
  struct thread_data* data;
  pthread_t thread;
  int err;

  data = malloc(sizeof(*data));
  if (!data) {
    ALOGE_ERRNO("malloc");
    return 0;
  }

  data->start = start;
  data->arg = arg;

  err = pthread_create(&thread, NULL, start_thread, data);
  if (err) {
    ALOGE_ERRNO_NO("pthread_create", err);
    return 0;
  }

  if (name) {
    char nam[16]; // POSIX restricts thread name to 16 characters
    size_t len;

    len = strnlen(name, sizeof(nam) - 1);
    memcpy(nam, name, len);
    nam[len] = '\0';

    err = pthread_setname_np(thread, nam);
    if (err) {
      ALOGW_ERRNO_NO("pthread_setname_np", err);
    }
  }

  return thread;
}
