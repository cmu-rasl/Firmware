/****************************************************************************
 *
 * Copyright (C) 2016 Nathan Michael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/prctl.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "MocapAttitudeController.h"

/**< Deamon exit flag */
static bool thread_should_exit = false;

/**< Deamon status flag */
static bool thread_running = false;

/**< Handle of deamon task / thread */
static int mocap_control_task;

/**
 * Mainloop of mocap_control.
 */
int mocap_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  fprintf(stderr, "usage: mocap_control {start|stop|status}\n");
  exit(1);
}

extern "C" __EXPORT int mocap_control_main(int argc, char *argv[]);

int mocap_control_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      warnx("already running\n");
      /* this is not an error */
      exit(0);
    }

    thread_should_exit = false;
    mocap_control_task = px4_task_spawn_cmd("mocap_control",
                                            SCHED_DEFAULT,
                                            SCHED_PRIORITY_MAX - 5,
                                            1500,
                                            (px4_main_t)&mocap_control_thread_main,
                                            (argv) ? (char* const*)&argv[2] : nullptr);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    thread_should_exit = true;

    while (thread_running)
    {
      usleep(200000);
    }

    warnx("stopped");
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      warnx("running");
      exit(0);
    }
    else
    {
      warnx("not started");
      exit(1);
    }

    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

int mocap_control_thread_main(int argc, char *argv[])
{
  warnx("[mocap_control] main thread started");
  MocapAttitudeController attitude_controller;

  if (!attitude_controller.initialize())
  {
    err(1, "[mocap_control] failed to initialize attitude controller");
    return -1;
  }

  thread_running = true;

  /* Main loop*/
  while (!thread_should_exit)
  {
    attitude_controller.update();
  }

  thread_running = false;

  attitude_controller.finalize();

  return 0;
}
