#! /usr/bin/env python3

import subprocess
import sys
import pyinotify
import os
import time


class OnWriteHandler(pyinotify.ProcessEvent):
    def my_init(self, cwd, extension, cmd):
        self.cwd = cwd
        self.extensions = extension.split(',')
        self.cmd = cmd

        self.last_update_time = time.time()

    def _run_cmd(self):
        print('==> Modification detected')
        subprocess.call(self.cmd.split(' '), cwd=self.cwd)

    def process_IN_MODIFY(self, event):
        if all(not event.pathname.endswith(ext) for ext in self.extensions):
            return
        if (time.time() - self.last_update_time) > 1:
            print('++++++++++ rebuild +++++++++++')
            self._run_cmd()
            self.last_update_time = time.time()



def auto_compile(path, extension, cmd):
    wm = pyinotify.WatchManager()
    handler = OnWriteHandler(cwd=path, extension=extension, cmd=cmd)
    notifier = pyinotify.Notifier(wm, default_proc_fun=handler)
    wm.add_watch(path, pyinotify.ALL_EVENTS, rec=True, auto_add=True)
    print("==> Start monitoring %s (type c^c to exit)" % path)
    notifier.loop()


if __name__ == '__main__':
    path = os.path.dirname(os.path.realpath(__file__))
    extension = 'rst,cpp,py'
    cmd = './build_locally.sh'
    auto_compile(path, extension, cmd)
