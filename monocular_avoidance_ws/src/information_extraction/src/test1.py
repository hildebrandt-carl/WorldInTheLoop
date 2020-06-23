#!/usr/bin/env python3

import sys
from subprocess import PIPE, Popen
from threading  import Thread
import time
import numpy as np
import re
from queue import Queue, Empty

pos1 = np.zeros(3)
ON_POSIX = 'posix' in sys.builtin_module_names

def process_output(out, queue):
    for line in iter(out.readline, b''):
        if "_first.worldPosition" in line:
            number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
            if ".x" in line:
                pos1[0] = float(number)
            if ".y" in line:
                pos1[1] = float(number)
            if ".z" in line:
                pos1[2] = float(number)
            queue.put(number)
    out.close()

command = "tlm-data-logger -r 0 inet:127.0.0.1:9060"
p = Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
q = Queue()
t = Thread(target=process_output, args=(p.stdout, q))
# thread dies with the program
t.daemon = True 
t.start()

# ... do other things here

# read line without blocking
while True:
    try:  
        line = q.get_nowait()
    except Empty:
        print('no output yet')
    else:
        print(pos1)
        # Clear out the queue
        q.queue.clear()
    time.sleep(1)