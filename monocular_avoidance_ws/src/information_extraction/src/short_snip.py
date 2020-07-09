#!/usr/bin/env python3
from subprocess import PIPE, Popen
from threading  import Thread
import sys
import numpy as np
import re
from queue import Queue, Empty

# Process the output from the file
def process_output(out, queue):
    for line in iter(out.readline, b''):
        line = str(line)
        if ".worldPosition" in line:
            number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
            if ".x" in line:
                pos1[0] = float(number)
            if ".y" in line:
                pos1[1] = float(number)
            if ".z" in line:
                pos1[2] = float(number)


        if ".worldAttitude" in line:
            number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
            if ".x" in line:
                att1[0] = float(number)
            if ".y" in line:
                att1[1] = float(number)
            if ".z" in line:
                att1[2] = float(number)
                
        queue.put(line)
    out.close()


if __name__ == "__main__":
    pos1 = np.zeros(3)
    att1 = np.zeros(3)
    q = Queue()

    # Run the command
    ON_POSIX = 'posix' in sys.builtin_module_names
    command = "tlm-data-logger -r 0 inet:127.0.0.1:9060"
    p = Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
    
    # Create a thread which dies with main program
    t = Thread(target=process_output, args=(p.stdout, q))
    t.daemon = True 
    t.start()

    for i in range(10000):
        try:  
            line = q.get_nowait()
        except Empty:
            # Clear out the queue
            q.queue.clear()

        print("Position: {}, {}, {}".format(pos1[0], pos1[1], pos1[2]))
        print("Orientation: {}, {}, {}".format(att1[0], att1[1], att1[2]))

    print("System Exiting\n")
    sys.exit(0)