'''
Created on Feb 29, 2012

@author: chcorbato

@summary: based on http://stackoverflow.com/questions/375427/non-blocking-read-on-a-subprocess-pipe-in-python
            and syscall.py in this same package
'''
import sys, subprocess
from subprocess import PIPE, Popen
from threading  import Thread

try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x

ON_POSIX = 'posix' in sys.builtin_module_names

def enqueue_output(out, queue):
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()
    
#----------------------------------------------------------------------------------

def runCommand(cmd, tout=3):
    '''
    Will execute a command, read the output and return it back.
    
    @param cmd: command to execute
    @param tout: time to wait for child process output - @todo: this is a critical parameter
    @return: the pair result: 1 if no error, 0 otherwise and
                log: stdout and stderr of child process during given tout
    @raise OSError: on missing command or if a timeout was reached
    '''
    p = Popen(cmd, bufsize=1, shell=True, stdout=PIPE, stderr=subprocess.STDOUT, close_fds=ON_POSIX)

    q = Queue()
    t = Thread(target=enqueue_output, args=(p.stdout, q))
    t.daemon = True # thread dies with the program
    t.start()
   
    # read line without blocking
    try:  line = q.get(block=True, timeout=tout) # or q.get(timeout=.1)
    except Empty:
        return (1,"")
    else: # got line
        # ... do something with line
        return (0,line)