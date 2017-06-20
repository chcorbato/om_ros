'''
@author: Arturo Bajuelos
Modified on Feb 29, 2012 by chcorbato

@deprecated: replaced by by ch_syscall.py
'''
import subprocess
from popen_improved import Popen as Popen

def runCmd(cmd, timeout=None):
    '''
    Will execute a command, read the output and return it back.
    
    @param cmd: command to execute
    @param timeout: process timeout in seconds
    @return: aggregated of return code
    @raise OSError: on missing command or if a timeout was reached
    '''

    ph_out = None # process output
    ph_err = None # stderr
    ph_ret = None # return code
    p = subprocess.Popen(cmd, shell=True)
                        # stdout=subprocess.PIPE,
                         #stderr=subprocess.PIPE)

   
        # if timeout is not set wait for process to complete
    if not timeout:
        ph_ret = p.wait()
    else:
        fin_time = time.time() + timeout
        while p.poll() == None and fin_time > time.time():
            time.sleep(1)

        # if timeout reached, raise an exception
        if fin_time < time.time():

            # starting 2.6 subprocess has a kill() method which is preferable
            # p.kill()
            os.kill(p.pid, signal.SIGKILL)
            raise OSError("Process timeout has been reached")

        ph_ret = p.returncode

  #  print "PID: "+str(p.pid)
    return ph_ret



def runCmdOutput(cmd, timeout=None):
    '''
    Will execute a command, read the output and return it back.
    
    @param cmd: command to execute
    @param timeout: process timeout in seconds
    @return: a tuple of three: first stdout, then stderr, then exit code
    @raise OSError: on missing command or if a timeout was reached
    '''

    ph_out = None # process output
    ph_err = None # stderr
    ph_ret = None # return code

    p = subprocess.Popen(cmd, shell=True)#, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

   
        # if timeout is not set wait for process to complete
    if not timeout:
        ph_ret = p.wait()
    else:
        fin_time = time.time() + timeout
        while p.poll() == None and fin_time > time.time():
            time.sleep(1)

        # if timeout reached, raise an exception
        if fin_time < time.time():

            # starting 2.6 subprocess has a kill() method which is preferable
            # p.kill()
            os.kill(p.pid, signal.SIGKILL)
            raise OSError("Process timeout has been reached")
        ph_ret = p.returncode

  #  print "PID: "+str(p.pid)
    #ph_out, ph_err = p.communicate() # this waits for children to terminate: not usable for roslaunch
    error = p.stdout[0]
    
    print 'error'
    print error
    
    return error #(ph_out, ph_err, ph_ret)
