"""
Scripts to drive a sparrow-yahboom car and train a model for it.

Usage:
    drive.py (drive)(yahboomcar|donkeycar) [--speed=<kn>] [--wechat]
    drive.py (test)
    drive.py (train) (--para=<kn>)
    drive.py (-h|--help)

Options:
    -h --help  Show this screen.
    --speed=<kn>  The parameter is used for setting PWM duty of wheels [default: 30].
    --wechat  Use wechat to control the car.
"""
import time
import os
from docopt import docopt
from memory import Memory
from threading import Thread
from camera import StreamingOutput, StreamingServer, StreamingHandler, webcam_recording, PAGE, output


def train_cmd_test(para = None):
    if not para:
        print('only train command!')
    else:
        print('parameter is %s'%para)

def drive_yahboom(CarSpeedControl, js=False):
    V = Vehicle()
    if not js:
        V.run(CarSpeedControl)

def print_time(threadName, counter, delay):
    while counter:
        time.sleep(delay)
        print ("%s: %s" % (threadName, time.ctime(time.time())))
        counter -= 1


if __name__ == '__main__':
    args = docopt(__doc__)

    if args['drive']:
        if args['yahboomcar']:
            print(args)
            from yahboomcar import *
            V=Vehicle(float(args['--speed']))
            V.start()
    if args['test']:
        mem = Memory()
        entry = {}
        p1 = [1,2,3,4,5]
        p2 = [2,3,4,5]
        r1 = [100, 45]
        r2 = [10000]
        inputs = [p1, p2]
        t = Thread(target=print_time, args=('thread1', 5,2))
        t.setDaemon(True)
        entry['thread'] = t
        entry['inputs'] = ['para1', 'para2']
        entry['outputs'] = ['result']
        try:
            entry.get('thread').start()
            mem.put(entry['inputs'], inputs)
            mem.put(entry['outputs'], [r1, r2])
            print(mem.iteritems())
            print(mem.keys())
            t.join()
        except KeyboardInterrupt:
            pass



