"""
Scripts to drive a sparrow-yahboom car and train a model for it.

Usage:
    drive.py (drive)(yahboomcar|donkeycar) [--speed=<kn>] [--wechat]
    drive.py (train) (--para=<kn>)
    drive.py (-h|--help)

Options:
    -h --help  Show this screen.
    --speed=<kn>  The parameter is used for setting PWM duty of wheels [default: 30].
    --wechat  Use wechat to control the car.
"""

import os
from docopt import docopt

def train_cmd_test(para = None):
    if not para:
        print('only train command!')
    else:
        print('parameter is %s'%para)

def drive_yahboom(CarSpeedControl, js=False):
    V = Vehicle()
    if not js:
        V.run(CarSpeedControl)

if __name__ == '__main__':
    args = docopt(__doc__)

    if args['drive']:
        if args['yahboomcar']:
            print(args)
            from yahboomcar import *
            V=Vehicle(float(args['--speed']))
            V.start()

