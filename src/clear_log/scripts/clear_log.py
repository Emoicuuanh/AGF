#!/usr/bin/env python
import os
import time
from datetime import datetime
import shutil
from os.path import expanduser
import argparse

HOME = expanduser("~")

def clear_log(path, keep_date=0):
    if not os.path.exists(path):
        print("{} not exist".format(path))
        return
    print("Last log time: {}".format(time.ctime(max(os.stat(root).st_mtime for root,_,_ in os.walk(path)))))
    today = datetime.now()
    for root, dirname, filename in os.walk(path):
        time_stamp = os.stat(root).st_mtime
        d = datetime.fromtimestamp(time_stamp)
        duration = (today - d).days
        if keep_date == 0 or duration > keep_date: # Todo: check file size
            print('Delete: {}, {} days, {}'.format(root, duration, d.strftime('%Y-%m-%d')))
            shutil.rmtree(root)

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-k", '--keep_date', dest="keep_date",
                    default=0,
                    type=int, help='Keep date')

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    keep_date = options.keep_date
    clear_log(HOME + '/.ros/log', keep_date)
    clear_log(HOME + '/log/ros', keep_date)
    clear_log(HOME + '/log/ai', keep_date)

if __name__ == '__main__':
    main()
