#!/usr/bin/env python
import os
import time
from datetime import datetime
import shutil
from os.path import expanduser

HOME = expanduser("~")


def main():
    # print(
    #     time.ctime(
    #         max(
    #             os.stat(root).st_mtime
    #             for root, _, _ in os.walk(HOME + "/.ros/log")
    #         )
    #     )
    # )
    today = datetime.now()

    for root, dirname, filename in os.walk(HOME + "/.ros/log"):
        time_stamp = os.stat(root).st_mtime
        d = datetime.fromtimestamp(time_stamp)
        duration = (today - d).days
        if duration > 30:  # Todo: check file size
            print(
                "Delete: {}, {} days, {}".format(
                    root, duration, d.strftime("%Y-%m-%d")
                )
            )
            shutil.rmtree(root)


if __name__ == "__main__":
    main()
