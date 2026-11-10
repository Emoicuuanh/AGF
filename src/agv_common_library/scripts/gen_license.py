#!/usr/bin/env python

import hashlib
import sys
import rospy

input_argv = rospy.myargv(argv=sys.argv)


def main():
    if len(input_argv) > 1:
        hash_fr_user = input_argv[1]
        print("Hash fr user:", hash_fr_user)
        final_txt = "1007" + str(hash_fr_user) + "meik0!"
        final_hash = hashlib.md5(final_txt.encode("UTF-8")).hexdigest()
        print("Final hash:", final_hash)


if __name__ == "__main__":
    main()
