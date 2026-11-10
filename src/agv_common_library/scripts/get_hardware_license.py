#!/usr/bin/env python

import hashlib, os

command = 'dmesg | grep UUID | grep "Kernel" | sed "s/.*UUID=//g" | sed "s/\ ro\ quiet.*//g"'
output = os.popen(command).read()
serial = output.strip()

serial = "Mkac@" + serial + "2917"
hash_fr_user = hashlib.md5(serial.encode("UTF-8")).hexdigest()
print("Hardware license:", hash_fr_user)
