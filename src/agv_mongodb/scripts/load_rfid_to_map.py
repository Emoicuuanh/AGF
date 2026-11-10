#!/usr/bin/env python
from mongodb import mongodb
from os.path import expanduser
from bson.json_util import dumps
import rospkg
import os

HOME = expanduser("~")


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-m",
        "--map",
        dest="map",
    )
    parser.add_option(
        "-d",
        "--data_folder",
        dest="data_folder",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"),
            "cfg",
            "coordinate_rfid",
            "data",
        ),
    )
    parser.add_option(
        "-f",
        "--file",
        dest="file",
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()

    # rosrun agv_mongodb load_rfid_to_map.py -m nc_room -f tendo_floor_1

    db = mongodb("mongodb://coffee:coffee@localhost:27017")
    # db = mongodb("mongodb://localhost:27017/")

    db.loadRfid2Map(
        options.map, os.path.join(options.data_folder, options.file)
    )


if __name__ == "__main__":
    main()
