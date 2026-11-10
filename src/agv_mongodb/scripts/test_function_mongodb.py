#!/usr/bin/env python
from mongodb import mongodb
from os.path import expanduser
from bson.json_util import dumps

HOME = expanduser("~")

if __name__ == "__main__":
    db = mongodb("mongodb://coffee:coffee@localhost:27017")
    # db = mongodb("mongodb://localhost:27017/")
    db.calcWaypoints('pose_17045569451','pose_4','office_cad')
    # Test download file function
    # db.downloadFile("2", HOME + "/Desktop/map/2.png", "map")
    # db.downloadYamlMapFile("mkac",  HOME + "/Desktop/map")
    # db.downloadMap("xxx", HOME + "/Desktop/map")
    # print(db.downloadMap("test", HOME + "/tmp/ros/maps", "png"))
    # db.downloadFile("1", HOME + "/Desktop/map/1.mp3", "sound")
    # db.downloadFiducialMapFile("maptest",  HOME + "/Desktop/map")
    # Test current position function
    # currentPos = db.getCurrentPos(-0.1, -0.1, "map2", 0.5)
    # print(currentPos)
    # currentPos = currentPos if currentPos != "" else "None"
    # print("Current pose: {}".format(currentPos))

    # Test download setting function
    # test = db.downloadSetting("Moving control")
    # getPos = db.getPosCoordinate("1","map2")
    # db.printJson(getPos)
    # db.setCurrentMap("map1")
    # print(db.getCurrentMap())
    # Test calculate waypoint
    # ret = db.calcWaypoint('pose_1', 'pose_2', 'map2')
    # db.printJson(ret)
    # db.printJson(db.getSafety("amr run with cart"))
    # data_dict = db.getQueueMission(curMap="office")
    # json_string = dumps(data_dict, indent=2, sort_keys=True)
    # print(json_string)
    # print(db.loadOdom(["left", "right"]))

    # print(db.saveOdom(["left", "right"], [1, 2]))

    # UWB
    # mycol = db.mapsCollection
    # map_name = "office_01"

    # # Print collection before update
    # for x in mycol.find({"name": map_name}):
    #     print(x)

    # # # Add params
    # # data = {"uwb": {"x_offset": 1, "y_offset": 1, "yaw": 0}}
    # # # mydb.saveUwbTransform(map_name, data)
    # # mydb.saveUwbTransform(map_name, data)

    # # # Print collection after update
    # # for x in mycol.find({"name": map_name}):
    # #     print(x)

    # params = db.loadUwbTransform(map_name)
    # print(params)
