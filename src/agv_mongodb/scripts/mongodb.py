#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum
import json
from re import sub

# pip install dnspython
from dns.rdatatype import NULL

# pip install pymongo
from pymongo import MongoClient
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gridfs import GridFSBucket
from bson.json_util import dumps
import copy
import rospkg
from dijkstra import build_graph, dijkstra
from geometry_msgs.msg import Pose
import os, sys
import datetime
import math
import rospy

# pip install pyyaml
import yaml
from os.path import expanduser

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    EnumString,
    ActionResult,
    angle_two_pose,
    lockup_pose,
    offset_pose_x,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    get_yaw,
    normalize_angle,
)


class LogLevel(EnumString):
    INFO = 0
    WARN = 1
    ERROR = 2


class MissionStatus(EnumString):
    RUNNING = 0
    SUCCEEDED = 1
    PAUSED = 2
    ERROR = 3
    CANCEL = 4


class mongodb:
    def __init__(self, connectionString="mongodb://localhost:27017/"):
        self.client = MongoClient(connectionString)
        self.database = self.getDatabase("admin")
        self.missionCollection = self.database["missions"]
        self.actionsCollection = self.database["actions"]
        self.dataListCollection = self.database["datalist"]
        self.markerTypeCollection = self.database["marker_type"]
        self.waypointsCollection = self.database["path_guides"]
        self.queueCollection = self.database["mission_queue"]
        self.mapsCollection = self.database["maps"]
        self.settingCollection = self.database["setting"]
        self.logCollection = self.database["log"]
        self.missionLogCollection = self.database["mission_log"]
        self.dataLogCollection = self.database[
            "data_log"
        ]  # Save data need update
        self.rulesPathCollection = self.database["path_rules"]
        self.fs = GridFSBucket(self.database)

    def getRulePath(self, map):
        return list(self.rulesPathCollection.find({"map": map}))

    def getDatabase(self, args):
        return self.client[args]

    def printJson(self, data):
        print_info(dumps(data, indent=4, sort_keys=True))
        return data

    def bson2Json(self, data):
        return dumps(data, indent=4, sort_keys=True)

    def bson2Dict(self, data):
        json.loads(self.bson2Json(data))

    def getActionsDetail(self, action, map):
        actionType = [
            copy.deepcopy(x)
            for x in self.listActions
            if x["name"] == action["type"]
        ]
        ret = copy.deepcopy(action)
        for item in actionType[0]["params"]:
            if item["type"] == "fromlist":
                value = ret["params"][item["name"]]  # value of actions param
                fromListData = [
                    copy.deepcopy(x)
                    for x in self.listDataList
                    if x["type"] == item["name"] and x["name"] == value
                ]
                _len = len(fromListData)
                for i in range(0, _len):
                    if "map" in fromListData[_len - i - 1]:
                        if fromListData[_len - i - 1]["map"] != map:
                            fromListData.remove(fromListData[_len - i - 1])
                # for filter in fromListData:
                #     if "map" in filter:
                #         if filter["map"] != map:
                #             fromListData.remove(filter)
                if len(fromListData) != 1:
                    print_error("Wrong data. Maybe duplicated or no data")
                    return

                if fromListData[0]["type"] == "marker":
                    # if data type is marker, we need get marker position and marker type params
                    pos = fromListData[0]["data"]
                    del ret["params"][item["name"]]
                    ret["params"][item["name"]] = {}
                    ret["params"][item["name"]]["position"] = pos
                    markerType = fromListData[0]["type_detail"]
                    marker = [
                        copy.deepcopy(y)
                        for y in self.listMarkerType
                        if y["name"] == markerType
                    ]
                    del marker[0]["_id"]
                    del marker[0]["name"]
                    ret["params"][item["name"]][
                        fromListData[0]["type"]
                    ] = marker
                else:
                    ret["params"][item["name"]] = fromListData[0]["data"]
        return ret

    def getMovingAction(self, action, fromPos, map):
        ret = {}
        ret["name"] = action["name"]
        ret["type"] = action["type"]
        ret["params"] = {}
        toPos = {}
        print(f"Get moving action: {action}")
        if action["type"] == "move":
            for k, v in action["params"].items():
                if k != "position":
                    ret["params"][k] = v
            ret["params"]["reconfigure"] = {}
            toPos = action["params"]["position"]
        if action["type"] == "follow_path":
            for k, v in action["params"].items():
                if k != "position":
                    ret["params"][k] = v
            ret["params"]["reconfigure"] = {}
            toPos = action["params"]["to_pos"]
            fromPos = action["params"]["from_pos"]
        print_info(
            'Get waypoints from "{}" to "{}" in map "{}"'.format(
                fromPos, toPos, map
            )
        )
        print_debug("db:calcWaypoints")
        ret["waypoints"] = self.calcWaypoints(fromPos, toPos, map)
        if ret["waypoints"] ==[]:
            return "NO_ROUTE"
        print_info("Waypoints: \n{}\n".format(dumps(ret, indent=2)))
        return ret

    def getPositions(self, mapName):
        listDataList = list(
            self.dataListCollection.find({"map": mapName, "type": "position"})
        )
        jsonDict = self.bson2Json(listDataList)
        dataDict = json.loads(jsonDict)
        print(json.dumps(dataDict, indent=2))

    def calcWaypoints(self, fromPos, toPos, mapName):
        print_debug('calcWaypoints')
        listDataList = list(
            self.dataListCollection.find({"map": mapName, "type": "position"})
        )
        print_debug("listDataList")
        jsonDict = self.bson2Json(listDataList)
        print_debug("jsonDict")
        # print(jsonDict)
        dataDict = json.loads(jsonDict)
        print_debug("datadict")
        graph = build_graph(dataDict)
        print_debug("build_graph_DONE: {}". format(graph))

        try:
            path, distance = dijkstra(graph,  fromPos, toPos)
            rospy.logerr(f"path found from {fromPos} to {toPos}")
        except Exception as e:
            rospy.logerr(f"No path found from {fromPos} to {toPos}")
            return []

        if path:
            print(f"The shortest path from {fromPos} to {toPos} is: {path}."
                    f" Distance: {round(distance, 2)} m")
        else:
            print(f"No path found from {fromPos} to {toPos}")
            return []

        ret = []
        for w in path:
            for i in dataDict:
                if w == i['name']:
                    i['position'] = i.pop('data') # Rename key "data" -> "position"
                    ret.append(i)
        # print(json.dumps(ret, indent=2))
        return ret

    def calcWaypoint(self, fromPos, toPos, map):
        # get list of all position
        _listDataList = list(
            self.dataListCollection.find({"map": map, "type": "position"})
        )
        _listWaypoints = list(self.waypointsCollection.find({"map": map}))
        _posList = [
            copy.deepcopy(x) for x in _listDataList if x["type"] == "position"
        ]
        _gpCoor = [
            copy.deepcopy(x)
            for x in _posList
            if x["map"] == map and x["name"] == toPos
        ]  # get goal point array
        _gpCoor[0]["position"] = _gpCoor[0].pop(
            "data"
        )  # replace key "data" by "position"
        _gpCoor[0]["position"].pop("display")
        ret = []
        _xpCoor = ""
        for x in _posList:
            if x["map"] == map and x["name"] == fromPos:
                _xpCoor = copy.deepcopy(x)
                _xpCoor["position"] = _xpCoor.pop("data")
                _xpCoor["position"].pop("display")
        if _xpCoor != "":
            ret.append(_xpCoor)
        if fromPos == "" or fromPos == toPos:
            print_info("Ignore make waypoint because fromPos = toPos")
            # ret.append(_gpCoor[0])
            # self.curPos = _gpCoor[0]["name"]
            return ret
        # _spCoor = [copy.deepcopy(x) for x in _posList if x["map"]==map and x["name"]==fromPos] # get list of start point
        # _spCoor[0]["position"] = _spCoor[0].pop("data")  #replace key "data" by "position"
        # _spCoor[0]["position"].pop("display")
        # ret.append(fromPosCoor[0]) # Dont need start point
        for wp in _listWaypoints:
            if wp["map"] == map:
                for sp in wp["start_position"]:
                    if fromPos == sp:
                        for gp in wp["goal_position"]:
                            if toPos == gp:
                                for _wp in wp["waypoints"]:
                                    _wpCoor = [
                                        copy.deepcopy(x)
                                        for x in _posList
                                        if x["map"] == map and x["name"] == _wp
                                    ]
                                    _wpCoor[0]["position"] = _wpCoor[0].pop(
                                        "data"
                                    )
                                    _wpCoor[0]["position"].pop("display")
                                    ret.append(_wpCoor[0])
                                ret.append(_gpCoor[0])
                                self.curPos = _gpCoor[0]["name"]
                                return ret
                for i in range(len(wp["waypoints"])):
                    if fromPos == wp["waypoints"][i]:
                        for gp in wp["goal_position"]:
                            if toPos == gp:
                                for j in range(i + 1, len(wp["waypoints"])):
                                    _wpCoor = [
                                        copy.deepcopy(x)
                                        for x in _posList
                                        if x["map"] == map
                                        and x["name"] == wp["waypoints"][j]
                                    ]
                                    _wpCoor[0]["position"] = _wpCoor[0].pop(
                                        "data"
                                    )
                                    _wpCoor[0]["position"].pop("display")
                                    ret.append(_wpCoor[0])
                                ret.append(_gpCoor[0])
                                self.curPos = _gpCoor[0]["name"]
                                return ret

        ret.append(_gpCoor[0])
        self.curPos = _gpCoor[0]["name"]
        return ret

    def getSafety(self, job_name):
        print_info(job_name)
        print_info("---")
        result = self.dataListCollection.find_one(
            {"type": "safety", "name": job_name}
        )
        if result != None:
            return result["data"]
        return None

    def getFootprint(self, job_name):
        print_info(job_name)
        print_info("---")
        result = self.dataListCollection.find_one(
            {"type": "footprint", "name": job_name}
        )
        if result != None:
            return result["data"]
        return None

    def makeMissionWithVar(self, missionName, curMap, dataDict):
        print_info("Make mission with var: {}".format(missionName))
        varDict = dataDict["variables"] if "variables" in dataDict else None
        desc = dataDict["desc"] if "desc" in dataDict else ""
        group = dataDict["group"] if "group" in dataDict else ""

        self.listActions = list(self.actionsCollection.find())
        self.listDataList = list(self.dataListCollection.find())
        self.listMarkerType = list(self.markerTypeCollection.find())
        self.listWaypoints = list(self.waypointsCollection.find())
        self.listMission = list(
            self.missionCollection.find({"name": missionName, "map": curMap})
        )
        firstMission = self.listMission[0]
        size = len(firstMission["actions"])

        # TODO: Check current position

        # Replace mission's var by value. If not, add mission base to queue
        if varDict != None:
            for i in range(0, size):
                actionParamDict = firstMission["actions"][i]["params"]
                missionVarDict = {}
                if "params" in firstMission:
                    missionVarDict = firstMission["params"]
                    # print_info(missionVarDict)
                if actionParamDict == {} or missionVarDict == {}:
                    break
                for actionParamKey, actionParamValue in actionParamDict.items():
                    paramType = type(actionParamValue).__name__
                    if paramType == "str" and actionParamValue[0] == "$":
                        # print_info(actionParamKey, actionParamValue[1:])
                        isAssign = False
                        for varKey, varValue in missionVarDict.items():
                            if actionParamValue[1:] == varKey:
                                actionParamDict[actionParamKey] = varDict[
                                    actionParamValue[1:]
                                ]
                                isAssign = True
                                break
                        if not isAssign:
                            print_error(
                                "Missing param: {}".format(actionParamValue)
                            )
            del firstMission["_id"]
            if "params" in firstMission:
                del firstMission["params"]
            firstMission["desc"] = desc
            firstMission["group"] = group

            # Set auto name for mission
            number_of_keys = len(varDict)

            # if number_of_keys == 1:
            #     for varKey, varValue in varDict.items():
            #         # print_info(varValue)
            #         firstMission["name"] = (
            #             firstMission["name"] + "_" + str(varValue)
            #         )
            if (number_of_keys > 1 and "pose_1" in varDict and "pose"):
                firstMission["name"] = (
                    firstMission["name"] + "_" + varDict["pose"] + "_" + varDict["pose_1"])
            else:
                firstMission["name"] = (
                    firstMission["name"] + "_" + varDict["pose"])

            print_info("firstMissionName : {}".format(firstMission["name"]))

            # Check if mission allready exist -> delete and add again to update
            listQueue = list(
                self.missionCollection.find(
                    {"name": firstMission["name"], "map": curMap}
                )
            )
            if len(listQueue) != 0:
                print_info(
                    'Mission "{}" allready exist -> Update'.format(
                        firstMission["name"]
                    )
                )
                mission = listQueue[0]
                myQuery = {"_id": mission["_id"]}
                self.missionCollection.delete_one(myQuery)
            self.missionCollection.insert_one(firstMission)

        # Add to queue
        missionQueue = {
            "name": firstMission["name"],
            "desc": firstMission["desc"],
            "time": datetime.datetime.now(),
        }
        self.queueCollection.insert_one(missionQueue)

    """
     #######  ##     ## ######## ##     ## ########    ##     ## ####  ######   ######  ####  #######  ##    ##
    ##     ## ##     ## ##       ##     ## ##          ###   ###  ##  ##    ## ##    ##  ##  ##     ## ###   ##
    ##     ## ##     ## ##       ##     ## ##          #### ####  ##  ##       ##        ##  ##     ## ####  ##
    ##     ## ##     ## ######   ##     ## ######      ## ### ##  ##   ######   ######   ##  ##     ## ## ## ##
    ##  ## ## ##     ## ##       ##     ## ##          ##     ##  ##        ##       ##  ##  ##     ## ##  ####
    ##    ##  ##     ## ##       ##     ## ##          ##     ##  ##  ##    ## ##    ##  ##  ##     ## ##   ###
     ##### ##  #######  ########  #######  ########    ##     ## ####  ######   ######  ####  #######  ##    ##
    """

    def getQueueMission(self, curPos="", curMap="", checkRobotInPoseWhenStart = False):
        """Get first mission in Queue Collection

        Args:
            curPos (String): Name of current position
            curMap (String): Name of current map

        Returns:
            BSON: All mission detail
        """
        print(f"getQueueMission call: {curMap}")
        self.curPos = curPos
        self.listActions = list(self.actionsCollection.find())
        self.listDataList = list(self.dataListCollection.find())
        self.listMarkerType = list(self.markerTypeCollection.find())
        self.listWaypoints = list(self.waypointsCollection.find())
        self.listQueue = list(self.queueCollection.find())
        if len(self.listQueue) == 0:
            return {}
        missionName = self.listQueue[0]["name"]
        self.listMission = list(
            self.missionCollection.find({"name": missionName, "map": curMap})
        )
        # printJson(listMarkerType)
        firstMission = self.listMission[0]
        convertedMission = firstMission
        size = len(convertedMission["actions"])
        start_path = ""
        old_goal = ""
        for i in range(0, size):
            if old_goal == "":
                start_path = self.curPos
            else:
                start_path = old_goal
            print_debug("Action type: {}".format(convertedMission["actions"][i]["type"]))
            if (
                convertedMission["actions"][i]["type"] == "move"
                or convertedMission["actions"][i]["type"]
                == "move_to_position_mbf"
            ):
                print_debug("action_moving")
                if self.curPos == "" and checkRobotInPoseWhenStart:
                    print_error("Robot is not in a defined pose")
                    return "ROBOT_NOT_IN_POSE"
                convertedMission["actions"][i] = self.getMovingAction(
                    firstMission["actions"][i], start_path, curMap
                )
                old_goal = convertedMission["actions"][i]["waypoints"][-1]["name"]
                if convertedMission["actions"][i] == "NO_ROUTE":
                    return "NO_ROUTE"
            elif convertedMission["actions"][i]["type"] == "follow_path":
                convertedMission["actions"][i] = self.getMovingAction(
                    firstMission["actions"][i], start_path, curMap
                )
                old_goal = convertedMission["actions"][i]["waypoints"][-1]["name"]
                if convertedMission["actions"][i] == "NO_ROUTE":
                    return "NO_ROUTE"
            else:
                convertedMission["actions"][i] = self.getActionsDetail(
                    firstMission["actions"][i], curMap
                )
        # print_info("Final mission: \n{}\n".format(dumps(convertedMission["actions"], indent=2)))
        return convertedMission

    def updateQueueMission(self, progress, status):
        """Update first mission in Queue

        Args:
            progress (String): Finished actions/Total actions (Eg.: "3/7")
            status (String): Status of mission ("Doing", "Error" ....)
        Returns:
            Don't return any value
        """
        self.listQueue = list(self.queueCollection.find())
        if len(self.listQueue) == 0:
            return
        mission = self.listQueue[0]
        newvalues = {"$set": {"progress": progress, "status": status}}
        self.queueCollection.update_one({"_id": mission["_id"]}, newvalues)

    def deleteQueueMission(self):
        """Delete first mission in Queue (When finish mission)"""
        self.listQueue = list(self.queueCollection.find())
        if len(self.listQueue) == 0:
            return
        mission = self.listQueue[0]
        myQuery = {"_id": mission["_id"]}
        self.queueCollection.delete_one(myQuery)

    def emptyQueueMission(self):
        """Delete all mission in queue"""
        self.queueCollection.drop()

    def newMissionQueue(self, data):
        """Insert mission to mission queue

        Args:
            data (dict): Mission data
                name: mission name
                time: current time
        """
        self.queueCollection.insert_one(data)

    def downloadFile(self, fileName, filePath, type):
        """Download file to local

        Args:
            fileName (String): name of file in mongodb server
            filePath (String): path where file was saved (Include file name)
            type (String): Type of file in mongodb server ("map", "sound, ...)
        """
        file = None
        try:
            for grid_data in self.fs.find({"filename": fileName, "type": type}):
                fileID = grid_data._id
                file = open(filePath, "wb")
                self.fs.download_to_stream(fileID, file)
                file.close()
                print_info(
                    'Download file done: "{}" to {}'.format(fileName, filePath)
                )
        except Exception as e:
            if file != None:
                file.close()
            print_info('Download file "{}: {}" error'.format(type, fileName))

    def downloadYamlMapFile(self, mapName, folderPath, imgFileType):
        """Download yaml file from map data in mongodb

        Args:
            fileName (String): name of map file in mongodb
            folderPath (String): Save folder (Don't include "/" at the end)
        """
        file = None
        resolution = -1
        try:
            # if True:
            fileName = folderPath + "/" + mapName + ".yaml"
            file = open(fileName, "w")
            print_debug(mapName)
            map = self.mapsCollection.find_one({"name": mapName})
            if map == None:
                print_error('DB has no map: "{}"'.format(fileName))
                return {}
            yamlData = {}
            yamlData["image"] = mapName + ".{}".format(imgFileType)
            if "info" in map:
                resolution = map["info"]["resolution"]
                x = map["info"]["origin"]["position"]["x"]
                y = map["info"]["origin"]["position"]["y"]
            else:
                resolution = 0.05
                x = 0.0
                y = 0.0
            yamlData["resolution"] = resolution
            # yamlData["origin"] = [x, y, 0.0]
            yamlData["origin"] = "[{}, {}, {}]".format(x, y, 0.0)
            yamlData["negate"] = 0
            yamlData["free_thresh"] = 0.195
            yamlData["occupied_thresh"] = 0.65
            dataYaml = "image: {}\nresolution: {}\norigin: [{}, {}, {}]\nnegate: 0\nfree_thresh: 0.195\noccupied_thresh: 0.65".format(
                yamlData["image"], resolution, x, y, 0.0
            )
            # yaml.dump(yamlData, file, allow_unicode=True)
            file.write(dataYaml)
            file.close()
            print_info("Downloaded YAML file: {}".format(mapName))
            return map
        except:
            if file != None:
                file.close()
            print_error('Download YAML file "{}" error'.format(mapName))
        return {}

    def downloadMap(self, mapName, folderPath, imgYamlFileType):
        """Download map file (png and yaml file) from mongodb server to local

        Args:
            mapName (String): Name of map file in mongodb
            folderPath (String): Folder for save file (Don't include "/" at the end)
        """
        filePngPath = folderPath + "/" + mapName + ".png"
        self.downloadFile(mapName, filePngPath, "map")
        self.downloadFiducialMapFile(mapName, folderPath)
        mapInfo = self.downloadYamlMapFile(mapName, folderPath, imgYamlFileType)
        return mapInfo

    def downloadFiducialMapFile(self, fileName, folderPath):
        """Download Fiducial file from map data in mongodb

        Args:
            fileName (String): name of map file in mongodb
            folderPath (String): Save folder (Don't include "/" at the end)
        """
        file = None
        try:
            file = open(folderPath + "/" + fileName + ".json", "w")
            map = self.mapsCollection.find_one({"name": fileName})
            if "fiducial" in map:
                listFiducial = map["fiducial"]
            else:
                listFiducial = []
            json.dump(listFiducial, file, indent=4)
            # file.write(dataYaml)
            file.close()
        except:
            if file != None:
                file.close()
            print_info('Download fiducial file "{}" error'.format(fileName))

    def getCurrentPos(self, pose, currentMap="", accuracy=0.5):
        """Get current position name

        Args:
            pose (geometry_msgs/Pose): Current position
            currentMap (string): Name of current map
            accuracy (float, optional): Accuracy distance (meter, ±accuracy). Defaults to 0.5.

        Returns:
            {"name": $name, "type": $type}: $name: name of pos. Type: "position" or marker base. If fail, return {}
        """
        ori = euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        x = pose.position.x
        y = pose.position.y
        angle = ori[2]
        ret = {}
        if True:
            # Get marker trước vì nếu có marker trùng với position mà marker cần
            # undock
            marker_cnt = 0
            print("maker")
            listMarker = list(
                self.dataListCollection.find(
                    {"type": "marker", "map": currentMap}
                )
            )
            for marker in listMarker:
                tempX = marker["data"]["position"]["x"]
                tempY = marker["data"]["position"]["y"]
                if (
                    x < (tempX + accuracy)
                    and x > (tempX - accuracy)
                    and y < (tempY + accuracy)
                    and y > (tempY - accuracy)
                ):
                    marker_cnt += 1
                    ret["name"] = marker["name"]
                    markerType = self.markerTypeCollection.find_one(
                        {"name": marker["type_detail"]}
                    )
                    ret["type"] = markerType["type"]
                    marker_pose = dict_to_obj(marker["data"], Pose())
                    diff_angle = angle_two_pose(pose, marker_pose)
                    ret["diff_angle"] = diff_angle
                if marker_cnt > 1:
                    ret[
                        "info"
                    ] = "duplicate_marker"  # For prevent undocking if there are multi points near each other
                    return ret
            if "name" in ret.keys():
                return ret
            print("done marker")
            # Pose list
            listPos = list(
                self.dataListCollection.find(
                    {"type": "position", "map": currentMap}
                )
            )
            # print(f"list_pose:{listPos}")
            for pos in listPos:
                is_start_point = False
                if "properties" in pos["data"]:
                    if "Start_point" in pos["data"]["properties"]:

                        start_point = pos["data"]["properties"]["Start_point"]
                        # Start_point may be stored as bool or string ("true"/"false")
                        if isinstance(start_point, str):
                            is_start_point = start_point.strip().lower() in ("true", "1", "yes")
                        else:
                            is_start_point = bool(start_point)
                        print_debug("is start_point: {}".format(pos["name"]))
                tempX = pos["data"]["position"]["x"]
                tempY = pos["data"]["position"]["y"]
                tempori = euler_from_quaternion(
                    [
                        pos["data"]["orientation"]["x"],
                        pos["data"]["orientation"]["y"],
                        pos["data"]["orientation"]["z"],
                        pos["data"]["orientation"]["w"],
                    ]
                )
                temp_angle = tempori[2]
                diff_angle = normalize_angle(temp_angle - angle)
                print(pos["name"],
                    "start:", is_start_point,
                    "diff_angle:", diff_angle,
                )
                if (
                    x < (tempX + accuracy)
                    and x > (tempX - accuracy)
                    and y < (tempY + accuracy)
                    and y > (tempY - accuracy)
                    and is_start_point
                    and abs(diff_angle) < math.pi/4
                ):
                    ret["name"] = pos["name"]
                    ret["type"] = "position"
            print("done pose")
            if "name" in ret.keys():
                return ret
        return ret

    def downloadSetting(
        self, settingName, rootpath=expanduser("~") + "/catkin_ws/src"
    ):
        """Download setting from mongodb to local file

        Args:
            settingName (String): Name of setting
            rootpath (String, optional): Root folder for save file. Detail path will read from mongodb. Defaults to expanduser("~")+"/catkin_ws/src".

        Returns:
            bool: True if success, other: False
        """
        settingJson = self.settingCollection.find_one({"name": settingName})
        if not "path" in settingJson.keys():
            return False
        if settingJson["path"] == "":
            return False
        filePath = rootpath + settingJson["path"]
        file = open(filePath, "w")
        # self.printJson(settingJson)
        try:
            fileData = {}
            fileData["params"] = {}
            for item in settingJson["params"].keys():
                if "params" in settingJson["params"][item]:
                    subJson = settingJson["params"][item]["params"]
                    fileData["params"][item] = {}
                    for subItem in subJson.keys():
                        fileData["params"][item][subItem] = subJson[subItem][
                            "value"
                        ]
                else:
                    fileData["params"][item] = settingJson["params"][item][
                        "value"
                    ]
            self.printJson(fileData)
            json.dump(fileData, file, indent=2)
            file.close()
            return True
        except:
            file.close()
            print_error("Error download setting")
            return False

    def getSetting(self, settingName):
        settingJson = self.settingCollection.find_one({"name": settingName})
        settingData = {}
        if settingJson == None:
            return settingData
        for item in settingJson["params"].keys():
            if "params" in settingJson["params"][item]:
                subJson = settingJson["params"][item]["params"]
                settingData[item] = {}
                for subItem in subJson.keys():
                    settingData[item][subItem] = subJson[subItem]["value"]
            else:
                settingData[item] = settingJson["params"][item]["value"]
        return settingData

    def getPosCoordinate(self, name, map):
        """Get position coordinatecalcWaypoints

        Args:
            name (String): Name of position
            map (String): Name of map

        Returns:
            None: if cannot find.
            json:
                "name": position name
                "type": position type ("position" or "marker")
                "type_detail" (only when "type" = "marker"): Marker name
                "map": map name
                "data":
                    "position":
                        "x": x position
                        "y": y position
                        "z": z position
                    "orientation":
                        "x","y","z","w": quaternion
        """
        ret = self.dataListCollection.find_one(
            {"name": name, "type": "position", "map": map}
        )
        if ret != None:
            return ret
        ret = self.dataListCollection.find_one(
            {"name": name, "type": "marker", "map": map}
        )
        return ret

    def getCurrentMap(self):
        """Get current map name

        Returns:
            None: if cannot find
            String: Name of map
        """
        result = self.dataListCollection.find_one(
            {"name": "current_map", "type": "current_map"}
        )
        if result != None:
            return result["data"]
        return None

    def setCurrentMap(self, mapname):
        self.dataListCollection.update_one(
            {"name": "current_map", "type": "current_map"},
            {"$set": {"data": mapname}},
            upsert=True,
        )

    def recordLog(self, msg, node_name, level):
        data = {
            "time": datetime.datetime.utcnow(),
            "node": node_name,
            "level": level,
            "msg": msg,
        }
        self.logCollection.insert_one(data)

    def recordMissionLog(self, key, name, status):
        findMission = self.missionLogCollection.find_one({"key": key})
        if findMission == None:
            data = {
                "start_time": datetime.datetime.now(),
                "end_time": "",
                "key": key,
                "name": name,
                "status": status,
                "is_deleted": False,
            }
            self.missionLogCollection.insert_one(data)
        else:
            self.missionLogCollection.update_one(
                {"key": key},
                {
                    "$set": {
                        "end_time": datetime.datetime.now(),
                        "status": status,
                    }
                },
            )

    def saveOdom(self, wheelList, dataList):
        """_summary_

        Args:
            wheelList (list): Eg ["left", "right"]
            dataList (list): Eg [1, 2]
        """
        findData = self.dataLogCollection.find_one({"type": "odom_wheel"})
        if findData == None:
            self.makeNewOdomRecord(wheelList)
        else:
            data = {}  # out: {}
            data["$set"] = {}  # out : {'$set': {}}
            for i in range(len(wheelList)):
                data["$set"][wheelList[i]] = dataList[i]
            self.dataLogCollection.update_one({"type": "odom_wheel"}, data)

    def loadOdom(self, wheelList):
        """_summary_

        Args:
            wheelList (list): List of wheel. Eg: ["left", "right"]

        Returns:
            Json string, Eg: "{"type": "odom_wheel", "left": 0, "right": 0}"
        """
        findData = self.dataLogCollection.find_one({"type": "odom_wheel"})
        if findData == None:
            data = self.makeNewOdomRecord(wheelList)
        else:
            data = {"type": "odom_wheel"}
            for w in wheelList:
                data[w] = findData[w]

        return json.dumps(data)

    def makeNewOdomRecord(self, wheelList):
        data = {"type": "odom_wheel"}
        for w in wheelList:
            data[w] = 0
        self.dataLogCollection.insert_one(data)
        return data

    def saveUwbTransform(self, fileName, params):
        """Save Ultra wideband tranform params to map data in mongodb

        Args:
            fileName (String): name of map file in mongodb
            params (dict): params transform ( x_offset, y_offset, yaw)
        """

        map = self.mapsCollection.find_one({"name": fileName})
        if "uwb" not in map:
            print_info("No uwb params in database")

        self.mapsCollection.update_one({"name": fileName}, {"$set": params})
        print_info("Save uwb params in database done")

    def loadUwbTransform(self, fileName):
        """Load Ultra wideband tranform params to map data in mongodb

        Args:
            fileName (String): name of map file in mongodb
        Return:
            tuple (dict): params transform ( x_offset, y_offset, yaw)
        """

        map = self.mapsCollection.find_one({"name": fileName})
        if "uwb" not in map:
            print_warn("No uwb params in database")
        else:
            print_warn("Load uwb params in database done")
            return map["uwb"]

    def saveLift(self, liftCounter, dataLift):
        findData = self.dataLogCollection.find_one({"type": "lifting_cnt"})
        if findData == None:
            self.makeLiftRecord(liftCounter)
        else:
            data = {}  # out: {}
            data["$set"] = {}  # out : {'$set': {}}
            data["$set"][liftCounter] = dataLift
            self.dataLogCollection.update_one({"type": "lifting_cnt"}, data)

    def makeLiftRecord(self, liftCounter):
        data = {"type": "lifting_cnt"}
        data[liftCounter] = 0
        self.dataLogCollection.insert_one(data)
        return data

    def loadLift(self, liftCounter):
        findData = self.dataLogCollection.find_one({"type": "lifting_cnt"})
        if findData == None:
            data = self.makeLiftRecord(liftCounter)
        else:
            data = {"type": "lifting_cnt"}
            data[liftCounter] = findData[liftCounter]
        return json.dumps(data)

    def saveCoilData(self, name, dataCoil):
        findData = self.dataLogCollection.find_one({"type": "coil_orient"})
        if findData == None:
            self.makeNewCoilRecord(name)
        else:
            data = {}  # out: {}
            data["$set"] = {}  # out : {'$set': {}}
            data["$set"][name] = dataCoil
            self.dataLogCollection.update_one({"type": "coil_orient"}, data)

    def makeNewCoilRecord(self, name):
        data = {"type": "coil_orient"}
        data[name] = 0
        self.dataLogCollection.insert_one(data)
        return data

    def loadCoilData(self, name):
        findData = self.dataLogCollection.find_one({"type": "coil_orient"})
        if findData == None:
            data = self.makeNewCoilRecord(name)
        else:
            data = {"type": "coil_orient"}
            data[name] = findData[name]
        return json.dumps(data)

    def saveLastPose(self, params):
        """Save last pose in mongodb

        Args:
            params (dict): pose ( x, y, yaw)
        """
        self.dataLogCollection.update_one({"name": "last_pose"}, {"$set": params})

    def loadLastPose(self):
        """Load last pose in mongodb

        Return:
            tuple (dict): params pose ( x, y, yaw)
        """

        pose_data = self.dataLogCollection.find_one({"name": "last_pose"})
        if pose_data == None:
            data = {"name": "last_pose"}
            data["pose"] = {"position": {"x": 0, "y": 0, "z": 0},"orientation": {"x": 0,"y": 0,"z": 0,"w": 1}}
            self.dataLogCollection.insert_one(data)
            print_warn("not find last pose in database")
            return None
        else:
            return pose_data["pose"]

    def loadRfidData(self, mapName):
        map = self.mapsCollection.find_one({"name": mapName})
        if map == None:
            print_error('DB has no map: "{}"'.format(mapName))
            return None
        else:
            if "rfid" in map:
                return map["rfid"]
            else:
                return None


    def loadRfid2Map(self, mapName, yamlFile):
        yamlFile += ".yaml"
        print_info("RFID file: {}".format(yamlFile))
        if os.path.exists(yamlFile):
            with open(yamlFile) as file:
                rfidDict = yaml.load(file, Loader=yaml.Loader)
                rfidData = []
                for id, pose in rfidDict.items():
                    rfidData.append(
                        {
                            "rfid": str(id),
                            "pose": {
                                "position": {
                                    "x": pose[0]["position"][0],
                                    "y": pose[0]["position"][1],
                                    "z": pose[0]["position"][2],
                                },
                                "orientation": {
                                    "x": pose[1]["orientation"][0],
                                    "y": pose[1]["orientation"][1],
                                    "z": pose[1]["orientation"][2],
                                    "w": pose[1]["orientation"][3],
                                },
                            },
                        }
                    )

                # Tìm bản ghi trong collection
                map = self.mapsCollection.find_one({"name": mapName})

                # Cập nhật trường "rfid" trong map
                if map:
                    map["rfid"] = rfidData
                    self.mapsCollection.replace_one({"_id": map["_id"]}, map)
                    print_info("Uploaded RFID data to map: {}".format(mapName))
                else:
                    print_info("Map not found: {}".format(mapName))
