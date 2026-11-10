#!/usr/bin/env python
"""
Binary WebSocket server for /map OccupancyGrid.
Convert OccupancyGrid messages to a compact binary format and broadcast via WebSocket.
Protocol (little-endian):
  [0:4]   int32  width
  [4:8]   int32  height
  [8:12]  float32 resolution
  [12:16] float32 origin.x
  [16:20] float32 origin.y
  [20:24] float32 origin.yaw  (extracted from quaternion)
  [24:]   int8[] data  (width * height bytes)
"""
import asyncio
import struct
import math
import threading
import rospy
from nav_msgs.msg import OccupancyGrid
import websockets

_latest_payload = None
_clients = set()
_lock = threading.Lock()

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def map_callback(msg):
    global _latest_payload
    info = msg.info
    yaw = quat_to_yaw(info.origin.orientation)
    header = struct.pack('<iiffff',
        info.width,
        info.height,
        info.resolution,
        info.origin.position.x,
        info.origin.position.y,
        yaw,
    )
    # data is int8[] — pack as signed bytes
    data_bytes = bytes([b & 0xFF for b in msg.data])
    payload = header + data_bytes
    with _lock:
        _latest_payload = payload
    # broadcast to all connected clients
    asyncio.run_coroutine_threadsafe(_broadcast(payload), _loop)

async def _broadcast(payload):
    if _clients:
        await asyncio.gather(
            *[ws.send(payload) for ws in list(_clients)],
            return_exceptions=True
        )

async def handler(websocket, path):
    _clients.add(websocket)
    rospy.loginfo(f'[map_binary_ws] Client connected: {websocket.remote_address}')
    try:
        # send latest map immediately on connect
        with _lock:
            if _latest_payload:
                await websocket.send(_latest_payload)
        await websocket.wait_closed()
    finally:
        _clients.discard(websocket)
        rospy.loginfo(f'[map_binary_ws] Client disconnected')

_loop = asyncio.new_event_loop()

def start_ws_server():
    asyncio.set_event_loop(_loop)
    start_server = websockets.serve(handler, '0.0.0.0', 9092)
    _loop.run_until_complete(start_server)
    rospy.loginfo('[map_binary_ws] Listening on ws://0.0.0.0:9092')
    _loop.run_forever()

if __name__ == '__main__':
    rospy.init_node('map_binary_ws_server')
    ws_thread = threading.Thread(target=start_ws_server, daemon=True)
    ws_thread.start()
    rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
    rospy.loginfo('[map_binary_ws] Subscribed to /map')
    rospy.spin()