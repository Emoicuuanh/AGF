#!/usr/bin/env python3
"""Scene manager for the clearance-field Gazebo bench.

All scene parameters are re-read at 5 Hz, so they can be changed live with rosparam:

  rosparam set /scene_manager/fork_height 1.30             # ramps the lift at ~lift_speed
  rosparam set /scene_manager/stack_height_actual 1.26     # respawns the lower stack
  rosparam set /scene_manager/stack_height_estimate 1.144  # published (latched) on change
  rosparam set /scene_manager/stack_x 3.0                  # world x of the stack front face

Publishes
  lift_command        std_msgs/Float64  -> /agf/lift_controller/command (relative to lift_zero)
  fork_height         std_msgs/Float32  fork-top height above the floor, from joint_states
  lower_stack_height  std_msgs/Float32  estimate, latched, only on change (a new value resets
                                        the clearance node's debouncer)
"""

import threading

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

PAL_L, PAL_W, PAL_H = 1.2, 0.8, 0.144
STACK_MODEL = 'lower_stack'


def _box(name, cx, cz, sx, sy, sz, rgba):
    geom = '<geometry><box><size>%.4f %.4f %.4f</size></box></geometry>' % (sx, sy, sz)
    pose = '<pose>%.4f 0 %.4f 0 0 0</pose>' % (cx, cz)
    return ('<collision name="%s_c">%s%s</collision>'
            '<visual name="%s_v">%s%s<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'
            % (name, pose, geom, name, pose, geom, rgba, rgba))


def stack_sdf(height):
    """Static lower stack: pallet + load, front face at the model origin, total height `height`.

    A contact sensor reports anything touching the stack on /lower_stack/contacts
    (gazebo_msgs/ContactsState) -- the scenario's collision ground truth.
    """
    parts = [_box('pallet', PAL_L / 2, min(height, PAL_H) / 2, PAL_L, PAL_W, min(height, PAL_H),
                  '0.70 0.52 0.30 1')]
    names = ['pallet_c']
    if height > PAL_H + 1e-3:
        load = height - PAL_H
        parts.append(_box('load', PAL_L / 2, PAL_H + load / 2, PAL_L, PAL_W - 0.04, load,
                          '0.80 0.35 0.25 1'))
        names.append('load_c')
    sensor = ('<sensor name="contact" type="contact"><always_on>true</always_on>'
              '<update_rate>50</update_rate><contact>%s</contact>'
              '<plugin name="bumper" filename="libgazebo_ros_bumper.so">'
              '<bumperTopicName>/lower_stack/contacts</bumperTopicName><frameName>world</frameName>'
              '</plugin></sensor>' % ''.join('<collision>%s</collision>' % n for n in names))
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="%s"><static>true</static>'
            '<link name="link">%s%s</link></model></sdf>' % (STACK_MODEL, ''.join(parts), sensor))


class SceneManager(object):
    def __init__(self):
        self.lift_zero = float(rospy.get_param('~lift_zero', 1.40))
        self.lift_speed = float(rospy.get_param('~lift_speed', 0.25))
        self.lift_joint = rospy.get_param('~lift_joint', 'lift_joint')
        self.lift_min = float(rospy.get_param('~fork_min', 0.20))
        self.lift_max = float(rospy.get_param('~fork_max', 2.20))

        self.fork_now = None
        self.fork_cmd = None
        self.last_est = None
        self.spawned = None            # (height, x) of the stack currently in Gazebo
        self.lock = threading.Lock()

        self.pub_cmd = rospy.Publisher('lift_command', Float64, queue_size=1)
        self.pub_fork = rospy.Publisher('fork_height', Float32, queue_size=1)
        self.pub_est = rospy.Publisher('lower_stack_height', Float32, queue_size=1, latch=True)
        rospy.Subscriber('joint_states', JointState, self.on_joints, queue_size=1)

        rospy.loginfo('scene_manager: waiting for gazebo spawn services')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        self.srv_spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.srv_delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.dt = 0.02
        rospy.Timer(rospy.Duration(self.dt), self.on_lift_timer)
        rospy.Timer(rospy.Duration(0.2), self.on_param_timer)

    # ------------------------------------------------------------ lift
    def on_joints(self, msg):
        if self.lift_joint not in msg.name:
            return
        q = msg.position[msg.name.index(self.lift_joint)]
        with self.lock:
            self.fork_now = self.lift_zero + q
        self.pub_fork.publish(Float32(data=self.fork_now))

    def on_lift_timer(self, _evt):
        target = float(rospy.get_param('~fork_height', self.lift_zero))
        target = min(max(target, self.lift_min), self.lift_max)
        with self.lock:
            if self.fork_cmd is None:
                if self.fork_now is None:
                    return
                self.fork_cmd = self.fork_now
            step = self.lift_speed * self.dt
            self.fork_cmd += max(-step, min(step, target - self.fork_cmd))
            cmd = self.fork_cmd
        if not rospy.is_shutdown():
            try:
                self.pub_cmd.publish(Float64(data=cmd - self.lift_zero))
            except rospy.ROSException:      # topic closed during shutdown
                pass

    # ------------------------------------------------------------ estimate + stack
    def on_param_timer(self, _evt):
        est = float(rospy.get_param('~stack_height_estimate', 1.144))
        if est != self.last_est:
            self.last_est = est
            self.pub_est.publish(Float32(data=est))
            rospy.loginfo('scene_manager: stack estimate -> %.3f m', est)

        want = (round(float(rospy.get_param('~stack_height_actual', 1.144)), 4),
                round(float(rospy.get_param('~stack_x', 3.0)), 4))
        if want != self.spawned:
            self.respawn_stack(*want)

    def respawn_stack(self, height, x):
        try:
            self.srv_delete(STACK_MODEL)            # fails harmlessly if it does not exist
        except rospy.ServiceException:
            pass
        pose = Pose(position=Point(x=x, y=0.0, z=0.0), orientation=Quaternion(w=1.0))
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown():
            try:
                res = self.srv_spawn(STACK_MODEL, stack_sdf(height), '', pose, 'world')
            except rospy.ServiceException as e:
                res = None
                rospy.logwarn('scene_manager: spawn failed: %s', e)
            if res is not None and res.success:
                self.spawned = (height, x)
                rospy.loginfo('scene_manager: lower stack %.3f m high, front face at x=%.3f', height, x)
                return
            if rospy.Time.now() > deadline:
                rospy.logerr('scene_manager: could not spawn lower stack: %s',
                             res.status_message if res else 'service error')
                self.spawned = (height, x)          # do not retry every cycle
                return
            rospy.sleep(0.2)                        # delete is asynchronous


def main():
    rospy.init_node('scene_manager')
    SceneManager()
    rospy.spin()


if __name__ == '__main__':
    main()
