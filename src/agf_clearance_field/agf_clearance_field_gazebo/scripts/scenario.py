#!/usr/bin/env python3
"""Scenario runner for the clearance-field Gazebo bench.

For every case in ~cases:
  1. teleport the AGF back to x = 0, set fork height / actual stack / estimated stack
  2. publish the estimate (new job -> clearance node starts BLOCKED)
  3. drive until the LiDAR is ~check_gap from the stack face (beyond field x_max), stop, settle
  4. read the debounced /clearance_field/field_clear (+ status)
  5. compare with the ground truth: carried pallet underside (fork - deck_offset) above the
     ACTUAL stack top?
  6. if the field said CLEAR and ~drive_in: drive to ~place_gap; 'COLLIDED' if the stack's
     contact sensor saw any AGF collision (or the AGF stalled), else 'placed'

A case passes when the field result equals its `expect` value. Cases where the field is
CLEAR but the ground truth is BLOCKED are flagged UNSAFE whatever `expect` says.
"""

import json
import math
import sys

import rospy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64, String


class Scenario(object):
    def __init__(self):
        gp = rospy.get_param
        self.cases = gp('~cases', [])
        self.robot = gp('~robot_model', 'agf')
        self.rail_joint = gp('~rail_joint', 'rail_joint')
        self.scene = gp('~scene_ns', '/scene_manager').rstrip('/')
        self.stack_x = float(gp('~stack_x', 3.0))
        self.check_gap = float(gp('~check_gap', 1.6))
        self.place_gap = float(gp('~place_gap', 0.03))
        self.speed = float(gp('~speed', 0.3))
        self.settle = float(gp('~settle', 1.0))
        self.sample = float(gp('~sample', 0.5))
        self.drive_in = bool(gp('~drive_in', True))
        self.deck = float(gp('~deck_offset', 0.122))
        self.tol = float(gp('/clearance_field/field/tolerance', gp('~tolerance', 0.05)))

        self.fork = None
        self.x = None
        self.clear = None
        self.status = {}
        self.contact = None                             # first AGF collision touching the stack
        rospy.Subscriber('/lower_stack/contacts', ContactsState, self._on_contacts, queue_size=10)
        rospy.Subscriber('fork_height', Float32, self._on_fork, queue_size=1)
        rospy.Subscriber('joint_states', JointState, self._on_joints, queue_size=1)
        rospy.Subscriber('field_clear', Bool, self._on_clear, queue_size=1)
        rospy.Subscriber('field_status', String, self._on_status, queue_size=1)
        self.pub_vel = rospy.Publisher('drive_command', Float64, queue_size=1)
        self.pub_est = rospy.Publisher('lower_stack_height', Float32, queue_size=1, latch=True)
        self.pub_res = rospy.Publisher('~result', String, queue_size=10)

        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.srv_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

    # ------------------------------------------------------------ callbacks
    def _on_joints(self, m):
        if self.rail_joint in m.name:
            self.x = m.position[m.name.index(self.rail_joint)]

    def _on_contacts(self, m):
        if self.contact is not None:
            return
        for s in m.states:
            for c in (s.collision1_name, s.collision2_name):
                if c.startswith(self.robot + '::'):
                    self.contact = c
                    return

    def _on_fork(self, m):
        self.fork = m.data

    def _on_clear(self, m):
        self.clear = m.data

    def _on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    # ------------------------------------------------------------ motion
    def stop(self):
        for _ in range(3):
            self.pub_vel.publish(Float64(data=0.0))
            rospy.sleep(0.05)

    def reset_robot(self):
        """Teleport the AGF back to the start of the rail (LiDAR at world x = 0)."""
        self.stop()
        self.srv_config(self.robot, 'robot_description', [self.rail_joint], [0.0])
        rospy.sleep(0.5)

    def drive_to_gap(self, target, timeout=60.0):
        """Drive forward until stack_x - robot_x <= target. Returns False if stuck/timeout."""
        rate = rospy.Rate(20)
        t_end = rospy.Time.now() + rospy.Duration(timeout)
        best, t_best = float('inf'), rospy.Time.now()
        ok = False
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            gap = self.stack_x - self.x
            if gap <= target + 0.002:
                ok = True
                break
            if gap < best - 0.005:
                best, t_best = gap, rospy.Time.now()
            elif (rospy.Time.now() - t_best).to_sec() > 2.0:
                break                                   # no progress -> stuck
            self.pub_vel.publish(Float64(data=min(self.speed, max(0.03, 1.5 * (gap - target)))))
            rate.sleep()
        self.stop()
        return ok

    def wait_fork(self, target, timeout=20.0):
        t_end = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            if self.fork is not None and abs(self.fork - target) < 0.003:
                return True
            rospy.sleep(0.05)
        rospy.logwarn('scenario: fork did not reach %.3f (now %s)', target, self.fork)
        return False

    # ------------------------------------------------------------ case
    def run_case(self, c):
        name = c.get('name', 'case')
        fork, actual = float(c['fork']), float(c['actual'])
        est = float(c.get('estimate', actual))
        rospy.loginfo('==== %s: fork %.3f  stack actual %.3f  estimate %.3f', name, fork, actual, est)

        self.reset_robot()
        rospy.set_param(self.scene + '/stack_x', self.stack_x)
        rospy.set_param(self.scene + '/stack_height_actual', actual)
        rospy.set_param(self.scene + '/stack_height_estimate', est)
        rospy.set_param(self.scene + '/fork_height', fork)
        rospy.sleep(1.0)                                # scene_manager polls at 5 Hz + respawn
        self.wait_fork(fork)
        self.pub_est.publish(Float32(data=est))         # new job -> debouncer reset

        if not self.drive_to_gap(self.check_gap):
            rospy.logwarn('scenario: did not reach the check position')
        rospy.sleep(self.settle)
        self.clear, t_end = None, rospy.Time.now() + rospy.Duration(self.sample)
        while not rospy.is_shutdown() and (self.clear is None or rospy.Time.now() < t_end):
            rospy.sleep(0.02)
        field_clear, status = bool(self.clear), dict(self.status)

        bottom = fork - self.deck
        truth_clear = bottom > actual
        outcome = 'held'
        if field_clear and self.drive_in:
            self.contact = None
            reached = self.drive_to_gap(self.place_gap, timeout=30.0)
            outcome = 'COLLIDED' if self.contact or not reached else 'placed'
            if self.contact:
                rospy.logwarn('scenario: %s touched the lower stack', self.contact)

        r = {
            'name': name, 'fork': fork, 'actual': actual, 'estimate': est,
            'field_top': round(est + self.tol, 4), 'carried_bottom': round(bottom, 4),
            'field': 'CLEAR' if field_clear else 'BLOCKED',
            'truth': 'CLEAR' if truth_clear else 'BLOCKED',
            'count': status.get('count'), 'lowest_hit_z': status.get('lowest_hit_z'),
            'reason': status.get('reason'), 'outcome': outcome,
            'expect': str(c.get('expect', '')).upper() or None,
        }
        r['unsafe'] = field_clear and not truth_clear
        r['pass'] = r['expect'] is None or r['expect'] == r['field']
        self.pub_res.publish(String(data=json.dumps(r)))
        rospy.loginfo('---- %s: field %s (count %s, lowest %s), truth %s, outcome %s -> %s%s',
                      name, r['field'], r['count'], _f(r['lowest_hit_z']), r['truth'], outcome,
                      'PASS' if r['pass'] else 'FAIL', '  [UNSAFE]' if r['unsafe'] else '')
        return r

    def run(self):
        if not self.cases:
            rospy.logerr('scenario: ~cases is empty (load config/scenarios.yaml)')
            return 2
        rospy.loginfo('scenario: waiting for /clearance_field, fork_height and joint_states')
        while not rospy.is_shutdown() and None in (self.fork, self.clear, self.x):
            rospy.sleep(0.1)
        results = [self.run_case(c) for c in self.cases if not rospy.is_shutdown()]
        self.reset_robot()

        hdr = '%-24s %6s %6s %6s %6s %6s  %-7s %-7s %5s %-9s %s'
        lines = [hdr % ('case', 'fork', 'act', 'est', 'top', 'bottom', 'field', 'truth', 'pts',
                        'outcome', 'result'),
                 '-' * 104]
        for r in results:
            lines.append(hdr % (r['name'][:24], '%.3f' % r['fork'], '%.3f' % r['actual'],
                                '%.3f' % r['estimate'], '%.3f' % r['field_top'],
                                '%.3f' % r['carried_bottom'], r['field'], r['truth'],
                                r['count'] if r['count'] is not None else '-', r['outcome'],
                                ('PASS' if r['pass'] else 'FAIL') + (' UNSAFE' if r['unsafe'] else '')))
        n_fail = sum(not r['pass'] for r in results)
        lines.append('%d/%d cases matched expectation' % (len(results) - n_fail, len(results)))
        rospy.loginfo('scenario summary\n%s', '\n'.join(lines))
        return 1 if n_fail else 0


def _f(v):
    return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else '%.3f' % v


def main():
    rospy.init_node('clearance_scenario')
    code = Scenario().run()
    rospy.sleep(0.5)
    sys.exit(code)


if __name__ == '__main__':
    main()
