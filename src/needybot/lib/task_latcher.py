import rospy

from needybot.control.trees.blackboard import Blackboard 
from needybot.lib.logger import *
from needybot.lib.patterns import Singleton

from needybot_srvs.srv import *


class TaskLatcher(object):

    #__metaclass__ = Singleton

    def __init__(self):
        self.latches = {}

    def add_step_latch_dict(self, step):
        self.latches[step] = {
            'timers': {},
            'subscribers': {}
        }
        return self.latches[step]

    def get_step_dict(self, step_name):
        step_dict = self.latches.get(step_name, None)
        if not step_dict:
            step_dict = self.add_step_latch_dict(step_name)
        return step_dict

    def latch_subscriber(self, step_name, topic, topic_cls, cb):
        step_dict = self.get_step_dict(step_name)

        def handle_topic(msg):
            if self.step == step_name:
                cb(msg)

        subs = step_dict.get('subscribers', {})

        # Shutdown old timer with same ID if it exists
        sub = subs.get(topic, None)
        if sub: sub.unregister()

        sub = rospy.Subscriber(
            topic,
            topic_cls,
            handle_topic
        )

        subs[topic] = sub 

    def latch_timer(self, step_name, tmr_id, duration, cb, oneshot=False):
        step_dict = self.get_step_dict(step_name)

        def handle_evt(evt):
            if self.step == step_name:
                cb(evt)

        timers = step_dict.get('timers', {})
        # Shutdown old timer with same ID if it exists
        timer = timers.get(tmr_id, None)
        if timer and timer.is_alive(): timer.shutdown()

        timer = rospy.Timer(duration, handle_evt, oneshot=oneshot)
        timers[tmr_id] = timer

    def register_task(self):
        self.step = None
        self.latches = {
        }

    def step_task(self, step_name):
        if self.step and self.latches.get(self.step, None):
            step_dict = self.latches.get(self.step, None)

            timers = step_dict.get('timers', {})
            subscribers = step_dict.get('subscribers', {})
            for (tmr_id, timer) in timers.iteritems():
                if timer.is_alive(): timer.shutdown()

            for (sub_id, sub) in subscribers.iteritems():
                sub.unregister()

            del self.latches[self.step] 

        self.step = step_name

    def ulatch_subscriber(self, topic):
        for (step, step_dict) in self.latches.iteritems():
            subs = step_dict.get('subscribers', {})
            sub = subs.get(topic, None)
            if sub:
                sub.unregister()
                del subs[topic] 

    def unlatch_timer(self, tmr_id):
        for (step, step_dict) in self.latches.iteritems():
            timers = step_dict.get('timers', {})
            timer = timers.get(tmr_id, None)
            if timer:
                if timer.is_alive(): timer.shutdown()
                del timers[tmr_id] 

    def unregister(self):
        for (step, step_dict) in self.latches.iteritems():
            timers = step_dict.get('timers', {})
            subs = step_dict.get('subscribers', {})
            if subs:
                for (topic, sub) in subs.iteritems():
                    sub.unregister()
            if timers:
                for (tmr_id, tmr) in timers.iteritems():
                    if tmr.is_alive(): tmr.shutdown()

        self.step = None
        self.latches = {}

