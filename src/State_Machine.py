from transitions import Machine
import threading
import time


class CarHighlevelStates(object):

    states =['following', 'lane_change_left', 'lane_change_right', 'on_overtaking_line']
    transitions = [{'trigger': 'Car_ahead_too_slow',    'source': 'following',          'dest': 'lane_change_left',
                    'before': ['set_overtaking_true', 'set_lanelet_ids'], 'conditions': 'lanelet_ids_valid'},

                   #{'trigger': 'on_new_centerline',     'source': 'lane_change_left',   'dest': 'on_overtaking_line',
                   # 'before': 'set_timer_overtaking', 'conditions': 'overtaking_is_true'},

                   {'trigger': 'on_new_centerline',     'source': 'lane_change_left',   'dest': 'following',
                    'before': 'reset_lanelet_ids', 'after': 'set_overtaking_false'},

                   {'trigger': 'timer_elapsed',         'source': 'on_overtaking_line', 'dest': 'lane_change_right',
                    'before': 'set_lanelet_ids'},

                   {'trigger': 'on_new_centerline',     'source': 'lane_change_right',  'dest': 'following',
                    'after': ['reset_lanelet_ids', 'cancel_timer', 'set_overtaking_false']},

                   {'trigger': 'goal_region_left',      'source': 'following',          'dest': 'lane_change_left',
                    'before': 'reset_lanelet_ids'},

                   {'trigger': 'goal_region_right',     'source': 'following',          'dest': 'lane_change_right',
                    'before': 'reset_lanelet_ids'}]

    def __init__(self):

        self.old_lanelet = None
        self.new_lanelet = None

        self.overtaking = None

        self.time_for_overtaking = None
        self.timer = None
        self.timer_elapsed = None

        self.starttime = None

        self.machine = Machine(model=self, states=CarHighlevelStates.states,
                               transitions=CarHighlevelStates.transitions, initial='following')

    def overtaking_is_true(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        if self.overtaking:
            return True
        else:
            return False

    def set_overtaking_true(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.overtaking = True

    def set_overtaking_false(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.overtaking = False


    def set_lanelet_ids(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.old_lanelet = old_lanelet
        self.new_lanelet = new_lanelet

    def reset_lanelet_ids(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.old_lanelet = None
        self.new_lanelet = None

    def lanelet_ids_valid(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        if old_lanelet is not None and new_lanelet is not None:
            return True
        return False


    def set_timer_elapsed(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):

        if time_for_overtaking <= time.time() - self.starttime:
            self.timer_elapsed = True

    def set_timer_overtaking(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.starttime = time.time()

    def cancel_timer(self, old_lanelet = None, new_lanelet = None, time_for_overtaking = None):
        self.timer.cancel()
