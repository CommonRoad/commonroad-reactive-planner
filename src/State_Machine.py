from transitions import Machine


class CarHighlevelStates(object):

    states =['following', 'lane_change_left']
    transitions = [{'trigger': 'Car_ahead_too_slow',    'source': 'following',          'dest': 'lane_change_left',
                    'before': 'set_lanelet_ids', 'conditions': 'lanelet_ids_valid'},

                   {'trigger': 'on_new_centerline',     'source': 'lane_change_left',   'dest': 'following',
                    'before': 'reset_lanelet_ids'}]

    def __init__(self):

        self.old_lanelet = None
        self.new_lanelet = None

        self.machine = Machine(model=self, states=CarHighlevelStates.states,
                               transitions=CarHighlevelStates.transitions, initial='following')


    def set_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        self.old_lanelet = old_lanelet
        self.new_lanelet = new_lanelet

    def reset_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        self.old_lanelet = None
        self.new_lanelet = None

    def lanelet_ids_valid(self, old_lanelet = None, new_lanelet = None):
        if old_lanelet is not None and new_lanelet is not None:
            return True
        return False
