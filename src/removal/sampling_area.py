from typing import List


class SamplingArea:
    def __init__(self, s_min: float, s_max: float):
        self.s_min = s_min
        self.s_max = s_max

    @classmethod
    def safe_region_for_coordinate_system(cls, lanelet_ids: List):
        # TODO: calculate min_s and max_s with safe regions from Christian
        return cls(39, 79)  # only for lanelet id 538!!!
