from commonroad_rp.utility.config import ReactivePlannerConfiguration
from .implementation.sampling_space_fixed import FixedIntervalSampling
from .implementation.sampling_space_corridor import CorridorSampling


def sampling_space_factory(config: ReactivePlannerConfiguration):
    """Factory function to select SamplingSpace class"""
    sampling_method = config.sampling.sampling_method
    if sampling_method == 1:
        return FixedIntervalSampling(config)
    elif sampling_method == 2:
        return CorridorSampling(config)
    else:
        ValueError("Invalid sampling method specified")
