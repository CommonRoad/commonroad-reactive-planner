import unittest
from pathlib import Path
import os

# own code base
from run_planner import main
from commonroad_rp.utility.config import ReactivePlannerConfiguration


class ExampleTest(unittest.TestCase):
    """
    Test run_planner.py example
    """

    def test_example(self):
        filename = "ZAM_Over-1_1.xml"
        path_config = Path(__file__).parents[1] / "configurations" / f"{filename[:-4]}.yaml"
        config = ReactivePlannerConfiguration.load(path_config, filename)
        config.general.path_output = Path(os.path.abspath(Path(__file__).parents[1])) / "output"
        config.update()

        main(
            config=config
        )


