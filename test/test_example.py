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
        xml = Path(__file__).parents[1] / "example_scenarios" / filename
        config_path = Path(__file__).parents[1] / "configurations" / f"{filename[:-4]}.yaml"
        config = ReactivePlannerConfiguration.load_from_xml_and_yaml(xml, config_path)
        config.general.path_output = Path(os.path.abspath(Path(__file__).parents[1])) / "output"
        config.update()

        main(
            config=config
        )


