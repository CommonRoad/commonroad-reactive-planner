import os
import unittest
from pathlib import Path

# own code base
from commonroad_rp.reactive_planner_easy_api import run_planner


class EasyAPITest(unittest.TestCase):
    """
    Tests easy API
    """
    def test_run_planner(self):
        not_working = [
            "ZAM-Ramp-1_1-T-1.xml",
        ]

        for file in sorted(os.listdir(Path(__file__).parents[1] / "example_scenarios")):
            filename = file.split('.')[0] + ".xml"
            if filename in not_working:
                continue

            with self.subTest(msg=f"Testing scenario {filename}"):
                xml_path = Path(__file__).parents[1] / "example_scenarios" / filename
                path_config = Path(__file__).parents[1] / "configurations" / f"{filename[:-4]}.yaml"
                output_path = Path(__file__).parents[1] / "output" / filename.split(".")[0]

                state_trajectory, input_trajectory = run_planner(
                    scenario_xml_path=xml_path,
                    config_path=path_config,
                    output_save_path=output_path,
                )



