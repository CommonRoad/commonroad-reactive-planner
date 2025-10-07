# Minimal Examples
Below are some minimal examples that most users will find sufficient for their puproses


## Run Planner from scenario and config files
```Python
import os
import unittest
from pathlib import Path

# own code base
from commonroad_rp.reactive_planner_easy_api import run_planner

xml_path = Path("PATH/TO/SCENARIO/XML")
path_config = Path("PATH/TO/CONFIG/YAML")
output_path = Path("PATH/TO/OUTPUT/DIR")

state_trajectory, input_trajectory = run_planner(
    scenario_xml_path=xml_path,
    config_path=path_config,
    output_save_path=output_path,
)
```