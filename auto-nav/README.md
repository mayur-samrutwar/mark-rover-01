# Autonomous navigation

Status: not yet implemented.

This module will implement autonomous indoor navigation and exploration. Its scope includes sensor integration, localization, mapping, path planning, obstacle avoidance, and motion control.

## Design questions

- Sensor configuration and calibration.
- State estimation and map representation.
- Planning and obstacle-detection methods.
- Command and feedback interfaces with the ESP32 motor controller.
- Pose and timestamp conventions shared with [`../reconstruction/`](../reconstruction/).

Evaluation should distinguish localization accuracy, navigation success, collisions, and trajectory-tracking error. Methods, datasets, and acceptance criteria have not been selected.

The existing manual controller is maintained in [`../wifi-control/`](../wifi-control/).
