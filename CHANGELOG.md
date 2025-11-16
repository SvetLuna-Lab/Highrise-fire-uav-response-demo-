# Changelog
All notable changes to this project will be documented in this file.  
This project follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] - 2025-11-16
### Added
- Initial public scaffold of **highrise-fire-uav-response-demo**:
  - `configs/default.yaml`, baseline parameters.
  - `data/`: `building_130f.json`, `wind_profiles.yaml`, scenarios (`case_small.yaml`, `case_multi.yaml`).
  - `src/`: environment (2.5D facade), wind field, allocation (Hungarian), planner (A*), controller, safety checks (geofence/gust/RTL), suppression proxy, simulation loop, metrics, simple matplotlib visualization, CLI `run_scenario.py`.
  - `reports/` artifacts: `mission_log.csv`, `summary.json`, `paths.png`.
  - `tests/`: basic tests for planner, allocation, safety.
  - `Makefile`, `.gitignore`, `README.md`.

### Notes
- Educational simulator; not an engineering guarantee.
- Next steps: hex grid/corridors, refills/multi-goal routes, richer wind/smoke, ROS2/Gazebo bridge.

[0.1.0]: https://github.com/<ORG_OR_USER>/highrise-fire-uav-response-demo/releases/tag/v0.1.0
