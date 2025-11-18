# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
## [Unreleased]

## [0.2.0-pre] - 2025-11-17
### Added
- **Tethered nozzle (roof-fed hose)** mode: nozzle model, quasi-static hose approximation, basic safety nudges/checks.
- Scenario: `configs/tethered_case.yaml`.
- Online/time-weighted metrics: `time_on_target_s`, `ir_over_limit_s`, `tension_N_peak`, `min_bend_radius_m`.

### Changed
- **README** updated: scenarios section and quick-run instructions for tethered mode.

## [0.1.0] - 2025-11-16
### Added
- Initial public release: baseline high-rise UAV fire response simulator, smoke tests, and CI.
