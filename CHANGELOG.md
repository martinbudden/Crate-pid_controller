# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

Releases of the form `0.1.n` do not adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0.html),
that is each release may contain incompatible API changes.

Once the API has stabilized this project will adopt semantic versioning, the first release to do so will be `0.2.0`.

## [Unreleased]

### Added

### Changed

### Removed

### Deprecated

### Fixed

### Security

## [0.1.5] - 2026-05-16

### Added

- `update_gains` for bumpless changing of gains.

### Changed

- Made limits `Option` type.
- Improved anti-windup handling.
- Constructors changed to `new(gains)` and `with_limits(gains, limits)`.

## Removed

- `integral_threshold` from `PidLimits`.
- `update_spi` function.
- `update_skpi` function
- Individual accessor functions from `PidGains`.
- Individual accessor functions from `PidErrors`.

## [0.1.4] - 2026-05-14

### Changed

- Improved documentation.
- Rearranged modules.
- Consolidated numeric traits.

## [0.1.3] - 2026-05-10

### Added

- Added `set_gains` to `Pid`.
- Added serialization to `PidGains`, `PidLimits`, and `PidErrors`.

## [0.1.2] - 2026-05-06

### Changed

- Updated to signal-filters 0.1.3.

## [0.1.1] - 2026-04-26

### Added

- This changelog.
- CONTRIBUTING.md

## [0.1.0] - 2026-04-12

Initial release.
