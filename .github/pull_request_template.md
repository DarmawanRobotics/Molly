## What and why

<!-- One or two sentences. Link the issue or field report. -->

## How it was tested

- [ ] CI green (lint, Humble build and tests of the whole stack from `molly.repos`)
- [ ] Tried in the simulator (`ros2 launch molly_bringup sim.launch.py`), in the dev container or
      on the robot: <!-- what you ran and saw -->

## Checklist

- [ ] Conventional commit titles (`fix(guide): ...`, `feat(bridge): ...`)
- [ ] Parameters and launch arguments documented (package README, `config/*.yaml`)
- [ ] CHANGELOG entry for user-visible changes
- [ ] Safety-relevant change (motion, e-stop, limits, robot profile)? Described above and tested
      on the robot
