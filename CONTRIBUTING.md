# Contributing

## Branches

| Branch                                           | Purpose                                                                               |
| ------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `main`                                           | integration branch, always builds and passes CI; the robot runs a tagged commit of it |
| `fix/<topic>`                                    | bug fixes, small and reviewable, merged first                                         |
| `feat/<topic>`                                   | one feature per branch, with tests and docs                                           |
| `refactor/<topic>`, `ci/<topic>`, `docs/<topic>` | no behaviour change (or a documented breaking change)                                 |

Open a pull request into `main`; CI (`.github/workflows/ci.yml`) must be green. Stacked branches
(one built on another) say so in the PR description and are merged in that order. Releases are
tags `vX.Y.Z` on `main` with an entry in [CHANGELOG.md](CHANGELOG.md).

Commits follow [Conventional Commits](https://www.conventionalcommits.org):
`fix(navigation): ...`, `feat(bridge): ...`, `refactor(bringup)!: ...` plus a `BREAKING CHANGE:`
footer when users have to change something.

## Build and test

```bash
scripts/dev.sh up && scripts/dev.sh build && scripts/dev.sh test     # dev container (Humble)
colcon build && colcon test --executor sequential && colcon test-result --verbose   # on the robot
pip install pre-commit && pre-commit install                      # same lint as CI, on commit
```

Every package follows the Dharma layout: pure logic in `<package>/core/` (no ROS imports, unit
tests in `test/unit`), ROS adapters in `<package>/utils/`, thin `*_node.py` files that subclass
`dharma_utils` `DharmaNode` (typed configuration with `declare_config`, the robot profile with
`declare_robot`, `self.log`, `self.tf`, `run()`), node tests in `test/ros` (in-process, fed
with fakes) and `test/test_style.py` (`dharma_utils.testing.check_package`: module size at most
300 lines, tests 400, no ROS imports in `core/`, no shadowed node members, nothing that breaks
on Python 3.8 when ruff targets it). Lint is ruff with the hub's
[ruff.toml](ruff.toml); YAML and Markdown are formatted by prettier. The end-to-end simulator
test in `molly_bringup/test/e2e` takes about two minutes; `MOLLY_SKIP_SIM=1` skips it locally.

## Rules that come from real crashes

- **Never redefine an rclpy `Node` member** (`handle`, `context`, `executor`, `_clock`, ...) in
  a node class; the style test checks it. Do not reuse `DharmaNode`'s own names (`log`, `tf`,
  `every`, `latched`, ...) either: a hooks object's `log()` once replaced the node's logger.
- **One logging call site per severity.** rclpy raises `Logger severity cannot be changed between
calls` when the same line logs once as `info` and once as `warn`; write two calls instead of
  `log = logger.warn if bad else logger.info`.
- **Stop cleanly:** `main()` is `dharma_utils.ros.run(NodeClass)`, which handles Ctrl-C and the
  external shutdown; a node that starts a thread stops it in `destroy_node()`.
- **TF listeners run on the node's own executor:** `TransformListener(buffer, self)`, never
  `spin_thread=True`, which adds the whole node to a second executor.
- **One BLAS thread per process.** pip numpy's OpenBLAS threads busy-wait between the small matrix
  operations nodes run at 10-50 Hz and burn whole cores; `run_robot.sh` and `scripts/env.sh` set
  `OPENBLAS_NUM_THREADS=1`.
- **No per-element Python objects at sensor rate.** rclpy builds one object per element of a
  message array (a Livox CustomMsg has ~20k points): read big messages as numpy straight from the
  buffer (`PointCloud2.data`, or `raw=True` and the CDR bytes, see `dharma_perception.core.livox`).
- **Subscribe to heavy topics only while someone uses them** (the teleop camera and cloud).
- **Validate everything from the network** (teleop UDP, rosbridge requests, site files): clamp
  numbers, reject non-finite values, keep file paths inside their directory, never raise in a
  callback.
- **Parameters** are typed fields of the package's config dataclasses (`core/config.py`, with
  unit, range and doc), listed in the package README; Molly's values for the Lite3 are in
  `molly_bringup/config/stack.yaml`, the robot's body and limits in `robot.yaml`, and a robot or
  site is tuned in `/etc/molly/site.yaml`, never by editing the repository on the robot.
- **The env file** (`/etc/molly/molly.env`) is read by both systemd and bash: no comment
  after a value (systemd makes it part of the value) and quotes around a value with spaces (bash
  runs the second word as a command). `scripts/setup.sh check` flags both.
- **Signal only `ros2 launch`.** Stop the stack with one SIGINT to the launch process (systemd does
  this with `KillMode=mixed`), not to its process group: launch forwards it to every node, and a
  second SIGINT interrupts the nodes in the middle of their shutdown.

## Safety-relevant changes

Anything that changes how the robot moves (speeds, limits, e-stop, avoidance, the control mux, the
teleop gateway) needs a test that drives the logic, a note in the PR on how it was tried on the robot
(slow speed, open area, someone at the e-stop), and a CHANGELOG entry. Try it in the simulator
first (`ros2 launch molly_bringup sim.launch.py rviz:=true`); its end-to-end test (the demo
tour to its end with a handheld takeover through the control mux) runs in CI and must stay
green.
