# Contributing

## Branches

| Branch | Purpose |
|---|---|
| `main` | integration branch, always builds and passes CI; the robot runs a tagged commit of it |
| `fix/<topic>` | bug fixes, small and reviewable, merged first |
| `feat/<topic>` | one feature per branch, with tests and docs |
| `refactor/<topic>`, `ci/<topic>`, `docs/<topic>` | no behaviour change (or a documented breaking change) |

Open a pull request into `main`; CI (`.github/workflows/ci.yml`) must be green. Stacked branches
(one built on another) say so in the PR description and are merged in that order. Releases are
tags `vX.Y.Z` on `main` with an entry in [CHANGELOG.md](CHANGELOG.md).

Commits follow [Conventional Commits](https://www.conventionalcommits.org):
`fix(navigation): ...`, `feat(bridge): ...`, `refactor(bringup)!: ...` plus a `BREAKING CHANGE:`
footer when users have to change something.

## Build and test

```bash
script/dev.sh up && script/dev.sh build && script/dev.sh test     # dev container (macOS/Linux)
colcon build && colcon test --executor sequential && colcon test-result --verbose   # on the robot
pip install pre-commit && pre-commit install                      # same lint as CI, on commit
```

Every Python package runs `ament_flake8`, `ament_pep257` and its own tests in `colcon test`. Tests
that need ROS (an in-process node fed with fake data) skip themselves when `rclpy` is missing, so
pure-logic tests also run with plain `pytest` on a laptop.

## Rules that come from real crashes

- **Never redefine an rclpy `Node` member** (`handle`, `context`, `executor`, `_clock`, ...) in a
  node class. `test/test_node_members.py` in every package enforces it.
- **One logging call site per severity.** rclpy raises `Logger severity cannot be changed between
  calls` when the same line logs once as `info` and once as `warn`; write two calls instead of
  `log = logger.warn if bad else logger.info`.
- **Stop cleanly:** `main()` catches `KeyboardInterrupt` and `ExternalShutdownException` and wraps
  `rclpy.shutdown()` in `try/except`; a node that starts a thread stops it in `destroy_node()`.
  A Ctrl-C can land while the executor builds its wait set, so any other
  exception from `spin()` is a normal stop when `rclpy.ok()` is already false; re-raise it only
  while `rclpy.ok()` is true. Copy the `main()` of an existing node.
- **TF listeners run on the node's own executor:** `TransformListener(buffer, self)`, never
  `spin_thread=True`, which adds the whole node to a second executor.
- **One BLAS thread per process.** pip numpy's OpenBLAS threads busy-wait between the small matrix
  operations nodes run at 10-50 Hz and burn whole cores; `run_robot.sh` and `script/env.sh` set
  `OPENBLAS_NUM_THREADS=1`.
- **No per-element Python objects at sensor rate.** rclpy builds one object per element of a
  message array (a Livox CustomMsg has ~20k points): read big messages as numpy straight from the
  buffer (`PointCloud2.data`, or `raw=True` and the CDR bytes, see `molly_perception/livox_cdr.py`).
- **Subscribe to heavy topics only while someone uses them** (the teleop camera and cloud).
- **Validate everything from the network** (teleop UDP, rosbridge requests, site files): clamp
  numbers, reject non-finite values, keep file paths inside their directory, never raise in a
  callback.
- **Parameters** live in the package `config/*.yaml` with a comment, are listed in the package
  README, and are tuned per robot in `/etc/molly/site.yaml`, never by editing the repository on
  the robot.
- **The env file** (`/etc/molly/molly.env`) is read by both systemd and bash: no comment
  after a value (systemd makes it part of the value) and quotes around a value with spaces (bash
  runs the second word as a command). `script/setup.sh check` flags both.
- **Signal only `ros2 launch`.** Stop the stack with one SIGINT to the launch process (systemd does
  this with `KillMode=mixed`), not to its process group: launch forwards it to every node, and a
  second SIGINT interrupts the nodes in the middle of their shutdown.

## Safety-relevant changes

Anything that changes how the robot moves (speeds, limits, e-stop, avoidance, the control mux, the
teleop gateway) needs a test that drives the logic, a note in the PR on how it was tried on the robot
(slow speed, open area, someone at the e-stop), and a CHANGELOG entry. Try it in the simulator
first (`ros2 launch molly_sim sim.launch.py rviz:=true`); its end-to-end tests (a tour through
the demo site, mapping a new site, the path finder walking back, sliding around a person,
stopping for a crowd, the handheld driving at a person) run in CI and must stay green.
