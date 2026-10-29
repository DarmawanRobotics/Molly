# Dev container

ROS 2 Humble with every dependency of `src/` (rosdep: Molly, the Dharma stack, the Lite3
package), Livox-SDK2, the lint tools (pre-commit, ruff) and an XFCE desktop over VNC for RViz.
The repository is bind-mounted at the path it has on the robot (`/home/robot/dev/Molly`), so
configs and launch defaults behave the same; `build/`, `install/` and `log/` live in Docker
volumes (fast on macOS, kept across restarts).

```bash
scripts/dev.sh up        # imports the Dharma stack (molly.repos) if missing, builds the image, starts it
scripts/dev.sh build     # colcon build inside (args go to colcon)
scripts/dev.sh test      # colcon test + colcon test-result
scripts/dev.sh shell     # a shell with ROS and the workspace sourced
scripts/dev.sh vnc       # VNC localhost:5901 (macOS Screen Sharing) or http://localhost:6080/vnc.html
scripts/dev.sh down      # stop (volumes kept); scripts/dev.sh clean wipes build/install/log
scripts/dev.sh doctor    # Docker, compose, memory and GitHub access
```

The VNC password defaults to `molly` (`VNC_PASSWORD=... scripts/dev.sh up`); the ports are bound
to localhost only. Rebuild the image (`up` does it) after a `package.xml` gains a dependency;
code changes only need `scripts/dev.sh build`. Docker Desktop on macOS cannot do DDS multicast to
the robot: use the container for building, tests and the simulator
(`ros2 launch molly_bringup sim.launch.py rviz:=true`).
