#!/usr/bin/env bash
# Check or install everything the Molly stack needs on a robot (or in the dev container).
#
#   script/setup.sh                      # same as: check
#   script/setup.sh check                # status of every component
#   script/setup.sh install              # install the default components that are missing
#   script/setup.sh install livox build  # install only these
#   script/setup.sh install all          # every component, including ros and service
#
# Components: ros tools submodules dharma livox robot legkilo rosdep realsense webrtc web speech ollama
#             build config service network
# Settings come from the environment, then from /etc/molly/molly.env (ROBOT_SETUP,
# ROBOT_IP, CONVERSATION, WEB, ...): no need to source it first.
# Env: ROS_DISTRO (humble), LIVOX_SDK_REF (master), ROBOT_SETUP (the robot package's own
#      installer, `<script> check|install`, e.g. for a vendor SDK; see src/xtras/README.md),
#      ROBOT_IP (robot SDK address for the network check), LIVOX_IP (LiDAR address, idem),
#      CHAT_MODEL (qwen2.5:3b, the conversation model), PIPER_VOICES (id_ID-news_tts-medium
#      en_US-lessac-medium), BUILD_WORKERS / BUILD_JOBS (parallel packages / compile jobs, lower them when
#      the build runs out of memory), REALSENSE_BLACKLIST_UVC (1 on Jetson), MEDIAMTX (a MediaMTX
#      binary to use instead of installing one), FORCE=1 (reinstall anyway)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${MOLLY_ENV:-/etc/molly/molly.env}"
ENV_EXAMPLE="$ROOT/script/molly.env.example"
SITE_EXAMPLE="$ROOT/src/molly/config/site.example.yaml"

# Fill what the caller did not set from the robot's env file (read by bash, like the service).
load_env_file() {
    [[ -r "$ENV_FILE" ]] || return 0
    local entry key
    while IFS= read -r -d '' entry; do
        key=${entry%%=*}
        [[ -n "${!key+x}" ]] || export "${entry?}"
    done < <(
        _keys=$(sed -nE 's/^[[:space:]]*(export[[:space:]]+)?([A-Za-z_][A-Za-z0-9_]*)=.*/\2/p' "$ENV_FILE")
        set +eu -a
        # shellcheck disable=SC1090
        source "$ENV_FILE" >/dev/null 2>&1
        for _key in $_keys; do printf '%s=%s\0' "$_key" "${!_key-}"; done
    )
}
load_env_file

ROS_DISTRO="${ROS_DISTRO:-humble}"
LIVOX_SDK_REPO="${LIVOX_SDK_REPO:-https://github.com/Livox-SDK/Livox-SDK2.git}"
LIVOX_SDK_REF="${LIVOX_SDK_REF:-master}"
ROBOT_SETUP="${ROBOT_SETUP:-}"
SERVICE_NAME=molly
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"
SITE_FILE="${SITE_CONFIG:-/etc/molly/site.yaml}"
CHAT_MODEL="${CHAT_MODEL:-qwen2.5:3b}"
REALSENSE_RULES=/etc/udev/rules.d/99-realsense-libusb.rules
LIVOX_IP="${LIVOX_IP:-}"
ROBOT_IP="${ROBOT_IP:-}"
OLLAMA_URL="${OLLAMA_URL:-http://127.0.0.1:11434}"
BUILD_WORKERS="${BUILD_WORKERS:-}"
BUILD_JOBS="${BUILD_JOBS:-}"
BUILD_STAMP="$ROOT/install/.molly_build"
UVC_BLACKLIST=/etc/modprobe.d/blacklist-uvcvideo-realsense.conf
REALSENSE_BLACKLIST_UVC="${REALSENSE_BLACKLIST_UVC:-$([[ "$(uname -m)" == aarch64 ]] && echo 1 || echo 0)}"

ALL=(ros tools submodules dharma livox robot legkilo rosdep realsense webrtc web speech ollama
     build config service network)
DEFAULT_INSTALL=(tools submodules dharma livox robot legkilo rosdep realsense webrtc web speech ollama
                 build config)

SUDO=$([[ "$(id -u)" -eq 0 ]] && echo "" || echo "sudo")
IN_DOCKER=$([[ -f /.dockerenv ]] && echo 1 || echo 0)
if [[ -t 1 ]]; then
    GREEN=$'\e[32m' RED=$'\e[31m' YELLOW=$'\e[33m' BOLD=$'\e[1m' RESET=$'\e[0m'
else
    GREEN="" RED="" YELLOW="" BOLD="" RESET=""
fi

log() { echo "${BOLD}==>${RESET} $*"; }
enabled() { [[ "${1,,}" =~ ^(1|true|yes|on)$ ]]; }
# apt may be busy (packagekit or unattended-upgrades refreshing): wait for it. A broken mirror in
# the apt sources fails `apt-get update` but rarely the packages we need: warn, then try the
# install (it says which package it could not get).
apt_install() {
    local try out
    for try in $(seq 18); do
        out=$($SUDO apt-get update -qq 2>&1) && break
        if [[ "$out" == *"Could not get lock"* && $try -lt 18 ]]; then
            [[ $try -eq 1 ]] && log "apt is busy (another update, e.g. packagekit), waiting up to 3 min"
            sleep 10
            continue
        fi
        echo "$out" >&2
        log "${YELLOW}apt-get update had errors: check the mirrors in /etc/apt/sources.list*${RESET}"
        break
    done
    $SUDO apt-get -o DPkg::Lock::Timeout=180 install -y --no-install-recommends "$@"
}
source_ros() {
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
}

# ---------------------------------------------------------------- components
# Each component has check_<name> (exit 0 = ok, prints one detail line) and install_<name>.

check_ros() {
    [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] && echo "/opt/ros/$ROS_DISTRO" && return 0
    echo "ROS 2 $ROS_DISTRO not found"; return 1
}
install_ros() {
    apt_install software-properties-common curl gnupg lsb-release locales
    $SUDO add-apt-repository -y universe
    $SUDO curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
    apt_install "ros-$ROS_DISTRO-ros-base"
}

check_tools() {
    local missing=()
    for cmd in colcon rosdep git cmake g++ tmux curl; do
        command -v "$cmd" >/dev/null || missing+=("$cmd")
    done
    [[ ${#missing[@]} -eq 0 ]] && echo "colcon rosdep git cmake g++ tmux curl" && return 0
    echo "missing: ${missing[*]}"; return 1
}
install_tools() {
    apt_install python3-colcon-common-extensions python3-rosdep python3-pip git cmake \
        build-essential tmux curl ca-certificates
}

check_submodules() {
    local missing
    missing=$(git -C "$ROOT" submodule status 2>/dev/null | awk '/^-/ {print $2}' | xargs -r basename -a)
    [[ -z "$missing" ]] && echo "all checked out" && return 0
    echo "not checked out: $(echo "$missing" | tr '\n' ' ')"; return 1
}
install_submodules() {
    git -C "$ROOT" submodule update --init --recursive && return 0
    log "SSH clone failed, retrying submodules over HTTPS"
    git -C "$ROOT" -c "url.https://github.com/.insteadOf=git@github.com:" \
        submodule update --init --recursive
}

check_dharma() {
    local missing=() name
    for name in $(awk '/^  dharma\// {sub(":", ""); print $1}' "$ROOT/molly.repos"); do
        [[ -f "$ROOT/src/$name/package.xml" || -d "$ROOT/src/$name/.git" ]] || missing+=("$name")
    done
    [[ ${#missing[@]} -eq 0 ]] && echo "src/dharma: all checked out" && return 0
    echo "not checked out: ${missing[*]}"; return 1
}
install_dharma() {
    command -v vcs >/dev/null || apt_install python3-vcstool
    if ! vcs import --skip-existing "$ROOT/src" < "$ROOT/molly.repos"; then
        log "SSH clone failed, retrying the dharma packages over HTTPS"
        GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=url.https://github.com/.insteadOf \
            GIT_CONFIG_VALUE_0=git@github.com: \
            vcs import --skip-existing "$ROOT/src" < "$ROOT/molly.repos"
    fi
    vcs pull "$ROOT/src/dharma"
}

check_livox() {
    [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]] \
        && echo "/usr/local/lib/liblivox_lidar_sdk_shared.so" && return 0
    echo "Livox-SDK2 not installed"; return 1
}
install_livox() {
    local tmp
    tmp=$(mktemp -d)
    git clone --depth 1 --branch "$LIVOX_SDK_REF" "$LIVOX_SDK_REPO" "$tmp"
    cmake -S "$tmp" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$tmp/build" -j"$(nproc)"
    $SUDO cmake --install "$tmp/build"
    $SUDO ldconfig
    rm -rf "$tmp"
}

# The robot package may bring its own installer (vendor SDK, udev rules, ...): ROBOT_SETUP.
check_robot() {
    [[ -n "$ROBOT_SETUP" ]] || { echo "skipped, no ROBOT_SETUP"; return 0; }
    [[ -x "$ROBOT_SETUP" ]] || { echo "ROBOT_SETUP $ROBOT_SETUP is not executable"; return 1; }
    "$ROBOT_SETUP" check
}
install_robot() {
    [[ -x "$ROBOT_SETUP" ]] || { echo "set ROBOT_SETUP to the robot package's installer"; return 1; }
    "$ROBOT_SETUP" install
}

# Leg-KILO (src/xtras/slam/leg_kilo, odometry:=leg_kilo) builds Iridescence and needs these.
LEGKILO_DIR="$ROOT/src/xtras/slam/leg_kilo"
LEGKILO_APT=(libeigen3-dev libpcl-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev
    libboost-filesystem-dev libboost-system-dev libtbb-dev liblz4-dev libceres-dev libglm-dev
    libglfw3-dev libpng-dev libjpeg-dev)
check_legkilo() {
    [[ -d "$LEGKILO_DIR/legkilo" ]] || { echo "skipped, no submodule at $LEGKILO_DIR"; return 0; }
    local missing=()
    for pkg in "${LEGKILO_APT[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    [[ ${#missing[@]} -eq 0 ]] || { echo "missing: ${missing[*]}"; return 1; }
    [[ -f "$LEGKILO_DIR/msg/livox_ros_driver2/COLCON_IGNORE" ]] \
        || { echo "upstream checkout: use the molly branch of the fork"; return 1; }
    echo "build deps installed"
}
install_legkilo() {
    apt_install "${LEGKILO_APT[@]}"
}

check_rosdep() {
    check_ros >/dev/null || { echo "needs ROS"; return 1; }
    [[ -d /etc/ros/rosdep/sources.list.d ]] || { echo "rosdep not initialised"; return 1; }
    source_ros
    rosdep check --from-paths "$ROOT/src" --ignore-src >/tmp/molly_rosdep.log 2>&1 \
        && echo "all system dependencies installed" && return 0
    echo "missing keys, see /tmp/molly_rosdep.log"; return 1
}
install_rosdep() {
    [[ -d /etc/ros/rosdep/sources.list.d ]] || $SUDO rosdep init
    source_ros
    rosdep update --rosdistro "$ROS_DISTRO"
    rosdep install --from-paths "$ROOT/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO"
}

# RealSense: the ROS driver comes from apt (it pulls librealsense2); the camera needs udev rules.
REALSENSE_PKG="ros-$ROS_DISTRO-realsense2-camera"
realsense_rules() {
    local f
    for f in /etc/udev/rules.d/*realsense*.rules /lib/udev/rules.d/*realsense*.rules; do
        [[ -f "$f" ]] && echo "$f" && return 0
    done
    return 1
}
check_realsense() {
    dpkg -s "$REALSENSE_PKG" >/dev/null 2>&1 || { echo "$REALSENSE_PKG not installed"; return 1; }
    [[ "$IN_DOCKER" == "1" ]] && echo "$REALSENSE_PKG (udev skipped in docker)" && return 0
    local rules
    rules=$(realsense_rules) || { echo "RealSense udev rules missing (camera needs root)"; return 1; }
    if [[ "$REALSENSE_BLACKLIST_UVC" == "1" && ! -f "$UVC_BLACKLIST" ]]; then
        echo "uvcvideo not blacklisted: it fights librealsense (libusb) for the camera"; return 1
    fi
    echo "$REALSENSE_PKG, $rules"
}
install_realsense() {
    dpkg -s "$REALSENSE_PKG" >/dev/null 2>&1 || apt_install "$REALSENSE_PKG"
    [[ "$IN_DOCKER" == "1" ]] && return 0
    if ! realsense_rules >/dev/null; then
        local shipped
        shipped=$(find "/opt/ros/$ROS_DISTRO" -name '*realsense*.rules' 2>/dev/null | head -n1 || true)
        if [[ -n "$shipped" ]]; then
            $SUDO cp "$shipped" "$REALSENSE_RULES"
        else
            curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
                | $SUDO tee "$REALSENSE_RULES" >/dev/null
        fi
        $SUDO udevadm control --reload-rules && $SUDO udevadm trigger
    fi
    # The apt librealsense reads the camera over libusb; when the kernel uvcvideo driver binds it
    # too, udev rebinds the camera every second (log storm, camera drops out).
    if [[ "$REALSENSE_BLACKLIST_UVC" == "1" && ! -f "$UVC_BLACKLIST" ]]; then
        echo "blacklist uvcvideo" | $SUDO tee "$UVC_BLACKLIST" >/dev/null
        $SUDO modprobe -r uvcvideo 2>/dev/null || log "uvcvideo is in use: reboot once"
    fi
}

# The handheld's WebRTC video: GStreamer (H.264 encoders, RTSP) and MediaMTX, a pinned release
# checked against its sha256 (robot.launch.py writes the configuration and starts it).
MEDIAMTX="${MEDIAMTX:-}"
MEDIAMTX_VERSION=v1.20.1
MEDIAMTX_DIR=/opt/molly/mediamtx
declare -A MEDIAMTX_SHA256=(
    [amd64]=81b143f55a5d23d4a8c028d52869c14ea4a59919900528698fcc97a747fd69c6
    [arm64]=d1689f0bfefb1864e5ed3dcc8495eb2d7ec0a654f90bf3cd48980cb3bd08718a
    [armv7]=ee4dde41e04de75a237c7bb2733cf4102ca212db05efefbcbbcd1ea60bd77e33
)
WEBRTC_APT=(gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-rtsp python3-gi
    gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0)
# Same search as robot.launch.py: MEDIAMTX, our install, PATH, the usual manual installs.
mediamtx_bin() {
    local candidates=("$MEDIAMTX") path
    [[ -n "$MEDIAMTX" ]] || candidates=("$MEDIAMTX_DIR/mediamtx" "$(command -v mediamtx || true)"
        /usr/local/bin/mediamtx "$HOME/mediamtx/mediamtx")
    for path in "${candidates[@]}"; do
        [[ -n "$path" && -f "$path" && -x "$path" ]] && echo "$path" && return 0
    done
    return 1
}
mediamtx_version() { "$1" --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || true; }
check_webrtc() {
    local missing=() pkg bin version
    for pkg in "${WEBRTC_APT[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    [[ ${#missing[@]} -eq 0 ]] || { echo "missing: ${missing[*]}"; return 1; }
    bin=$(mediamtx_bin) || { echo "MediaMTX not installed (handheld video: JPEG only)"; return 1; }
    version=$(mediamtx_version "$bin")
    # the configuration robot.launch.py writes needs 1.11 (rtspTransports)
    printf '1.11.0\n%s\n' "${version:-0}" | sort -V -C \
        || { echo "$bin is v${version:-?}, needs v1.11 or newer"; return 1; }
    echo "MediaMTX v$version ($bin), GStreamer"
}
install_webrtc() {
    apt_install "${WEBRTC_APT[@]}"
    if [[ "${FORCE:-0}" != "1" ]] && check_webrtc >/dev/null 2>&1; then
        return 0  # a recent enough MediaMTX is already there
    fi
    local arch tmp tarball
    case "$(uname -m)" in
        aarch64|arm64) arch=arm64 ;;
        x86_64|amd64) arch=amd64 ;;
        armv7l) arch=armv7 ;;
        *) echo "no MediaMTX release for $(uname -m)"; return 1 ;;
    esac
    tmp=$(mktemp -d)
    tarball="mediamtx_${MEDIAMTX_VERSION}_linux_${arch}.tar.gz"
    curl -fsSL -o "$tmp/$tarball" \
        "https://github.com/bluenviron/mediamtx/releases/download/$MEDIAMTX_VERSION/$tarball"
    echo "${MEDIAMTX_SHA256[$arch]}  $tmp/$tarball" | sha256sum -c --quiet \
        || { rm -rf "$tmp"; echo "$tarball: checksum mismatch"; return 1; }
    tar -xzf "$tmp/$tarball" -C "$tmp" mediamtx
    $SUDO install -D -m 755 "$tmp/mediamtx" "$MEDIAMTX_DIR/mediamtx"
    rm -rf "$tmp"
    log "MediaMTX $MEDIAMTX_VERSION in $MEDIAMTX_DIR: robot.launch.py runs it (8890/tcp, 8190/udp)"
}

# The dashboard: rosbridge (WebSocket) and web_video_server (MJPEG); robot.launch.py web:=true.
WEB_APT=("ros-$ROS_DISTRO-rosbridge-suite" "ros-$ROS_DISTRO-web-video-server")
check_web() {
    enabled "${WEB:-true}" || { echo "skipped, WEB=false"; return 0; }
    local missing=() pkg
    for pkg in "${WEB_APT[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    [[ ${#missing[@]} -eq 0 ]] || { echo "missing: ${missing[*]}"; return 1; }
    echo "rosbridge (ws ${WEB_PORT:-9090}), web_video_server (http ${WEB_VIDEO_PORT:-8080})"
}
install_web() {
    apt_install "${WEB_APT[@]}"
}

# Speech: Piper (natural voices, a pinned release checked against its sha256, voices from the
# piper-voices repository), eSpeak NG as the fallback, aplay for the sound card.
PIPER_VERSION=2023.11.14-2
PIPER_DIR=/opt/molly/piper
PIPER_VOICES="${PIPER_VOICES:-id_ID-news_tts-medium en_US-lessac-medium}"
PIPER_VOICES_URL=https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0
declare -A PIPER_SHA256=(
    [aarch64]=fea0fd2d87c54dbc7078d0f878289f404bd4d6eea6e7444a77835d1537ab88eb
    [x86_64]=a50cb45f355b7af1f6d758c1b360717877ba0a398cc8cbe6d2a7a3a26e225992
)
SPEECH_APT=(espeak-ng alsa-utils)
# id_ID-news_tts-medium -> id/id_ID/news_tts/medium/id_ID-news_tts-medium
voice_path() {
    local voice=$1 locale name quality
    locale=${voice%%-*}
    quality=${voice##*-}
    name=${voice#*-}
    name=${name%-*}
    echo "${locale%%_*}/$locale/$name/$quality/$voice"
}
check_speech() {
    enabled "${SPEECH:-true}" || { echo "skipped, SPEECH=false"; return 0; }
    local missing=() pkg voice
    for pkg in "${SPEECH_APT[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    [[ ${#missing[@]} -eq 0 ]] || { echo "missing: ${missing[*]}"; return 1; }
    [[ -x "$PIPER_DIR/piper" ]] || { echo "Piper not installed (eSpeak NG voice only)"; return 1; }
    for voice in $PIPER_VOICES; do
        [[ -f "$PIPER_DIR/voices/$voice.onnx" && -f "$PIPER_DIR/voices/$voice.onnx.json" ]] \
            || { echo "Piper voice $voice missing"; return 1; }
    done
    echo "Piper $PIPER_VERSION ($PIPER_VOICES), eSpeak NG, aplay"
}
install_speech() {
    apt_install "${SPEECH_APT[@]}"
    local arch tmp tarball voice file
    arch=$(uname -m)
    [[ -n "${PIPER_SHA256[$arch]:-}" ]] || { echo "no Piper release for $arch"; return 1; }
    tmp=$(mktemp -d)
    if [[ ! -x "$PIPER_DIR/piper" || "${FORCE:-0}" == "1" ]]; then
        tarball="piper_linux_${arch}.tar.gz"
        curl -fsSL -o "$tmp/$tarball" \
            "https://github.com/rhasspy/piper/releases/download/$PIPER_VERSION/$tarball"
        echo "${PIPER_SHA256[$arch]}  $tmp/$tarball" | sha256sum -c --quiet \
            || { rm -rf "$tmp"; echo "$tarball: checksum mismatch"; return 1; }
        tar -xzf "$tmp/$tarball" -C "$tmp"
        $SUDO mkdir -p "$PIPER_DIR"
        $SUDO cp -a "$tmp/piper/." "$PIPER_DIR/"
    fi
    $SUDO mkdir -p "$PIPER_DIR/voices"
    for voice in $PIPER_VOICES; do
        for file in "$voice.onnx" "$voice.onnx.json"; do
            [[ -f "$PIPER_DIR/voices/$file" && "${FORCE:-0}" != "1" ]] && continue
            curl -fsSL --retry 3 -o "$tmp/$file" "$PIPER_VOICES_URL/$(voice_path "$voice")${file#"$voice"}"
            $SUDO install -m 644 "$tmp/$file" "$PIPER_DIR/voices/$file"
        done
    done
    rm -rf "$tmp"
    log "Piper in $PIPER_DIR: speech_node uses it (espeak-ng when a voice is missing)"
}

check_build() {
    [[ -f "$ROOT/install/setup.bash" ]] || { echo "workspace not built"; return 1; }
    local n built head
    n=$(find "$ROOT/install" -mindepth 1 -maxdepth 1 -type d | wc -l)
    head=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)
    built=$(cat "$BUILD_STAMP" 2>/dev/null || echo unknown)
    [[ "$built" == "$head" ]] || { echo "built from $built, source is $head: rebuild"; return 1; }
    echo "$n packages in install/, built from $head"
}
install_build() {
    source_ros
    local args=(--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
    [[ -n "$BUILD_WORKERS" ]] && args=(--parallel-workers "$BUILD_WORKERS" "${args[@]}")
    [[ -n "$BUILD_JOBS" ]] && export MAKEFLAGS="-j$BUILD_JOBS"
    (cd "$ROOT" && colcon build "${args[@]}")
    git -C "$ROOT" rev-parse --short HEAD > "$BUILD_STAMP" 2>/dev/null || echo unknown > "$BUILD_STAMP"
}

# The conversation model (CONVERSATION=true) runs in Ollama. llama-server, Ollama's runner, keeps
# old prompts in RAM: up to 8 GiB per loaded model by default, which on a Jetson is the memory the
# GPU shares: off for the Ollama service (OLLAMA_CACHE_RAM for an Ollama that passes --cache-ram
# itself, LLAMA_ARG_CACHE_RAM for one that does not).
OLLAMA_DROPIN=/etc/systemd/system/ollama.service.d/molly.conf
ollama_dropin() {
    printf '%s\n' \
        '# Molly (script/setup.sh install ollama): no prompt cache in RAM (up to 8 GiB per model)' \
        '[Service]' \
        'Environment="OLLAMA_CACHE_RAM=0"' \
        'Environment="LLAMA_ARG_CACHE_RAM=0"'
}

# The model must be in the store of the Ollama service (user ollama), not in ~/.ollama.
check_ollama() {
    [[ "$IN_DOCKER" == "1" ]] && echo "skipped in docker" && return 0
    enabled "${CONVERSATION:-false}" || { echo "skipped, CONVERSATION=false"; return 0; }
    command -v ollama >/dev/null || { echo "Ollama not installed"; return 1; }
    local tags
    tags=$(curl -fsS --max-time 3 "$OLLAMA_URL/api/tags" 2>/dev/null) \
        || { echo "Ollama service not running: sudo systemctl enable --now ollama"; return 1; }
    [[ "$tags" == *"\"name\":\"$CHAT_MODEL\""* || "$tags" == *"\"name\":\"$CHAT_MODEL:"* ]] \
        || { echo "model $CHAT_MODEL not in the Ollama service"; return 1; }
    [[ "$(cat "$OLLAMA_DROPIN" 2>/dev/null)" == "$(ollama_dropin)" ]] \
        || { echo "prompt cache can take 8 GiB of RAM ($OLLAMA_DROPIN missing)"; return 1; }
    echo "ollama service with $CHAT_MODEL, no prompt cache in RAM"
}
install_ollama() {
    enabled "${CONVERSATION:-false}" || { echo "CONVERSATION=false: no conversation model"; return 0; }
    command -v ollama >/dev/null || curl -fsSL https://ollama.com/install.sh | sh
    if [[ "$(cat "$OLLAMA_DROPIN" 2>/dev/null)" != "$(ollama_dropin)" ]]; then
        $SUDO mkdir -p "$(dirname "$OLLAMA_DROPIN")"
        ollama_dropin | $SUDO tee "$OLLAMA_DROPIN" >/dev/null
        $SUDO systemctl daemon-reload
        $SUDO systemctl restart ollama 2>/dev/null || true  # also frees a full prompt cache
    fi
    $SUDO systemctl enable --now ollama 2>/dev/null || true
    for _ in $(seq 20); do curl -fsS --max-time 2 "$OLLAMA_URL/api/tags" >/dev/null 2>&1 && break; sleep 1; done
    ollama pull "$CHAT_MODEL"  # pulled by the service, into its own store
}


# /etc/molly: the env file (read by the service) and the site file (parameters).
env_keys() { sed -nE 's/^([A-Za-z_][A-Za-z0-9_]*)=.*/\1/p' "$1"; }
has_key() { grep -qE "^[[:space:]]*(export[[:space:]]+)?$1=" "$ENV_FILE"; }
check_config() {
    [[ -f "$ENV_FILE" ]] || { echo "no $ENV_FILE"; return 1; }
    [[ -r "$ENV_FILE" ]] || { echo "$ENV_FILE not readable by $(id -un)"; return 1; }
    local key missing=() bad
    for key in $(env_keys "$ENV_EXAMPLE"); do
        has_key "$key" || missing+=("$key")
    done
    [[ ${#missing[@]} -eq 0 ]] || { echo "$ENV_FILE lacks ${missing[*]}"; return 1; }
    # bash runs `KEY=a b` as the command b, and systemd keeps a trailing comment in the value
    bad=$(grep -nE "^[A-Za-z_][A-Za-z0-9_]*=[^\"'].*[[:space:]#]" "$ENV_FILE" | cut -d: -f1 | xargs)
    [[ -z "$bad" ]] || { echo "$ENV_FILE line $bad: quote values with spaces, no comments"; return 1; }
    [[ -f "$SITE_FILE" ]] || { echo "no $SITE_FILE"; return 1; }
    echo "$ENV_FILE, $SITE_FILE"
}
install_config() {
    local group key added=()
    group=$(id -gn)
    if [[ ! -f "$ENV_FILE" ]]; then
        $SUDO install -D -m 640 -o root -g "$group" "$ENV_EXAMPLE" "$ENV_FILE"
        log "created $ENV_FILE: set ROBOT_BRINGUP, ROBOT_NS, ROBOT_IP, SITE"
    else
        for key in $(env_keys "$ENV_EXAMPLE"); do
            has_key "$key" && continue
            grep -E "^$key=" "$ENV_EXAMPLE" | $SUDO tee -a "$ENV_FILE" >/dev/null
            added+=("$key")
        done
        [[ ${#added[@]} -eq 0 ]] || log "added to $ENV_FILE with example values: ${added[*]}"
    fi
    if [[ ! -f "$SITE_FILE" ]]; then
        $SUDO install -D -m 640 -o root -g "$group" "$SITE_EXAMPLE" "$SITE_FILE"
        log "created $SITE_FILE: set the teleop token and this robot's values"
    fi
}

service_unit() {
    local deps=network-online.target
    enabled "${CONVERSATION:-false}" && deps+=" ollama.service"
    cat <<EOF
[Unit]
Description=Molly robot stack
After=$deps
Wants=$deps
StartLimitIntervalSec=0

[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$ROOT
EnvironmentFile=-$ENV_FILE
Environment=WS=$ROOT
Environment=PYTHONUNBUFFERED=1
Environment=RCUTILS_COLORIZED_OUTPUT=0
ExecStart=$ROOT/script/run_robot.sh
Restart=always
RestartSec=5
# SIGINT to ros2 launch only, which stops every node once; leftovers get SIGKILL after the timeout
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=30
Nice=-5
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
EOF
}
check_service() {
    [[ "$IN_DOCKER" == "1" ]] && echo "skipped in docker" && return 0
    [[ -f "$SERVICE_FILE" ]] || { echo "$SERVICE_NAME.service not installed"; return 1; }
    service_unit | cmp -s - "$SERVICE_FILE" \
        || { echo "$SERVICE_FILE is outdated (other path, user or CONVERSATION)"; return 1; }
    systemctl is-enabled "$SERVICE_NAME.service" >/dev/null 2>&1 \
        || { echo "$SERVICE_NAME.service installed but not enabled at boot"; return 1; }
    echo "$SERVICE_NAME.service enabled ($(systemctl is-active $SERVICE_NAME.service))"
}
install_service() {
    install_config
    service_unit | $SUDO tee "$SERVICE_FILE" >/dev/null
    $SUDO systemctl daemon-reload
    $SUDO systemctl enable "$SERVICE_NAME.service"
    log "start: sudo systemctl restart $SERVICE_NAME   logs: journalctl -fu $SERVICE_NAME"
}

check_network() {
    local ok=0 out=() pairs=()
    [[ -n "$LIVOX_IP" ]] && pairs+=("livox=$LIVOX_IP")
    [[ -n "$ROBOT_IP" ]] && pairs+=("robot=$ROBOT_IP")
    [[ ${#pairs[@]} -gt 0 ]] || { echo "skipped, no ROBOT_IP / LIVOX_IP"; return 0; }
    for pair in "${pairs[@]}"; do
        if ping -c1 -W1 "${pair#*=}" >/dev/null 2>&1; then out+=("${pair} up"); else out+=("${pair} down"); ok=1; fi
    done
    echo "${out[*]}"; return $ok
}
install_network() {
    echo "configure the interfaces by hand, as the robot package says (Lite3: lite3_setup.sh network)"
    return 1
}

# ---------------------------------------------------------------- driver

check_one() {
    local name=$1 detail status
    if detail=$("check_$name" 2>&1); then
        status="${GREEN}ok     ${RESET}"
    else
        status="${RED}missing${RESET}"
    fi
    printf '  %-15s %s  %s\n' "$name" "$status" "$(echo "$detail" | tail -n1)"
    [[ "$status" == *ok* ]]
}

do_check() {
    local failed=0
    echo "${BOLD}Molly setup ($([[ $IN_DOCKER == 1 ]] && echo docker || echo host), $(uname -m))${RESET}"
    for name in "${ALL[@]}"; do
        check_one "$name" || failed=$((failed + 1))
    done
    [[ $failed -eq 0 ]] && echo "${GREEN}everything is ready${RESET}" \
        || echo "${YELLOW}$failed component(s) need attention: script/setup.sh install <component>${RESET}"
}

do_install() {
    local targets=("$@")
    [[ ${#targets[@]} -eq 0 ]] && targets=("${DEFAULT_INSTALL[@]}")
    [[ "${targets[0]}" == "all" ]] && targets=("${ALL[@]/network}")
    for name in "${targets[@]}"; do
        [[ -z "$name" ]] && continue
        declare -F "install_$name" >/dev/null || { echo "${RED}unknown component: $name${RESET}"; exit 1; }
        if [[ "${FORCE:-0}" != "1" ]] && "check_$name" >/dev/null 2>&1; then
            echo "  $name: already ok"
            continue
        fi
        log "installing $name"
        "install_$name"
        check_one "$name" || { echo "${RED}$name still not ok after install${RESET}"; exit 1; }
    done
}

case "${1:-check}" in
    check) do_check ;;
    install) shift; do_install "$@" ;;
    *) sed -n '2,13p' "$0"; exit 1 ;;
esac
