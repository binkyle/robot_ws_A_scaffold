#!/bin/sh
# ros2_auto_install_ft_portable.sh — Fault‑tolerant ROS 2 installer (POSIX /bin/sh)
# - Works with /bin/sh (dash), avoids bashisms & process substitution
# - Auto-detects Ubuntu version → chooses ROS 2 (24.04→jazzy, 22.04→humble, 20.04→foxy[EOL])
# - Scans /etc/apt/sources.list.d/*.list; backs up & comments out failing third‑party lists
# - Adds ROS 2 apt key & repo, installs desktop + colcon, sets up ~/.bashrc
# - Provides Docker fallback for non‑Ubuntu/macOS
#
# Usage:
#   sh ros2_auto_install_ft_portable.sh                 # auto
#   sh ros2_auto_install_ft_portable.sh --ros-distro jazzy|humble|foxy
#   sh ros2_auto_install_ft_portable.sh --scan-only     # only scan failing repos
#   sh ros2_auto_install_ft_portable.sh --no-disable    # don't modify failing repos
#   sh ros2_auto_install_ft_portable.sh --no-bashrc     # don't edit ~/.bashrc
#   sh ros2_auto_install_ft_portable.sh --docker        # print Docker helper and exit
#   sh ros2_auto_install_ft_portable.sh --restore       # restore *.bak_ros2ft backups
#
set -eu

DRY_RUN=0
SCAN_ONLY=0
AUTO_DISABLE=1
NO_BASHRC=0
DOCKER_ONLY=0
RESTORE_BACKUPS=0
ROS_DISTRO_OVERRIDE=""
LOG_FILE="${HOME}/ros2_auto_install_ft_$(date +%Y%m%d_%H%M%S).log"

log(){ printf "[+] %s\n" "$*" | tee -a "$LOG_FILE"; }
warn(){ printf "[!] %s\n" "$*" | tee -a "$LOG_FILE"; }
err(){ printf "[X] %s\n" "$*" | tee -a "$LOG_FILE"; }
run(){
  if [ "$DRY_RUN" -eq 1 ]; then
    printf "DRY-RUN $ %s\n" "$*" | tee -a "$LOG_FILE"
    return 0
  fi

  cmd="$*"
  tmp_status=$(mktemp)
  (
    set +e
    # shellcheck disable=SC2086
    sh -c "$cmd"
    status=$?
    printf '%s' "$status" >"$tmp_status"
    exit "$status"
  ) 2>&1 | tee -a "$LOG_FILE"
  status=$(cat "$tmp_status")
  rm -f "$tmp_status"
  if [ "$status" -ne 0 ]; then
    err "Command failed ($status): $cmd"
    return "$status"
  fi
}


usage(){
  sed -n '1,80p' "$0" | sed 's/^# \{0,1\}//'
}

docker_help(){
  cat <<'EOX'
# Docker quick start (recommended for macOS/other Linux):
docker pull osrf/ros:jazzy-desktop
docker run --rm -it --net=host -v $HOME/robot_ws_A:/ws osrf/ros:jazzy-desktop sh -lc '
  apt update && apt install -y python3-colcon-common-extensions git
  cd /ws && colcon build --symlink-install
  . /opt/ros/jazzy/setup.sh || . /opt/ros/jazzy/setup.bash
'
EOX
}

# Parse args
while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=1;;
    --scan-only) SCAN_ONLY=1;;
    --no-disable) AUTO_DISABLE=0;;
    --no-bashrc) NO_BASHRC=1;;
    --docker) DOCKER_ONLY=1;;
    --restore) RESTORE_BACKUPS=1;;
    --ros-distro) shift; ROS_DISTRO_OVERRIDE="${1:-}";;
    -h|--help) usage; exit 0;;
    *) err "Unknown arg: $1"; usage; exit 1;;
  esac
  shift
done

trap 'status=$?; if [ "$status" -ne 0 ]; then err "Script aborted (exit $status). See log: $LOG_FILE"; fi' EXIT

UNAME_S="$(uname -s)"
if [ "$DOCKER_ONLY" -eq 1 ]; then docker_help; exit 0; fi

if [ "$UNAME_S" = "Darwin" ]; then
  warn "Detected macOS. Use Docker fallback:"
  docker_help
  exit 0
fi
if [ "$UNAME_S" != "Linux" ]; then
  err "Unsupported OS: $UNAME_S"; exit 1
fi

if [ ! -r /etc/os-release ]; then err "/etc/os-release not found."; exit 1; fi
# shellcheck disable=SC1091
. /etc/os-release
ID="${ID:-}"
VERSION_ID="${VERSION_ID:-}"
UBUNTU_CODENAME="${UBUNTU_CODENAME:-}"

if [ "$ID" != "ubuntu" ]; then
  warn "Non-Ubuntu Linux detected ($ID). Use Docker fallback:"
  docker_help
  exit 0
fi

log "Detected Ubuntu $VERSION_ID ($UBUNTU_CODENAME)"

# Restore
if [ "$RESTORE_BACKUPS" -eq 1 ]; then
  for f in /etc/apt/sources.list.d/*.bak_ros2ft; do
    [ -f "$f" ] || continue
    orig="${f%\.bak_ros2ft}"
    log "Restoring $orig from backup"
    run "sudo mv -f '$f' '$orig'"
  done
  log "Backups restored. Exiting (--restore)."
  exit 0
fi

# Pick ROS distro
if [ -n "$ROS_DISTRO_OVERRIDE" ]; then
  ROS_DISTRO="$ROS_DISTRO_OVERRIDE"
else
  case "$VERSION_ID" in
    24.04) ROS_DISTRO="jazzy" ;;
    22.04) ROS_DISTRO="humble" ;;
    20.04) ROS_DISTRO="foxy" ; warn "Ubuntu 20.04 detected; ROS 2 Foxy is EOL." ;;
    *) ROS_DISTRO="humble" ; warn "Unknown Ubuntu $VERSION_ID; default humble." ;;
  esac
fi
log "Target ROS 2 distro: $ROS_DISTRO"

if ! command -v sudo >/dev/null 2>&1; then err "sudo not found"; exit 1; fi

# Test a single list file
test_list(){
  list_file="$1"
  if [ ! -f "$list_file" ]; then return 0; fi
  log "Testing APT source: $list_file"
  # Only use this file as source list
  if apt-get update -o Dir::Etc::sourcelist="$list_file" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0" -y >>"$LOG_FILE" 2>&1; then
    echo "OK"
    return 0
  else
    echo "FAIL"
    return 1
  fi
}

disable_list(){
  list_file="$1"
  if [ "$AUTO_DISABLE" -ne 1 ]; then
    warn "Would disable $list_file (AUTO_DISABLE=0)"
    return 0
  fi
  backup="${list_file}.bak_ros2ft"
  log "Disabling broken APT source: $list_file (backup: $backup)"
  run "sudo cp -f '$list_file' '$backup'"
  # comment 'deb' & 'deb-src' lines
  run "sudo sed -i 's|^deb |# disabled by ros2_auto_install_ft: deb |' '$list_file'"
  run "sudo sed -i 's|^deb-src |# disabled by ros2_auto_install_ft: deb-src |' '$list_file'"
}

# Scan lists
log "Scanning APT source lists..."
BROKEN_LIST=""
MAIN_LIST="/etc/apt/sources.list"
if [ -f "$MAIN_LIST" ]; then
  if ! test_list "$MAIN_LIST" >/dev/null; then
    warn "Main /etc/apt/sources.list has issues; consider switching to official mirrors for $UBUNTU_CODENAME."
  fi
fi
for lf in /etc/apt/sources.list.d/*.list; do
  [ -f "$lf" ] || continue
  if test_list "$lf" >/dev/null; then :
  else
    BROKEN_LIST="$BROKEN_LIST $lf"
  fi
done

if [ -n "$BROKEN_LIST" ]; then
  warn "Broken third-party sources:"
  for b in $BROKEN_LIST; do echo "  - $b" | tee -a "$LOG_FILE" >/dev/null; done
  if [ "$SCAN_ONLY" -eq 1 ]; then
    warn "SCAN-ONLY: not disabling."; exit 2
  fi
  for b in $BROKEN_LIST; do disable_list "$b"; done
else
  log "All third-party sources look OK."
fi

# Refresh remaining sources
log "Refreshing apt lists"
run "sudo apt-get update -y"

# Prereqs & universe
log "Installing prerequisites"
run "sudo apt-get install -y curl gnupg2 lsb-release ca-certificates software-properties-common"
log "Enabling 'universe' (idempotent)"
run "sudo add-apt-repository -y universe || true"

# Add ROS 2 key & repo
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
log "Adding ROS 2 apt key"
run "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ${KEYRING}"
log "Adding ROS 2 apt repository for ${UBUNTU_CODENAME}"
run "sh -c \"echo 'deb [arch=\$(dpkg --print-architecture) signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main' | sudo tee ${ROS_LIST} >/dev/null\""

# Selective updates
log "Selective apt update (Ubuntu main)"
run "sudo apt-get update -o Dir::Etc::sourcelist='/etc/apt/sources.list' -o Dir::Etc::sourceparts='-' -y"
log "Selective apt update (ROS 2 repo)"
run "sudo apt-get update -o Dir::Etc::sourcelist='${ROS_LIST}' -o Dir::Etc::sourceparts='-' -y"

# Install ROS
PKG_DESKTOP="ros-${ROS_DISTRO}-desktop"
log "Installing $PKG_DESKTOP + colcon"
run "sudo apt-get install -y ${PKG_DESKTOP} python3-colcon-common-extensions git build-essential cmake"

# Setup environment
SETUP_BASH="/opt/ros/${ROS_DISTRO}/setup.bash"
SETUP_SH="/opt/ros/${ROS_DISTRO}/setup.sh"
SETUP=""
if [ -f "$SETUP_SH" ]; then SETUP="$SETUP_SH"; elif [ -f "$SETUP_BASH" ]; then SETUP="$SETUP_BASH"; fi
if [ -n "$SETUP" ]; then
  if [ "$NO_BASHRC" -eq 0 ]; then
    if ! grep -qs "$SETUP" "$HOME/.bashrc"; then
      log "Appending source line to ~/.bashrc"
      run "printf '\n# ROS 2 ${ROS_DISTRO}\n. ${SETUP}\n' >> \"$HOME/.bashrc\""
    else
      warn "~/.bashrc already contains source line for ${ROS_DISTRO}"
    fi
  fi
else
  err "ROS setup script not found under /opt/ros/${ROS_DISTRO}"
  exit 1
fi

# Verify
log "Verifying installation"
# shellcheck disable=SC1090
if [ -n "$SETUP" ]; then . "$SETUP"; fi
if ! command -v ros2 >/dev/null 2>&1; then
  err "ros2 command not found after sourcing $SETUP"
  exit 1
fi
ros2 --version | tee -a "$LOG_FILE" >/dev/null || true

trap - EXIT
log "Done. Open a new shell or run: . ${SETUP}"
log "Log saved to: $LOG_FILE"
