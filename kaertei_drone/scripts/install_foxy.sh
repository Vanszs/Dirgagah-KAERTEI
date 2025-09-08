#!/bin/bash

# KAERTEI — Jetson Nano (Ubuntu 20.04) ROS 2 Foxy Installer
# Minimal, no Docker. Installs ROS 2 Foxy, MAVROS, core deps, and sets permissions.

set -e

ROS_DISTRO=foxy

info()  { echo -e "\033[0;36m[INFO]\033[0m  $*"; }
ok()    { echo -e "\033[0;32m[OK]\033[0m    $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
err()   { echo -e "\033[0;31m[ERR]\033[0m   $*"; }

check_os() {
  . /etc/os-release || { err "Cannot read /etc/os-release"; exit 1; }
  if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "20.04" ]]; then
    warn "Detected: $PRETTY_NAME. This script targets Ubuntu 20.04 (Focal)."
    read -p "Continue anyway? [y/N]: " -n 1 -r; echo
    [[ $REPLY =~ ^[Yy]$ ]] || exit 1
  fi
  ok "OS: $PRETTY_NAME"
}

install_ros2() {
  if command -v ros2 >/dev/null 2>&1; then
    ok "ROS 2 already installed: $(ros2 --version || echo unknown)"
    return
  fi

  info "Installing ROS 2 $ROS_DISTRO..."
  sudo apt update
  sudo apt install -y software-properties-common curl gnupg2 lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
  sudo apt install -y ros-$ROS_DISTRO-desktop python3-rosdep python3-colcon-common-extensions

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
  fi
  rosdep update || true

  if ! grep -q "/opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "# ROS 2 $ROS_DISTRO" >> ~/.bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
  fi
  ok "ROS 2 $ROS_DISTRO installed"
}

install_mavros() {
  info "Installing MAVROS..."
  sudo apt install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ros-$ROS_DISTRO-mavros-msgs
  if [ -x "/opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh" ]; then
    sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh || true
  else
    warn "GeographicLib datasets installer not found; ensure datasets are present for GPS."
  fi
  ok "MAVROS installed"
}

install_deps() {
  info "Installing additional dependencies..."
  sudo apt install -y \
    python3-pip python3-venv python3-opencv python3-numpy python3-serial \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-sensor-msgs \
    v4l-utils usbutils

  # Ensure Python packaging tools are compatible on Ubuntu 20.04/Python 3.8.
  # Newer setuptools expects importlib_metadata to expose EntryPoints API.
  # Upgrade both to avoid colcon/setuptools errors during build.
  python3 -m pip install --user --upgrade \
    pip \
    setuptools \
    packaging \
    importlib_metadata>=4.6 \
    wheel \
    psutil \
    PyYAML
  ok "Dependencies installed"
}

setup_permissions() {
  info "Setting up device permissions..."
  sudo usermod -a -G dialout,video,tty "$USER"
  sudo tee /etc/udev/rules.d/99-kaertei.rules >/dev/null <<'EOF'
# Serial devices (Pixhawk/USB‑Serial)
SUBSYSTEM=="tty", GROUP="dialout", MODE="0666"
# Cameras
KERNEL=="video[0-9]*", GROUP="video", MODE="0666"
EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
  ok "Permissions configured (reboot or re-login may be required)"
}

echo "KAERTEI — ROS 2 Foxy installer (Jetson Nano / Ubuntu 20.04)"
check_os
install_ros2
install_mavros
install_deps
setup_permissions

ok "Installation complete. Next:"
echo "  1) New terminal or: source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  2) Build:   ./kaertei_drone/scripts/build_kaertei.sh"
echo "  3) Run:     ./kaertei_drone/scripts/run_kaertei.sh [debug|auto]"
