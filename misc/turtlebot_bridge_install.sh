#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SERVICE_NAME="robot_bringup.service"
readonly START_SCRIPT_SOURCE="${SCRIPT_DIR}/start_robot.sh"
readonly SERVICE_SOURCE="${SCRIPT_DIR}/${SERVICE_NAME}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROBOT_USER="${1:-${SUDO_USER:-${USER:-}}}"

if (( $# > 1 )); then
    echo "Usage: $0 [robot-user]" >&2
    exit 2
fi

if [[ ! "${ROS_DISTRO}" =~ ^[a-z0-9]+$ ]]; then
    echo "Invalid ROS_DISTRO: ${ROS_DISTRO}" >&2
    exit 2
fi

if [[ -z "${ROBOT_USER}" ]] || ! getent passwd "${ROBOT_USER}" >/dev/null; then
    echo "Usage: $0 [robot-user]" >&2
    echo "The robot user must be an existing local account." >&2
    exit 2
fi

if [[ ! -f "${START_SCRIPT_SOURCE}" || ! -f "${SERVICE_SOURCE}" ]]; then
    echo "start_robot.sh and ${SERVICE_NAME} must be beside this installer." >&2
    exit 1
fi

ROBOT_HOME="$(getent passwd "${ROBOT_USER}" | cut -d: -f6)"
ROBOT_GROUP="$(id -gn "${ROBOT_USER}")"
START_SCRIPT_DESTINATION="${ROBOT_HOME}/start_robot.sh"

if (( EUID == 0 )); then
    SUDO=()
else
    command -v sudo >/dev/null || {
        echo "sudo is required when this installer is not run as root." >&2
        exit 1
    }
    SUDO=(sudo)
fi

SERVICE_TMP="$(mktemp)"
trap 'rm -f -- "${SERVICE_TMP}"' EXIT

escape_sed_replacement() {
    sed 's/[&|\\]/\\&/g' <<<"$1"
}

escaped_user="$(escape_sed_replacement "${ROBOT_USER}")"
escaped_home="$(escape_sed_replacement "${ROBOT_HOME}")"
escaped_distro="$(escape_sed_replacement "${ROS_DISTRO}")"

sed \
    -e "s|@ROBOT_USER@|${escaped_user}|g" \
    -e "s|@ROBOT_HOME@|${escaped_home}|g" \
    -e "s|@ROS_DISTRO@|${escaped_distro}|g" \
    "${SERVICE_SOURCE}" >"${SERVICE_TMP}"

echo "Installing rosbridge for ROS 2 ${ROS_DISTRO}..."
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "ros-${ROS_DISTRO}-rosbridge-suite"

echo "Installing the robot startup script for ${ROBOT_USER}..."
"${SUDO[@]}" install -o "${ROBOT_USER}" -g "${ROBOT_GROUP}" -m 0755 \
    "${START_SCRIPT_SOURCE}" "${START_SCRIPT_DESTINATION}"

echo "Installing and starting ${SERVICE_NAME}..."
"${SUDO[@]}" install -o root -g root -m 0644 \
    "${SERVICE_TMP}" "/etc/systemd/system/${SERVICE_NAME}"
"${SUDO[@]}" systemctl daemon-reload
"${SUDO[@]}" systemctl enable --now "${SERVICE_NAME}"

echo
"${SUDO[@]}" systemctl --no-pager --full status "${SERVICE_NAME}"
