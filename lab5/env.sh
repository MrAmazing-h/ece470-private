#!/usr/bin/env bash
# Source this file in each lab terminal after building the Lab 5 overlay.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source /home/xliu173/ece470/lab5/env.sh'
    exit 1
fi
LAB5_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAB5_WORKSPACE="$LAB5_REPO/ece470lab/lab_files"
if [[ ! -f "$LAB5_WORKSPACE/.lab5-install/local_setup.bash" ]]; then
    echo 'Build the Lab 5 overlay first; see lab5/README.md.'
    return 1
fi
pushd "$LAB5_WORKSPACE" >/dev/null || return 1
source ur3_setup.sh
LAB5_SETUP_STATUS=$?
if [[ "$LAB5_SETUP_STATUS" -eq 0 ]]; then
    source .lab5-install/local_setup.bash
    LAB5_SETUP_STATUS=$?
fi
popd >/dev/null || return 1
if [[ "$LAB5_SETUP_STATUS" -ne 0 ]]; then
    echo 'ROS setup failed. Check this workstation in ur3_setup.sh.'
    unset LAB5_SETUP_STATUS
    return 1
fi
unset LAB5_SETUP_STATUS
export LAB5_REPO LAB5_WORKSPACE
export LAB5_CONFIG="$LAB5_REPO/lab5/config.json"
export LAB5_LOG_DIR="$LAB5_REPO/lab5_runs"
echo "Lab 5 configuration: $LAB5_CONFIG"
