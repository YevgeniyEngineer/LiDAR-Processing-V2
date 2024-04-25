#!/bin/bash

# Absolute path to the root of the project
ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )

# Remove the build, install, and log directories
rm -rf "${ROOT_DIR}/build"
rm -rf "${ROOT_DIR}/install"
rm -rf "${ROOT_DIR}/log"
