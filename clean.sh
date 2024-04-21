#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

rm -rf "${SCRIPT_DIR}/build"
rm -rf "${SCRIPT_DIR}/install"
rm -rf "${SCRIPT_DIR}/log"
