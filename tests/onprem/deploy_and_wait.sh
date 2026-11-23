#!/bin/bash
# Deploys a solution (built by intrinsic_solution) to the cluster given by
# $INTRINSIC_ORG and $CLUSTER_NAME. The deploy binary waits until the platform
# and the solution report ready, so a zero exit code means the solution is up.
set -euo pipefail

: "${INTRINSIC_ORG:?must be set, run this via onprem_e2e_test()}"
: "${CLUSTER_NAME:?must be set, run this via onprem_e2e_test()}"

SOLUTION_PATH="$(rlocation "${SOLUTION}")"

set -x
"${SOLUTION_PATH}" --org="${INTRINSIC_ORG}" --cluster="${CLUSTER_NAME}"
