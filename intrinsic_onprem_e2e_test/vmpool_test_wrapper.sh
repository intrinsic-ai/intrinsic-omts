#!/bin/bash
# Leases a VM from a VM pool, logs in with inctl and runs a test against it.
#
# Environment variables are set by intrinsic_onprem_e2e_test() in defs.bzl.
set -euo pipefail

# Bazel does not set HOME for tests by default, but inctl stores its login and
# config under $HOME/.config/intrinsic. Use the per-test temp dir so the test
# never touches the user's real inctl config.
export HOME="${TEST_TMPDIR:?must run under bazel test}/home"
mkdir -p "${HOME}"

# rlocationpath (in .bzl) + rlocation (here) yield absolute paths so that, e.g.,
# the test binary can safely cd into any dir and still use the tools.
BINARY_PATH=$(rlocation "${BINARY}")
INCTL=$(rlocation "${INCTL}")
# The toolchain uses `path`, which is relative to the execution directory.
# Removing the `external` prefix (if present) makes the path runfiles relative
# for resolution by rlocation.
JQ=$(rlocation "${JQ#external/}")
TEST_TIMEOUT=${TIMEOUT}
# Timeout for trying to lease a VM (in case there are no ready VMs available).
VM_LEASE_TIMEOUT=${VM_LEASE_TIMEOUT:-1800}
# Add some headroom for the setup before the test binary starts.
VM_LEASE_DURATION=$((TEST_TIMEOUT + 1800))

if [[ -n "${E2E_ORG}" ]]; then
  INTRINSIC_ORG="${E2E_ORG}"
fi
if [[ -z "${INTRINSIC_ORG:-}" ]]; then
  echo "INTRINSIC_ORG must be set (or the org attribute of intrinsic_onprem_e2e_test())." >&2
  exit 1
fi

VMPOOL="${E2E_POOL:-${INTRINSIC_VMPOOL:-default}}"
if [[ "${E2E_POOL_TYPE}" == "gpu" ]] && [[ "${VMPOOL}" != *"-gpu" ]]; then
  VMPOOL="${VMPOOL}-gpu"
fi

if [[ -z "${INTRINSIC_API_KEY:-}" ]]; then
  echo "INTRINSIC_API_KEY must be set to an API key for ${INTRINSIC_ORG}." >&2
  exit 1
fi

# NOTE inctl also accepts env vars: INTRINSIC_ORG = --org
export INTRINSIC_ORG
INTRINSIC_GCP_PROJECT="${INTRINSIC_ORG#*@}"

# Log in before enabling xtrace (below) so that the API key is never printed,
# and do not leak the key into the test binary's environment.
printf '%s\n' "${INTRINSIC_API_KEY}" | "${INCTL}" auth login
unset INTRINSIC_API_KEY

# Do not set earlier, rlocation (above) is very spammy.
set -x

# Adds a trap to the given signal. If a previous trap exists, it is replaced
# with a composite trap that first runs the new trap and then runs the previous
# trap.
function overtrap {
  trap="$1"
  sig="$(echo "$2" | tr 'a-z' 'A-Z')"
  cur="$(trap -p "$sig" | sed -nr "s/trap -- '(.*)' $sig\$/\1/p")"
  if [[ ${cur:+x} ]]; then
    trap "{ ${trap}; }; ${cur}" "$sig"
  else
    trap "${trap}" "$sig"
  fi
}

# Ensure signals trigger exit, which in turn runs EXIT traps.
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[$(date -u +'%Y-%m-%dT%H:%M:%SZ')] ======================================Phase: Leasing VM starts=================================================================="
# NOTE: `inctl vm` does not read INTRINSIC_ORG from the environment, so pass --org explicitly.
LEASE_OUTPUT="$("${INCTL}" vm lease --org="${INTRINSIC_ORG}" --pool="${VMPOOL}" --timeout="${VM_LEASE_TIMEOUT}s" --duration="${VM_LEASE_DURATION}s" --output=json)"
CLUSTER_NAME="$(echo "${LEASE_OUTPUT}" | "${JQ}" -r .instance)"

function cleanup_vm {
  echo "Returning VM ${CLUSTER_NAME}"
  "${INCTL}" vm return "${CLUSTER_NAME}" --org="${INTRINSIC_ORG}" || echo "Warning: Failed to return VM ${CLUSTER_NAME}"
}
overtrap cleanup_vm EXIT
echo "Leased VM ${CLUSTER_NAME}"
export CLUSTER_NAME

setup_start_time="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
logs_link_printed="false"
function print_logs_link {
  if [[ "${logs_link_printed}" != "true" ]]; then
    local end_time
    end_time="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
    local log_start_time="${start_time:-${setup_start_time}}"
    echo "Link to the logs: https://console.cloud.google.com/logs/query;query=resource.type%3D%22k8s_container%22%0Aresource.labels.cluster_name%3D%22${CLUSTER_NAME}%22;startTime=${log_start_time};endTime=${end_time}?project=${INTRINSIC_GCP_PROJECT}"
  fi
}
overtrap print_logs_link EXIT

# Write user config to make the SBL connect to the correct cluster.
mkdir -p "${HOME}/.config/intrinsic"
echo "{ \"selectedOrganization\": \"${INTRINSIC_ORG}\", \"selectedCluster\": \"${CLUSTER_NAME}\" }" \
  >"${HOME}/.config/intrinsic/user.config"

echo "[$(date -u +'%Y-%m-%dT%H:%M:%SZ')] ======================================Phase: Running Test Binary starts========================================================="
echo "Running test binary ${BINARY} with timeout ${TEST_TIMEOUT}s"
start_time="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
# Allow printing log links even if the test failed.
set +e
timeout "${TEST_TIMEOUT}s" "${BINARY_PATH}" "$@"
test_exit_code=$?
set -e
end_time="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
duration=$(($(date -d "$end_time" +%s) - $(date -d "$start_time" +%s)))
if [[ $test_exit_code -ne 0 ]]; then
  echo "FAILED running test ${BINARY} in ${duration}s"
else
  echo "SUCCEEDED running test ${BINARY} in ${duration}s"
fi

echo "Link to the logs: https://console.cloud.google.com/logs/query;query=resource.type%3D%22k8s_container%22%0Aresource.labels.cluster_name%3D%22${CLUSTER_NAME}%22;startTime=${start_time};endTime=${end_time}?project=${INTRINSIC_GCP_PROJECT}"
logs_link_printed="true"

exit $test_exit_code
