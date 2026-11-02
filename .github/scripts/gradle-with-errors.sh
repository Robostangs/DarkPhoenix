#!/usr/bin/env bash

set -o pipefail

log_file="$(mktemp)"
trap 'rm -f "$log_file"' EXIT

./gradlew "$@" 2>&1 | tee "$log_file"
gradle_status="${PIPESTATUS[0]}"

runtime_error=false
while IFS= read -r line; do
  if [[ "$line" == *"Error at "* || "$line" == *"The robot program quit unexpectedly"* ]]; then
    printf '::error::%s\n' "$line"
    if [[ "$line" == *"The robot program quit unexpectedly"* ]]; then
      runtime_error=true
    fi
  fi
done < "$log_file"

if [[ "$gradle_status" -ne 0 ]]; then
  printf '::error::Gradle command failed with exit code %s: %s\n' "$gradle_status" "$*"
  exit "$gradle_status"
fi

if [[ "$runtime_error" == true ]]; then
  exit 1
fi
