# A small utility to log metadata about the CI run.
#   See https://docs.github.com/en/actions/learn-github-actions/environment-variables#default-environment-variables
#   for the complete list of available variables that could be logged

#!/bin/bash
set -e

metadata_filename="$1"
template='{"actor":"%s","event":"%s","base_ref":"%s","head_ref":"%s","ref":"%s","run_id":"%s","sha":"%s"}'
json_string=$(printf "$template" "$GITHUB_ACTOR" "$GITHUB_EVENT_NAME" "$GITHUB_BASE_REF" "$GITHUB_HEAD_REF" "$GITHUB_REF" "$GITHUB_RUN_ID" "$GITHUB_SHA")
echo "Build metadata: $json_string"
echo "$json_string" >> $metadata_filename
