#!/bin/sh
# The tests nextest's ci profile (.config/nextest.toml) had to run twice
# to see pass, from its JUnit report: a warning each, and a list in the
# job's summary. It fails nothing: a flaky test passed, and what it says
# is that the test (or the code) has a race worth a look.
#
#   sh tools/ci/flaky.sh [JUNIT.XML]
#
# In the report a flaky test is a <testcase> with <flakyFailure> children.
set -u
junit=${1:-target/nextest/ci/junit.xml}
[ -f "$junit" ] || { echo "flaky.sh: no $junit (nextest did not get to run)"; exit 0; }
flaky=$(awk '
    /<testcase / {
        name = ""; class = ""; seen = 0
        if (match($0, / name="[^"]*"/)) name = substr($0, RSTART + 7, RLENGTH - 8)
        if (match($0, / classname="[^"]*"/)) class = substr($0, RSTART + 12, RLENGTH - 13)
    }
    /<flakyFailure/ && !seen { seen = 1; print class " " name }' "$junit")
if [ -z "$flaky" ]; then
    echo "no flaky test"
    exit 0
fi
echo "$flaky" | while read -r class name; do
    echo "FLAKY $class $name"
    [ -z "${GITHUB_ACTIONS:-}" ] || echo "::warning title=flaky test::$class $name passed on its second try"
done
if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
        echo "### Flaky tests"
        echo
        echo "Failed, then passed on a second try (${GITHUB_JOB:-this job}):"
        echo
        echo "$flaky" | while read -r class name; do echo "- \`$class\` \`$name\`"; done
    } >>"$GITHUB_STEP_SUMMARY"
fi
