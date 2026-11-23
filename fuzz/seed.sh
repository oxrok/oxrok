#!/bin/sh
# Seeds the corpora of the targets that read a file format from the
# fixtures of the test suite, so that libFuzzer starts from files that
# parse and not from nothing:
#
#   sh fuzz/seed.sh            every such target
#   sh fuzz/seed.sh sr_open    one target; a target with no seeds is fine
#
# The corpora (fuzz/corpus/TARGET) are not committed. Session files over
# 16 KiB stay out: libFuzzer mutates inputs as long as the longest seed,
# and the long ones are slow and add no structure the short ones lack.
set -eu

cd "$(dirname "$0")/.."

seed() {
	target=$1
	shift
	mkdir -p "fuzz/corpus/$target"
	# Named by content: two fixtures of one name both stay.
	find tests/fixtures -type f "$@" | while IFS= read -r file; do
		cp "$file" "fuzz/corpus/$target/$(cksum <"$file" | tr ' ' '-')"
	done
	echo "$target: $(find "fuzz/corpus/$target" -type f | wc -l | tr -d ' ') files in the corpus"
}

for target in ${*:-input_vcd sr_open}; do
	case $target in
	input_vcd) seed input_vcd -name '*.vcd' ;;
	sr_open) seed sr_open -name '*.sr' -size -33 ;;
	*) echo "$target: no seeds" ;;
	esac
done
