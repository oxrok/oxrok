# AGENTS.md

## How main changes

`main` changes only through a pull request whose required checks passed:
the branch rules allow no direct push, by anyone. Work on a branch, open
the PR, let CI run, and the owner merges it (an org admin may merge their
own PR without a review; a PR by the bot needs the owner's review). A
batch of ports merged locally is pushed as `batch/<name>` and opened as a
PR the same way, with `merge` so the per-port commits stay. The rules and
their reasons are in oxrok/management, `docs/HARDENING.md`.

## Roadmap

Before opening a pull request with new work, update `docs/ROADMAP.md`:

- Remove anything the work completes or makes effectively done.
- Add any newly discovered open work, with the same level of detail as
  the existing entries.
- Keep the roadmap forward-looking. It is not a completion log.

Put the roadmap update in the pull request with the work it describes,
or in one before it.

## Testing

The default check of a driver change is oxrok against the committed
goldens on the emulator: `sh tools/diffcli.sh --emu-model MODEL` (no
sigrok needed; the first line it prints names the mode). Run it with
`--oracle` (sigrok-cli and libsigrok alongside, after
`. ~/work/sigrok/env.sh`) only for a new port or to verify an upstream
sigrok change. When a change alters goldens on purpose, re-record them
with `EMU_BLESS=1 sh tools/diffcli.sh --emu-model MODEL`, read each
diff, and say in the commit message why they changed.

## Vendor firmware

Never commit vendor firmware to this repo or any other: FX2 firmware,
FPGA bitstreams, `.ccf`/`.rbf`/`.fw`/`.bitstream` files, vendor
installers, or anything extracted from them, whole or in part. That
includes test fixtures, emulator models and release assets; the emulator
uses stand-in images.

The one place vendor firmware is stored is
[oxrok/oxrok-firmware](https://github.com/oxrok/oxrok-firmware),
inherited from sigrok-firmware, and only firmware whose vendor has
allowed its distribution in writing, with that permission recorded next
to it. Everything else is fetched by an extractor (oxrok-util) on the
user's machine. Check `git diff --cached --stat` before committing after
running an extractor.

## Upstream sigrok work

Upstream pull requests worth porting are tracked on the
[Upstream intake](https://github.com/orgs/oxrok/projects/1) board
(issues labelled `upstream`), managed from
[oxrok/management](https://github.com/oxrok/management), whose
AGENTS.md has the triage rules.
