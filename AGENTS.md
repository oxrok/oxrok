# AGENTS.md

## Roadmap

Before pushing new work to `main`, update `docs/ROADMAP.md`:

- Remove anything the work completes or makes effectively done.
- Add any newly discovered open work, with the same level of detail as
  the existing entries.
- Keep the roadmap forward-looking. It is not a completion log.

Push the roadmap update with the work it describes, or before it.

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
