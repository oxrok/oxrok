# Licensing

Not legal advice. The policy oxrok follows:

- The whole workspace is GPL-3.0-or-later. The drivers, the parsers and
  the file formats reproduce libsigrok's and sigrok-cli's behaviour
  closely enough to be ports of GPL code, the LogicPort driver stays
  under the GPL its author chose, and a binary that links them is GPL
  as a whole whatever the other crates said. One license keeps the
  provenance bookkeeping out of every manifest.
- Firmware loaded onto a device is a separate program; shipping it next
  to the binary is aggregation. Proprietary firmware is never shipped,
  the user installs it from the vendor. The bench firmware under `hil/`
  is part of this repository and GPL-3.0-or-later too.
- Python decoders from the sigrok distribution are loaded at run time
  from the user's install or a decoders directory. The release archives
  ship libsigrokdecode 0.5.3's `decoders` directory unchanged next to
  the binary, with libsigrokdecode's `COPYING` (GPL-3.0); like the
  binary they are GPL, and the archive is an aggregate of the two.
- The `rustpython` feature builds RustPython (MIT) and its standard
  library, which is CPython's (PSF License), into the binary; both are
  compatible with the GPL.
- "sigrok" is the name of another project. oxrok describes itself as
  sigrok compatible and never uses the name as its own.

## Attribution of the sigrok counterparts

Every Rust source that corresponds to a sigrok file starts with a
`Counterpart in sigrok` comment naming that file, the copyright holders
from its header, and everyone else who committed to it according to
the libsigrok and sigrok-cli git histories, as credit to the people
whose work defined the behaviour oxrok reproduces. The comment also
says how the Rust file relates to it: a port of the counterpart (the
usual case), a port by the same author (the LogicPort driver), or a
file written from vendor documentation with the sigrok code as the
reference for scope (the FTDI layer).

`tools/attribution.py` generates these comments from the checkouts, so
they can be refreshed when the counterparts change. Names are kept as
the git history spells them; `--ascii` transliterates them. It
rewrites only what it is told to: `--only FILE...` for named files
(a new model's header) or `--all`, and `--check` lists the headers
that would change without writing. A counterpart missing from the
checkouts stops the run rather than dropping its credits (the
LogicPort's sources are only in the fork, so `--all` needs that
checkout).
