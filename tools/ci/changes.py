#!/usr/bin/env python3
"""Which slow CI jobs a pull request needs, from the files it changes.

Two jobs are expensive: the emulator tier (oxrok against the goldens,
through vhci-hcd) and the Python decoder runs. A pull request that
changes one driver needs that driver's emulator models only; one that
changes no decoder code needs no Python run. Anything this script does
not recognise selects everything: it fails safe, not open.

    changes.py --base REV --head REV [--event NAME] [--labels a,b]
    changes.py --files PATH... (no git; for a dry run)

A push (to main), a manual run, or the `ci:full` label on the pull
request selects everything. With $GITHUB_OUTPUT set it writes
`emu`, `emu_shards` and `python` there, and a report to
$GITHUB_STEP_SUMMARY; otherwise it prints both.

`emu_shards` is a JSON list of {"shard": N, "models": "a,b,c"}, the
models balanced over at most --shards jobs by their golden case count.

The tests are tools/ci/test_changes.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[2]

# Changes to these select nothing: no code the slow jobs run.
IGNORED_PREFIXES = (
    "docs/",
    "examples/",
    "hil/",
    "website/",
    "tools/third-party/",
    # Applied to the sigrok oracle, which CI does not build (--no-oracle).
    "tools/oracle-patches/",
    "tools/attribution.d/",
    "tools/site-data/",
    ".github/workflows/release.yml",
    ".github/ISSUE_TEMPLATE/",
)
IGNORED_FILES = {
    "LICENSE",
    "NOTICE",
    "Makefile",
    "deny.toml",
    ".gitignore",
    ".gitattributes",
    "tools/attribution.py",
    "tools/libsigrok-drivers.txt",
}

# Lint configuration: the Python clippy job reads it, the emulators do not.
LINT_FILES = {"clippy.toml", "rustfmt.toml"}

# Top-level modules of src/ that every driver goes through.
CORE_MODULES = {"core", "usb", "serial", "cli", "formats"}
# Top-level modules of src/ only the decoders use.
DECODER_MODULES = {"decode", "decoders", "pydecode"}
# Files outside those that the Python runs exercise.
PYTHON_FILES = {"src/cli/decode.rs", "tests/decode_cli.rs"}

FULL_LABEL = "ci:full"


@dataclass
class Selection:
    """What a set of changes needs, and why."""

    all_models: bool = False
    models: set[str] = field(default_factory=set)
    python: bool = False
    reasons: list[tuple[str, str]] = field(default_factory=list)

    def everything(self, why: str, path: str = "") -> None:
        self.all_models = True
        self.python = True
        self.reasons.append((path, why))

    def note(self, path: str, why: str) -> None:
        self.reasons.append((path, why))


class Repo:
    """What the selection needs to know about the tree."""

    def __init__(self, root: Path = ROOT):
        self.root = root
        self.models = sorted(p.stem for p in (root / "tools/emu").glob("*.sh"))
        self.driver_dirs = sorted(
            p.name for p in (root / "src/drivers").iterdir() if p.is_dir()
        )
        self.module_dirs = sorted(
            p.name
            for p in (root / "src").iterdir()
            if p.is_dir() and p.name != "drivers"
        )
        self._sources: dict[str, str] = {}
        self._dir_models = self._models_by_driver_dir()
        self._refs = {d: self._crate_refs(f"src/drivers/{d}") for d in self.driver_dirs}
        self._module_refs = {m: self._crate_refs(f"src/{m}") for m in self.module_dirs}
        self.shells = sorted(
            p.stem for p in (root / "emu/src/shells").glob("*.rs") if p.stem != "mod"
        )
        self._tcp_models = {
            m
            for m in self.models
            if re.search(r"^EMU_TCP=", (root / f"tools/emu/{m}.sh").read_text(errors="replace"), re.M)
        }

    def text(self, rel: str) -> str:
        """Every .rs file under rel, concatenated."""
        if rel not in self._sources:
            base = self.root / rel
            files = sorted(base.rglob("*.rs")) if base.is_dir() else [base]
            self._sources[rel] = "\n".join(
                f.read_text(errors="replace") for f in files if f.is_file()
            )
        return self._sources[rel]

    def _crate_refs(self, rel: str) -> set[str]:
        """The `crate::X` and `crate::drivers::X` names a subtree uses."""
        src = self.text(rel)
        refs = set(re.findall(r"crate::([a-z0-9_]+)", src))
        for group in re.findall(r"crate::\{([^}]*)\}", src):
            refs.update(re.findall(r"\b([a-z0-9_]+)\b", group))
        drivers = set(re.findall(r"crate::drivers::([a-z0-9_]+)", src))
        return refs | {f"drivers::{d}" for d in drivers}

    def _models_by_driver_dir(self) -> dict[str, set[str]]:
        """The models whose EMU_DRIVER names a driver of each src/drivers dir.

        A driver name is the directory with - for _, or a sub-driver
        (`victor-dmm` in serial_dmm) found as a string literal there.
        """
        out: dict[str, set[str]] = {d: set() for d in self.driver_dirs}
        for model in self.models:
            script = (self.root / f"tools/emu/{model}.sh").read_text(errors="replace")
            m = re.search(r'^EMU_DRIVER="?([^"\n]*)"?', script, re.M)
            if not m:
                continue
            for token in m.group(1).split():
                d = token.replace("-", "_")
                if d in out:
                    out[d].add(model)
                    continue
                for cand in self.driver_dirs:
                    if f'"{token}"' in self.text(f"src/drivers/{cand}"):
                        out[cand].add(model)
        return out

    def models_of_drivers(self, dirs: set[str]) -> set[str]:
        """The models of those drivers and of the drivers that use them."""
        seen = set(dirs)
        while True:
            more = {
                d
                for d, refs in self._refs.items()
                if d not in seen and any(f"drivers::{s}" in refs for s in seen)
            }
            if not more:
                break
            seen |= more
        return set().union(*(self._dir_models.get(d, set()) for d in seen))

    def module_users(self, module: str) -> set[str]:
        """A shared module and the top-level modules that use it, transitively."""
        mods = {module}
        while True:
            more = {
                m
                for m, refs in self._module_refs.items()
                if m not in mods and refs & mods
            }
            if not more:
                break
            mods |= more
        return mods

    def drivers_using(self, mods: set[str]) -> set[str]:
        """The drivers that use any of the modules."""
        if mods & CORE_MODULES:
            return set(self.driver_dirs)
        return {d for d, refs in self._refs.items() if refs & mods}

    def emu_source(self, model: str) -> str:
        """The emulator model's source: emu/src/models/NAME.rs or NAME/."""
        d = f"emu/src/models/{model}"
        return self.text(d) if (self.root / d).is_dir() else self.text(f"{d}.rs")

    def model_users(self, models: set[str]) -> set[str]:
        """The models and the models whose source uses theirs."""
        seen = set(models)
        while True:
            idents = {m.replace("-", "_") for m in seen}
            more = {
                m
                for m in self.models
                if m not in seen
                and any(
                    re.search(rf"\b(?:models|super)::{i}\b", self.emu_source(m))
                    for i in idents
                )
            }
            if not more:
                return seen
            seen |= more

    def shell_users(self, shell: str) -> set[str]:
        """The models that use an emulator shell, directly or through another."""
        uses = lambda src, names: any(re.search(rf"\bshells::{n}\b", src) for n in names)
        shells = {shell}
        while True:
            more = {
                s
                for s in self.shells
                if s not in shells and uses(self.text(f"emu/src/shells/{s}.rs"), shells)
            }
            if not more:
                break
            shells |= more
        got = {m for m in self.models if uses(self.emu_source(m), shells)}
        if "tcp" in shells:
            # oxrok-emu serve --tcp: the scripts that set EMU_TCP.
            got |= self._tcp_models
        return self.model_users(got)

    def golden_weight(self, model: str) -> int:
        """How long a model runs, by its golden case count."""
        d = self.root / "tests/emu" / model
        if not d.is_dir():
            return 1
        return max(1, sum(1 for p in d.iterdir() if p.suffix in (".out", ".oracle")))


# A line of src/lib.rs that only declares or registers one driver.
LIB_DRIVER_LINE = re.compile(
    r"^\s*(?:pub mod ([a-z0-9_]+);"
    r"|(?:registry\.register\(Box::new\()?drivers::([a-z0-9_]+)::[A-Za-z0-9_:]+(?:\(registry\)|\)\));)\s*$"
)


def lib_rs_drivers(diff: str) -> set[str] | None:
    """The drivers a src/lib.rs diff adds or removes, or None when it does more."""
    found: set[str] = set()
    for line in diff.splitlines():
        if line.startswith(("+++", "---")) or not line.startswith(("+", "-")):
            continue
        body = line[1:]
        if not body.strip():
            continue
        m = LIB_DRIVER_LINE.match(body)
        if not m:
            return None
        found.add(m.group(1) or m.group(2))
    return found


def classify(
    paths: list[str],
    repo: Repo,
    lib_diff: str | None = None,
) -> Selection:
    """What the changed paths need. lib_diff is src/lib.rs's diff, if it changed."""
    sel = Selection()
    models = set(repo.models)
    for path in paths:
        p = PurePosixPath(path)
        parts = p.parts
        if path in IGNORED_FILES or path.startswith(IGNORED_PREFIXES) or p.suffix == ".md":
            sel.note(path, "ignored")
        elif path in LINT_FILES:
            sel.python = True
            sel.note(path, "lint configuration: python clippy")
        elif path.startswith("tools/emu/") and len(parts) == 3 and p.stem in models:
            sel.models.add(p.stem)
            sel.note(path, f"model {p.stem}")
        elif path.startswith("emu/src/models/") and len(parts) >= 4:
            name = PurePosixPath(parts[3]).stem
            if name in models:
                got = repo.model_users({name})
                sel.models |= got
                sel.note(path, "model " + ", ".join(sorted(got)))
            else:
                sel.everything("emulator code shared by models", path)
        elif path.startswith("tests/emu/") and len(parts) >= 4 and parts[2] in models:
            sel.models.add(parts[2])
            sel.note(path, f"goldens of {parts[2]}")
        elif (
            path.startswith("emu/src/shells/")
            and len(parts) == 4
            and p.suffix == ".rs"
            and p.stem in repo.shells
        ):
            got = repo.shell_users(p.stem)
            sel.models |= got
            sel.note(path, f"shell {p.stem}: " + (", ".join(sorted(got)) or "no model"))
        elif path.startswith("emu/"):
            sel.all_models = True
            sel.note(path, "emulator code shared by models: every model")
        elif path.startswith("src/drivers/") and len(parts) >= 4:
            d = parts[2]
            got = repo.models_of_drivers({d})
            sel.models |= got
            sel.note(path, f"driver {d}: " + (", ".join(sorted(got)) or "no emulator model"))
        elif path == "src/lib.rs":
            drivers = lib_rs_drivers(lib_diff) if lib_diff is not None else None
            if drivers is None:
                sel.everything("src/lib.rs beyond a driver's declaration", path)
            else:
                got = repo.models_of_drivers(drivers)
                sel.models |= got
                sel.note(
                    path,
                    f"declares {', '.join(sorted(drivers)) or 'nothing'}: "
                    + (", ".join(sorted(got)) or "no emulator model"),
                )
        elif path in PYTHON_FILES or (
            len(parts) >= 3 and parts[0] == "src" and parts[1] in DECODER_MODULES
        ):
            sel.python = True
            sel.note(path, "decoders: python")
        elif len(parts) >= 3 and parts[0] == "src" and parts[1] in CORE_MODULES:
            sel.everything(f"src/{parts[1]} is used by every driver", path)
        elif len(parts) >= 3 and parts[0] == "src" and parts[1] in repo.module_dirs:
            mods = repo.module_users(parts[1])
            got = repo.models_of_drivers(repo.drivers_using(mods))
            sel.models |= got
            uses = ", ".join(sorted(got)) or "no emulator model"
            if mods & (DECODER_MODULES | CORE_MODULES):
                sel.python = True
                uses += "; python"
            sel.note(path, f"shared module {parts[1]}: {uses}")
        elif path.startswith("tests/"):
            if path.startswith("tests/decoder_") or path.startswith("tests/support/"):
                sel.python = True
                sel.note(path, "decoder tests: python")
            elif path.startswith("tests/fixtures/"):
                if len(parts) >= 4 and parts[2] != "decoders":
                    sel.note(path, "replay fixture: the test job")
                else:
                    sel.python = True
                    sel.note(path, "decoder fixture: python")
            elif len(parts) == 2:
                sel.note(path, "test of the test job")
            else:
                sel.everything("unrecognised test path", path)
        else:
            sel.everything("not recognised: everything", path)
    if sel.all_models:
        sel.models = set(models)
    return sel


def shards(models: set[str], repo: Repo, max_shards: int) -> list[dict]:
    """The models over the fewest jobs that keep each as short as a full run's."""
    if not models:
        return []
    weight = {m: repo.golden_weight(m) for m in models}
    full = sum(repo.golden_weight(m) for m in repo.models)
    target = full / max_shards
    n = max(1, min(max_shards, len(models), math.ceil(sum(weight.values()) / target)))
    bins: list[tuple[int, list[str]]] = [(0, []) for _ in range(n)]
    for m in sorted(models, key=lambda m: (-weight[m], m)):
        i = min(range(n), key=lambda i: bins[i][0])
        bins[i] = (bins[i][0] + weight[m], bins[i][1] + [m])
    return [
        {"shard": i, "models": ",".join(sorted(ms)), "weight": w}
        for i, (w, ms) in enumerate(bins)
    ]


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout


def report(sel: Selection, plan: list[dict], why: str) -> str:
    lines = ["## CI selection", "", why, ""]
    lines.append(f"- python decoder jobs: {'run' if sel.python else 'skipped'}")
    if plan:
        n = sum(len(s["models"].split(",")) for s in plan)
        lines.append(f"- emulator models: {n} over {len(plan)} job(s)")
    else:
        lines.append("- emulator models: none, tier skipped")
    if sel.reasons:
        lines += ["", "| path | selects |", "| --- | --- |"]
        for path, reason in sel.reasons[:200]:
            lines.append(f"| `{path}` | {reason} |" if path else f"| | {reason} |")
        if len(sel.reasons) > 200:
            lines.append(f"| ... | {len(sel.reasons) - 200} more |")
    if plan:
        lines += ["", "| shard | weight | models |", "| --- | --- | --- |"]
        for s in plan:
            lines.append(f"| {s['shard']} | {s['weight']} | {s['models'].replace(',', ', ')} |")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--base", help="the revision the change is against")
    ap.add_argument("--head", help="the revision with the change")
    ap.add_argument("--files", nargs="*", help="changed paths, instead of --base/--head")
    ap.add_argument("--event", default="pull_request", help="the GitHub event name")
    ap.add_argument("--labels", default="", help="comma-separated pull request labels")
    ap.add_argument("--shards", type=int, default=6, help="the most emulator jobs")
    args = ap.parse_args(argv)

    repo = Repo()
    labels = {s.strip() for s in args.labels.split(",") if s.strip()}
    if args.event != "pull_request":
        sel = Selection()
        sel.everything(f"a {args.event} event runs everything")
        why = f"Event `{args.event}`: everything runs."
    elif FULL_LABEL in labels:
        sel = Selection()
        sel.everything(f"the {FULL_LABEL} label")
        why = f"Label `{FULL_LABEL}`: everything runs."
    else:
        if args.files is not None:
            paths, lib_diff = args.files, None
            if "src/lib.rs" in paths and args.base and args.head:
                lib_diff = git("diff", "-U0", args.base, args.head, "--", "src/lib.rs")
        elif args.base and args.head:
            paths = git("diff", "--name-only", "--no-renames", args.base, args.head).split()
            lib_diff = (
                git("diff", "-U0", args.base, args.head, "--", "src/lib.rs")
                if "src/lib.rs" in paths
                else None
            )
        else:
            ap.error("need --base and --head, or --files")
        sel = classify(paths, repo, lib_diff)
        why = f"{len(paths)} changed file(s)."
        if not paths:
            sel.note("", "no changed files")
    if sel.all_models:
        sel.models = set(repo.models)
    plan = shards(sel.models, repo, args.shards)

    outputs = {
        "emu": "true" if plan else "false",
        "emu_shards": json.dumps(plan, separators=(",", ":")),
        "python": "true" if sel.python else "false",
    }
    text = report(sel, plan, why)
    out_path = os.environ.get("GITHUB_OUTPUT")
    if out_path:
        with open(out_path, "a") as f:
            for k, v in outputs.items():
                f.write(f"{k}={v}\n")
        summary = os.environ.get("GITHUB_STEP_SUMMARY")
        if summary:
            with open(summary, "a") as f:
                f.write(text)
    sys.stdout.write(text)
    for k, v in outputs.items():
        print(f"{k}={v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
