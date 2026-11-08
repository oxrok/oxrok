#!/usr/bin/env python3
"""Tests of tools/ci/changes.py, on a small made-up tree.

    python3 -I tools/ci/test_changes.py
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("changes", HERE / "changes.py")
changes = importlib.util.module_from_spec(_spec)
sys.modules["changes"] = changes
_spec.loader.exec_module(changes)

TREE = {
    # Models: one per driver, one with two drivers, one with a sub-driver.
    "tools/emu/alpha.sh": "EMU_MODEL=alpha\nEMU_DRIVER=alpha\n",
    "tools/emu/beta.sh": "EMU_MODEL=beta\nEMU_DRIVER=beta-la\n",
    "tools/emu/pair.sh": 'EMU_MODEL=pair\nEMU_DRIVER="scpi-a scpi-b"\n',
    "tools/emu/meter.sh": "EMU_MODEL=meter\nEMU_DRIVER=meter-x\n",
    "tools/emu/sniff.sh": "EMU_MODEL=sniff\nEMU_DRIVER=sniff\nEMU_TCP=1\n",
    "tools/emu/alpha.firmware": "",
    "emu/src/models/alpha.rs": "use crate::shells::cdc;\n",
    "emu/src/models/beta/mod.rs": "",
    "emu/src/models/beta/ep.rs": "",
    "emu/src/models/pair.rs": "use crate::shells::scpi::Engine;\n",
    "emu/src/models/meter.rs": "use crate::models::alpha;\n",
    "emu/src/models/sniff.rs": "",
    "emu/src/models/mod.rs": "",
    "emu/src/shells/mod.rs": "",
    "emu/src/shells/cdc.rs": "",
    "emu/src/shells/tcp.rs": "",
    "emu/src/shells/usbtmc.rs": "",
    "emu/src/shells/scpi.rs": "use crate::shells::tcp::Port;\n",
    "emu/src/lib.rs": "",
    "src/lib.rs": "",
    "src/core/mod.rs": "",
    "src/serial/mod.rs": "use crate::hid;\n",
    "src/hid/mod.rs": "",
    "src/scpi/mod.rs": "",
    "src/ftdi/mod.rs": "",
    "src/dmm/mod.rs": "",
    "src/lcr/mod.rs": "use crate::dmm;\n",
    "src/decoders/spi.rs": "",
    "src/pydecode/mod.rs": "",
    "src/cli/decode.rs": "",
    "src/cli/mod.rs": "",
    "src/drivers/alpha/mod.rs": "use crate::{core, ftdi};\n",
    "src/drivers/beta_la/mod.rs": "use crate::core;\n",
    "src/drivers/scpi_a/mod.rs": "use crate::scpi::Scpi;\n",
    "src/drivers/scpi_b/mod.rs": "use crate::scpi;\n",
    "src/drivers/meters/mod.rs": 'use crate::lcr;\nconst M: &[&str] = &["meter-x"];\n',
    "src/drivers/ols/mod.rs": "",
    "src/drivers/sniff/mod.rs": "use crate::drivers::ols::Proto;\n",
    "src/drivers/demo/mod.rs": "",
    "tests/emu/alpha/a.out": "",
    "tests/emu/alpha/b.out": "",
    "tests/emu/alpha/c.out": "",
    "tests/emu/alpha/d.out": "",
    "tests/emu/alpha/d.tx": "",
    "tests/emu/beta/a.out": "",
    "tests/emu/beta/b.oracle": "",
    "tests/emu/pair/a.out": "",
    "tests/emu/meter/a.out": "",
    "tests/emu/sniff/a.out": "",
}

LIB_ADD = """\
diff --git a/src/lib.rs b/src/lib.rs
--- a/src/lib.rs
+++ b/src/lib.rs
@@ -86,0 +87 @@ pub mod drivers {
+    pub mod alpha;
@@ -224,0 +226,2 @@ pub fn register_all(registry: &mut core::Registry) {
+    registry.register(Box::new(drivers::alpha::AlphaDriver));
+    drivers::beta_la::register_all(registry);
"""

LIB_OTHER = """\
--- a/src/lib.rs
+++ b/src/lib.rs
@@ -60 +60 @@
-pub const VERSION: &str = "0.2.0";
+pub const VERSION: &str = "0.2.1";
"""


class ChangesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        for rel, text in TREE.items():
            p = root / rel
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(text)
        cls.repo = changes.Repo(root)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def sel(self, *paths, lib_diff=None):
        return changes.classify(list(paths), self.repo, lib_diff)

    def assertSelects(self, sel, models, python=False):
        self.assertEqual(sorted(sel.models), sorted(models))
        self.assertEqual(sel.python, python)

    def test_docs_only_selects_nothing(self):
        s = self.sel("docs/ROADMAP.md", "CHANGELOG.md", "examples/x.rs", "Makefile",
                     "tools/oracle-patches/x.patch")
        self.assertSelects(s, [])
        self.assertFalse(s.all_models)

    def test_one_driver_selects_its_model(self):
        self.assertSelects(self.sel("src/drivers/alpha/mod.rs"), ["alpha"])

    def test_driver_name_with_dashes(self):
        self.assertSelects(self.sel("src/drivers/beta_la/mod.rs"), ["beta"])

    def test_sub_driver_found_by_literal(self):
        self.assertSelects(self.sel("src/drivers/meters/mod.rs"), ["meter"])

    def test_driver_used_by_another(self):
        self.assertSelects(self.sel("src/drivers/ols/mod.rs"), ["sniff"])

    def test_driver_without_model(self):
        self.assertSelects(self.sel("src/drivers/demo/mod.rs"), [])

    def test_model_files(self):
        s = self.sel(
            "tools/emu/alpha.firmware",
            "emu/src/models/beta/mod.rs",
            "tests/emu/pair/a.out",
        )
        self.assertSelects(s, ["alpha", "beta", "pair"])

    def test_model_with_two_drivers(self):
        self.assertSelects(self.sel("src/drivers/scpi_b/mod.rs"), ["pair"])

    def test_shared_module_selects_its_users(self):
        self.assertSelects(self.sel("src/scpi/mod.rs"), ["pair"])
        self.assertSelects(self.sel("src/ftdi/mod.rs"), ["alpha"])

    def test_shared_module_through_another(self):
        self.assertSelects(self.sel("src/dmm/mod.rs"), ["meter"])

    def test_shared_module_used_by_core(self):
        # serial uses hid, and every driver goes through serial.
        s = self.sel("src/hid/mod.rs")
        self.assertSelects(s, self.repo.models, python=True)

    def test_core_selects_everything(self):
        s = self.sel("src/core/mod.rs")
        self.assertTrue(s.all_models)
        self.assertSelects(s, self.repo.models, python=True)

    def test_emulator_model_used_by_another(self):
        self.assertSelects(self.sel("emu/src/models/alpha.rs"), ["alpha", "meter"])
        self.assertSelects(self.sel("emu/src/models/beta/ep.rs"), ["beta"])

    def test_shell_selects_its_models(self):
        self.assertSelects(self.sel("emu/src/shells/cdc.rs"), ["alpha", "meter"])
        self.assertSelects(self.sel("emu/src/shells/scpi.rs"), ["pair"])
        self.assertSelects(self.sel("emu/src/shells/usbtmc.rs"), [])

    def test_shell_through_another_and_tcp_serve(self):
        # scpi uses tcp; sniff is served with --tcp.
        self.assertSelects(self.sel("emu/src/shells/tcp.rs"), ["pair", "sniff"])

    def test_shared_emulator_code_selects_every_model(self):
        for path in ("emu/src/shells/mod.rs", "emu/src/models/mod.rs", "emu/src/lib.rs", "emu/Cargo.toml"):
            s = self.sel(path)
            self.assertEqual(sorted(s.models), sorted(self.repo.models), path)

    def test_unknown_path_selects_everything(self):
        for path in ("tools/diffcli.sh", "Cargo.lock", ".github/workflows/ci.yml", "tools/ci/changes.py", "foo"):
            s = self.sel(path)
            self.assertTrue(s.all_models and s.python, path)

    def test_decoders_select_python_only(self):
        for path in ("src/decoders/spi.rs", "src/pydecode/mod.rs", "src/cli/decode.rs",
                     "tests/decoder_spi.rs", "tests/support/mod.rs", "tests/fixtures/uart-hi.vcd",
                     "tests/fixtures/decoders/x.txt"):
            self.assertSelects(self.sel(path), [], python=True)

    def test_other_tests_select_nothing(self):
        self.assertSelects(self.sel("tests/emu_replay.rs", "tests/fixtures/alpha/emu-x.txt"), [])

    def test_lib_rs_driver_declarations(self):
        s = self.sel("src/lib.rs", lib_diff=LIB_ADD)
        self.assertSelects(s, ["alpha", "beta"])

    def test_lib_rs_other_change_selects_everything(self):
        s = self.sel("src/lib.rs", lib_diff=LIB_OTHER)
        self.assertTrue(s.all_models and s.python)
        self.assertTrue(self.sel("src/lib.rs").all_models)

    def test_lint_configuration(self):
        self.assertSelects(self.sel("clippy.toml"), [], python=True)

    def test_shards_cover_each_model_once(self):
        plan = changes.shards(set(self.repo.models), self.repo, 3)
        got = [m for s in plan for m in s["models"].split(",")]
        self.assertEqual(sorted(got), sorted(self.repo.models))
        self.assertEqual([s["shard"] for s in plan], list(range(len(plan))))
        weights = {m: self.repo.golden_weight(m) for m in self.repo.models}
        self.assertEqual(sum(s["weight"] for s in plan), sum(weights.values()))
        self.assertEqual(weights["alpha"], 4)  # .out and .oracle, not .tx

    def test_few_models_take_few_shards(self):
        self.assertEqual(len(changes.shards({"pair"}, self.repo, 6)), 1)
        self.assertEqual(changes.shards(set(), self.repo, 6), [])

    def test_lib_line_patterns(self):
        self.assertEqual(changes.lib_rs_drivers(LIB_ADD), {"alpha", "beta_la"})
        self.assertIsNone(changes.lib_rs_drivers(LIB_OTHER))


class MainTest(unittest.TestCase):
    """The command line, on the real tree."""

    def run_main(self, *argv):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "out"
            env = {"GITHUB_OUTPUT": str(out), "GITHUB_STEP_SUMMARY": str(Path(d) / "sum")}
            old = {k: os.environ.get(k) for k in env}
            os.environ.update(env)
            try:
                with open(os.devnull, "w") as null:
                    stdout, sys.stdout = sys.stdout, null
                    try:
                        changes.main(list(argv))
                    finally:
                        sys.stdout = stdout
            finally:
                for k, v in old.items():
                    if v is None:
                        os.environ.pop(k, None)
                    else:
                        os.environ[k] = v
            return dict(line.split("=", 1) for line in out.read_text().splitlines())

    def test_push_runs_everything(self):
        o = self.run_main("--event", "push")
        self.assertEqual(o["python"], "true")
        plan = json.loads(o["emu_shards"])
        self.assertEqual(len(plan), 6)
        n = len(list((changes.ROOT / "tools/emu").glob("*.sh")))
        self.assertEqual(sum(len(s["models"].split(",")) for s in plan), n)

    def test_label_runs_everything(self):
        o = self.run_main("--labels", "x, ci:full", "--files", "docs/USB.md")
        self.assertEqual(o["python"], "true")
        self.assertEqual(o["emu"], "true")

    def test_docs_pull_request_runs_nothing(self):
        o = self.run_main("--files", "docs/USB.md", "README.md")
        self.assertEqual(o, {"emu": "false", "emu_shards": "[]", "python": "false"})


if __name__ == "__main__":
    unittest.main()
