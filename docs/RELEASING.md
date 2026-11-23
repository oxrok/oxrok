# Releasing

A release is a version in `Cargo.toml`, a dated section of
`CHANGELOG.md`, a `v<version>` tag, a GitHub release with the archives
of [README.md](../README.md#release-artefacts), and the crate on
crates.io. The tag starts the `Release binaries` workflow
(`.github/workflows/release.yml`), which builds everything and leaves a
draft release; publishing the draft and the crate are done by hand, in
that order.

No release is public before it has every archive: the draft is filled in
one step, after the source tarball has built with no network and every
target has passed its smoke test.

## 1. The release commit

One pull request, named `Release <version>`, as every other change to
`main`:

- `CHANGELOG.md`: the `[Unreleased]` entries become
  `## [<version>] - YYYY-MM-DD`, with a summary on top and the known
  issues. An empty `[Unreleased]` stays above it. The section is the
  text of the GitHub release, as it is.
- The version, everywhere it is written: `Cargo.toml` (the workspace
  package and the `oxrok` workspace dependency), `Cargo.lock`
  (`cargo check`), the `cargo install` lines of `README.md` and of the
  crate documentation in `src/lib.rs`, the status line of
  `docs/ROADMAP.md`, `%global upstream_version` and `Version:` in
  `packaging/fedora/oxrok.spec` (RPM writes a pre-release with a tilde:
  `0.3.0~alpha.1`), and the fixtures that print it (`grep -rl "^oxrok
  <old version>" tests/fixtures`).
- `cargo publish --dry-run --locked` packages and builds the crate. It
  refuses a tree with files git does not know; do not pass
  `--allow-dirty`, which would publish them.

`python3 -I tools/ci/release.py --tag v<version>` says whether the tag
would pass the workflow's first job: the tag is `v` and the version of
`Cargo.toml`, and the changelog has that version's dated section.

## 2. The tag

On the merged release commit, an annotated tag, pushed:

```sh
git switch main && git pull --ff-only
git tag -a v<version> -m "oxrok <version>"
git push origin v<version>
```

A version with a hyphen (`0.3.0-alpha.1`) is a pre-release, and the
draft is marked as one.

## 3. The workflow

The push starts `Release binaries`. Its jobs, with read access only:

- `tag, version and changelog` stops the run when they disagree, before
  anything is built.
- The vendored source tarball, then that tarball built with the network
  off.
- One archive per target, each unpacked and run.

Then `draft release`, the one job that can write. It builds nothing: it
checks each archive against the `.sha256` its build wrote, writes
`SHA256SUMS` over all of them, attests their build provenance, and
creates the release as a draft with every file, in one step. It refuses
a tag that has a release already, draft or published: a release is never
written twice.

A run takes 30 to 55 minutes, most of it the macOS x86_64 build and
waiting for runners.

## 4. Publish the draft

Read the draft on the releases page: the notes, six archives, six
`.sha256` files and `SHA256SUMS`. Then publish it there, or:

```sh
gh release edit v<version> --draft=false
```

## 5. crates.io

By hand, last, from a clean checkout of the tag:

```sh
git switch --detach v<version>
git status --short        # nothing
cargo publish --locked
```

It is not automated, on purpose: a published crate version cannot be
replaced, only yanked, so it goes out once the GitHub release is seen to
be right. `tests/package.rs` pins what the crate may contain.

## When something fails

- **The first job refuses the tag.** Nothing was built or published. If
  the tag is wrong, delete it (`git push origin :v<version>`, and
  locally), fix what is wrong in a pull request, and tag again. Only
  before the release is published: a tag that has been public stays.
- **A build fails for a reason outside the tree** (a runner, a
  download). Re-run the failed jobs from the run's page; `draft release`
  runs once all of them have passed.
- **A build fails because of the tree.** Fix it on `main` and release
  the next version, or, when nothing was published, move the tag as
  above.
- **The draft is wrong, or the run is wanted again.** Delete the draft
  (`gh release delete v<version> --yes`, which keeps the tag) and start
  the workflow on the tag: Actions, `Release binaries`, `Run workflow`,
  and the tag in place of the branch.
- **A release was published by hand before the workflow finished.** The
  `draft release` job fails and attaches nothing. Delete that release,
  keeping the tag, and re-run the failed job.

## A test build

`Run workflow` on a branch builds every archive and the source tarball
and leaves them as artifacts of the run. Nothing is checked against a
tag, attested or released.

## Checking a download

```sh
sha256sum --check --ignore-missing SHA256SUMS
gh attestation verify oxrok-<version>-<target>.tar.gz --repo oxrok/oxrok
```

The second says that the file was built by this repository's release
workflow, and from which commit. Releases up to 0.3.0-alpha.1 have the
`.sha256` files only.
