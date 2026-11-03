//! Recording files that hold several sessions, and replays served from
//! the environment (`OXROK_USB_REPLAY`, `OXROK_HID_REPLAY`).
//!
//! A program opens a device more than once (a scan that probes it, then
//! the acquisition), so a recorder that writes to a file appends each
//! session after the first one this process wrote there, and a replay
//! from the environment serves the sessions in the order they were
//! recorded, one per open. Every session a program was served must be
//! played to its end; [`finish_replays`] says when one was not, and the
//! command line tool exits with an error then.

use crate::core::{Error, Result};
use std::sync::Mutex;

/// Writes one recorded session to `path`: the first time this process
/// writes there it replaces the file, later sessions are appended.
pub fn write_session(path: &str, text: &str) -> std::io::Result<()> {
    static WRITTEN: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let mut written = WRITTEN.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    if written.iter().any(|p| p == path) {
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new().append(true).open(path)?;
        f.write_all(text.as_bytes())
    } else {
        std::fs::write(path, text)?;
        written.push(path.to_owned());
        Ok(())
    }
}

/// The sessions in `text`, each starting at a line equal to `magic`.
pub fn split_sessions(text: &str, magic: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        if line.trim() == magic || out.is_empty() {
            out.push(String::new());
        }
        let last = out.last_mut().expect("a session");
        last.push_str(line);
        last.push('\n');
    }
    out
}

/// The replay file named by the environment variable `var`, when set.
pub fn replay_file(var: &str) -> Option<String> {
    std::env::var(var).ok().filter(|f| !f.is_empty())
}

/// True when a replay from the environment stands in for the devices:
/// listings then show only the replayed ones.
pub fn replaying() -> bool {
    replay_file("OXROK_USB_REPLAY").is_some() || replay_file("OXROK_HID_REPLAY").is_some()
}

type Check = Box<dyn Fn() -> Result<()> + Send>;

struct Served {
    /// Per variable: the sessions not served yet.
    queues: Vec<(String, std::collections::VecDeque<String>)>,
    /// One check per session served.
    checks: Vec<Check>,
}

static SERVED: Mutex<Served> = Mutex::new(Served { queues: Vec::new(), checks: Vec::new() });

/// The first session of the replay file of `var`, read again for every
/// call (for a listing, which opens nothing).
pub fn first_session(var: &str, magic: &str) -> Result<Option<String>> {
    let Some(file) = replay_file(var) else { return Ok(None) };
    let text = std::fs::read_to_string(&file)
        .map_err(|e| Error::Arg(format!("{var}: cannot read {file}: {e}")))?;
    Ok(split_sessions(&text, magic).into_iter().next())
}

/// The next session of the replay file of `var` for an open; `check`
/// is called on it later by [`finish_replays`].
pub fn next_session(var: &str, magic: &str) -> Result<String> {
    let file = replay_file(var).ok_or_else(|| Error::Arg(format!("{var} is not set")))?;
    let mut served = SERVED.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    if !served.queues.iter().any(|(v, _)| v == var) {
        let text = std::fs::read_to_string(&file)
            .map_err(|e| Error::Arg(format!("{var}: cannot read {file}: {e}")))?;
        served.queues.push((var.to_owned(), split_sessions(&text, magic).into()));
    }
    let queue = &mut served.queues.iter_mut().find(|(v, _)| v == var).expect("queued").1;
    queue.pop_front().ok_or_else(|| {
        Error::Device(format!("replay: {file} has no more sessions for another open"))
    })
}

/// Registers the end-of-run check of a session served by
/// [`next_session`].
pub fn register(check: impl Fn() -> Result<()> + Send + 'static) {
    SERVED.lock().unwrap_or_else(std::sync::PoisonError::into_inner).checks.push(Box::new(check));
}

/// Fails when a session served from the environment was not played to
/// its end, or when sessions were left that no open asked for.
pub fn finish_replays() -> Result<()> {
    let served = SERVED.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    for check in &served.checks {
        check()?;
    }
    for (var, queue) in &served.queues {
        if !queue.is_empty() {
            return Err(Error::Device(format!(
                "replay: {} recorded sessions of {var} were not opened",
                queue.len()
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sessions_split_at_the_magic_line() {
        let text = "# m\na\n# m\nb\nc\n";
        assert_eq!(split_sessions(text, "# m"), ["# m\na\n", "# m\nb\nc\n"]);
        assert_eq!(split_sessions("# m\n", "# m"), ["# m\n"]);
    }
}
