// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later

//! Starting another program as glib's `g_spawn_*` does: with nothing of ours open
//! in it beyond stdin, stdout and stderr. glib closes every descriptor above 2 in
//! the child unless asked not to (`G_SPAWN_LEAVE_DESCRIPTORS_OPEN`); Rust's
//! `Command` passes on every descriptor that lacks `FD_CLOEXEC`. Rust opens its
//! own with the flag, so what would leak is what oxrok was given when it started
//! (a CI runner hands over two). This is the one module besides `pydecode` with
//! unsafe code: one call that names a descriptor by its number.

/// Marks every descriptor above 2 that oxrok has open close-on-exec, so that a
/// program started next gets none of them. Call it right before the spawn.
///
/// It does this in the parent, not in a `pre_exec` hook in the child: closing
/// there would also close the pipe on which `Command` learns that an `exec`
/// failed. A descriptor oxrok inherited and meant for no one is then closed at
/// any exec, which is what glib's programs see too. A no-op where the system
/// lists no open descriptors (`/proc/self/fd`, `/dev/fd`) and off Unix.
pub fn close_inherited_fds() {
    #[cfg(unix)]
    for fd in open_fds() {
        set_cloexec(fd);
    }
}

/// The descriptors above 2 the process has open, as the system lists them.
#[cfg(unix)]
fn open_fds() -> Vec<i32> {
    for dir in ["/proc/self/fd", "/dev/fd"] {
        if let Ok(list) = std::fs::read_dir(dir) {
            return list
                .filter_map(|e| e.ok()?.file_name().to_str()?.parse::<i32>().ok())
                .filter(|fd| *fd > 2)
                .collect();
        }
    }
    Vec::new()
}

/// Sets `FD_CLOEXEC` on descriptor `fd`, if it is open.
#[cfg(unix)]
fn set_cloexec(fd: i32) {
    use rustix::io::{FdFlags, fcntl_getfd, fcntl_setfd};
    // SAFETY: `borrow_raw` asks that `fd` stay open while the borrow lives, which
    // here is two `fcntl` calls on the flags of a descriptor number. They touch
    // no memory. If `fd` was the listing's own (closed by now), they fail with
    // EBADF; if another thread has meanwhile closed it or opened a new one under
    // the number, the new one gets FD_CLOEXEC, which Rust sets on every
    // descriptor it opens anyway.
    let borrowed = unsafe { std::os::fd::BorrowedFd::borrow_raw(fd) };
    if let Ok(flags) = fcntl_getfd(borrowed)
        && !flags.contains(FdFlags::CLOEXEC)
    {
        let _ = fcntl_setfd(borrowed, flags | FdFlags::CLOEXEC);
    }
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use rustix::io::{FdFlags, fcntl_getfd, fcntl_setfd};
    use std::os::fd::AsFd as _;

    /// A descriptor without `FD_CLOEXEC`, as a parent hands one over, gets it.
    #[test]
    fn an_inherited_descriptor_is_marked() {
        let file = std::fs::File::open("/dev/null").unwrap();
        fcntl_setfd(file.as_fd(), FdFlags::empty()).unwrap();
        assert!(!fcntl_getfd(file.as_fd()).unwrap().contains(FdFlags::CLOEXEC));
        close_inherited_fds();
        assert!(fcntl_getfd(file.as_fd()).unwrap().contains(FdFlags::CLOEXEC));
    }
}
