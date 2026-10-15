//! `rpi-loader completions` against the real binary.
//!
//! The script itself is clap_complete's to generate, so there is nothing
//! to test about its contents -- what is worth guarding is the wiring
//! around it, and that has two failure modes neither the type system nor
//! `--help` would catch.
//!
//! The first is a completion that names the wrong command. It registers
//! against a binary name, and a script naming something other than the
//! binary installs cleanly, sources cleanly, and completes nothing.
//!
//! The second is the subcommand quietly falling out of the tree it
//! describes. The script is generated from the same `Command` clap builds
//! for parsing, so a subcommand added without a completion entry is not
//! possible -- but a subcommand that stops being generated, because the
//! generation moved or the wrong `Command` was handed to it, looks
//! exactly like success from the exit status.
//!
//! The third arrived with the `--device` wrapper. It delegates to clap's
//! generated function by name, and `write_completions` drops the wrapper
//! rather than emit one whose fall-through is missing -- correct, since a
//! broken delegation would take every other completion down with it, but
//! it degrades to the old behaviour without saying so. The test below is
//! what makes that degradation loud.

use std::process::{Command, Output};

/// Runs the binary with the given arguments.
fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rpi-loader"))
        .args(args)
        .output()
        .expect("failed to run rpi-loader")
}

/// Stdout as text, after asserting the command succeeded.
fn stdout_of(args: &[&str]) -> String {
    let output = run(args);
    assert!(
        output.status.success(),
        "rpi-loader {args:?} failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).expect("completion script is not UTF-8")
}

#[test]
fn bash_script_registers_against_this_binary() {
    let script = stdout_of(&["completions", "bash"]);

    // The line that makes the whole file do anything. `-F` names the
    // function, and the last word is the command being completed: that
    // word is what has to match the installed binary.
    let registration = script
        .lines()
        .find(|line| line.trim_start().starts_with("complete -F"))
        .expect("no `complete -F` line: nothing would register the completion");
    assert!(
        registration.split_whitespace().next_back() == Some("rpi-loader"),
        "completion registers against the wrong command: {registration}"
    );
}

#[test]
fn bash_script_covers_the_subcommands() {
    let script = stdout_of(&["completions", "bash"]);

    // A spread rather than all of them: one that opens a port, one that
    // does not, the hyphenated spelling (which the generated function
    // names mangle, so this checks the user-facing form survives), and
    // the subcommand generating the script, which is the one most likely
    // to be forgotten.
    for subcommand in ["boot", "bundle", "sd-write", "eeprom-read", "completions"] {
        assert!(
            script.contains(subcommand),
            "`{subcommand}` is missing from the completion script"
        );
    }
}

#[test]
fn every_subcommand_the_script_dispatches_to_has_a_branch() {
    let script = stdout_of(&["completions", "bash"]);

    // The bash script is two halves that have to agree on a name: a
    // dispatcher that walks the typed words and sets `cmd="<label>"`, and
    // a second `case` whose arms are those labels and which is where each
    // subcommand's options live. clap_complete builds the two halves
    // separately, and has been seen to mangle a hyphenated binary name
    // differently in each -- leaving no arm reachable, so that everything
    // past the first word completes to nothing.
    //
    // Checking the invariant rather than that one bug: every label the
    // dispatcher can set must exist as an arm.
    let dispatched: Vec<String> = script
        .lines()
        .filter_map(|line| line.trim().strip_prefix("cmd=\""))
        .filter_map(|rest| rest.strip_suffix('"'))
        .filter(|label| !label.is_empty())
        .map(str::to_string)
        .collect();
    assert!(
        dispatched.len() > 10,
        "found only {} dispatch labels; the script's shape has changed \
         enough that this test is no longer checking anything",
        dispatched.len()
    );

    for label in &dispatched {
        assert!(
            script.contains(&format!("\n        {label})")),
            "the dispatcher can set cmd=\"{label}\" but no case arm \
             matches it, so that subcommand completes to nothing"
        );
    }
}

#[test]
fn bash_script_completes_device_from_live_ports() {
    let script = stdout_of(&["completions", "bash"]);

    // The wrapper is only emitted when clap's own function was found to
    // delegate to, so its absence means that lookup failed and `--device`
    // has quietly gone back to completing every filename on the machine.
    assert!(
        script.contains("_rpi_loader_device_complete()"),
        "the --device wrapper is missing: clap_complete's generated \
         function was not found to delegate to, so the wrapper was \
         dropped. Check how its bash generator names that function."
    );

    // It has to ask the binary, not guess: the ports are a runtime fact.
    assert!(
        script.contains("rpi-loader list"),
        "the --device wrapper does not consult `rpi-loader list`"
    );

    // And the registration has to be the wrapper's, not clap's -- both
    // `complete -F` lines are in the file, and the last one wins.
    let last = script
        .lines()
        .rfind(|line| line.trim_start().starts_with("complete -F"))
        .expect("no `complete -F` line at all");
    assert!(
        last.contains("_rpi_loader_device_complete"),
        "the wrapper is defined but not registered; clap's own \
         completion is still the active one: {last}"
    );
}

#[test]
fn only_bash_is_wrapped() {
    // The wrapper is bash, so every other shell must come through as
    // clap_complete wrote it -- a stray `complete -F` in a zsh or fish
    // script would be a syntax error in that shell rather than a
    // degraded completion.
    for shell in ["zsh", "fish", "elvish", "powershell"] {
        let script = stdout_of(&["completions", shell]);
        assert!(
            !script.contains("_rpi_loader_device_complete"),
            "the bash --device wrapper leaked into the {shell} script"
        );
    }
}

#[test]
fn shell_defaults_to_bash() {
    assert_eq!(
        stdout_of(&["completions"]),
        stdout_of(&["completions", "bash"]),
        "`completions` with no shell should mean bash"
    );
}

#[test]
fn other_shells_are_available_and_differ() {
    // Not an exhaustive check of each generator -- only that asking for
    // another shell reaches a different one, rather than silently
    // handing back bash.
    let bash = stdout_of(&["completions", "bash"]);
    for shell in ["zsh", "fish"] {
        let script = stdout_of(&["completions", shell]);
        assert!(!script.is_empty(), "`completions {shell}` produced nothing");
        assert_ne!(
            script, bash,
            "`completions {shell}` produced the bash script"
        );
    }
}

#[test]
fn completions_needs_no_device() {
    // Every other subcommand but `list` and `bundle` fails without
    // `--device`. This one prints a script from the binary's own argument
    // tree, so requiring a serial port to describe it would be absurd --
    // and it is an easy thing to reintroduce, since the check sits in
    // `run` ahead of the dispatch rather than in the type.
    let output = run(&["completions", "bash"]);
    assert!(
        output.status.success(),
        "completions asked for a device: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}
