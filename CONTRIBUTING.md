# Contributing

## Workflow

1. Create a feature branch from `main`.
2. Make focused changes and keep commits small.
3. Run local checks before opening a PR.
4. Open a pull request with a short summary, test notes, and any ROS topic changes.

## Local Checks

```bash
python3 -m py_compile fast_isaac_sim.py
```

For runtime changes, include the exact launch command used and topic checks performed.

## Style

- Keep launcher flags and README commands in sync.
- Prefer clear, explicit defaults in scripts and USD variants.
- Avoid adding heavy sensors/render paths to baseline profiles.

