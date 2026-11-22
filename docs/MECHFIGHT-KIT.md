# Mechfight bot kit

This kit contains everything you need to write and check a bot for the 16-mech battle royale. A bot is a program that reads JSON lines on stdin and writes JSON lines on stdout. You submit a GitHub repository whose Dockerfile builds it.

Read [protocol.md](protocol.md) first for the full rules and message format.

## Start here

1. From the repository root, run the example bot through the conformance check and build its Docker image:

```sh
python3 tests/check.py -- python3 -u src/bot.py
docker build -t my-bot .
```

The scorer runs the image with no network, 1 CPU, 512 MB, a read-only disk except `/tmp`, and an unprivileged user (uid 65534, not root). A bot that passes the local check can still fail in that environment. The usual causes are writing files anywhere but `/tmp` and installing packages for root only (`pip install --user`, or into `/root`). Install them system-wide in the Dockerfile.

2. Push this repository and submit its address on the Mech Royale track page. The repository may hold 20 MB at the commit you submit. The scorer builds it on linux/amd64 with the network on, and the build must finish in 10 minutes. Each submission plays 48 practice matches against built-in bots.

### What `check.py` verifies

`check.py` feeds a recorded stream of real observations to your bot and verifies:

- your bot sends `ready` in time
- one schema-valid action per observation with the right `tick`
- answer times against the 50 ms budget
- the newest-observation rule when observations queue up
- debug drawings stay inside the caps
- your bot exits cleanly after `end`

The observations are recorded and do not react to your actions. Passing `check.py` confirms protocol compliance, not how well your bot plays.

## Files in this kit

| Path | What it is |
| --- | --- |
| `docs/protocol.md` | The rules and the protocol. Read this first. |
| `docs/mechfight.schema.json` | JSON Schema for every message (draft-07). Generate types from it in any language. |
| `src/mechfight.py` | Python wrapper: standard library only. Typed messages, the stdin/stdout loop, debug drawing. |
| `src/bot.py` | A small Python bot using the wrapper. |
| `vendor/typescript/mechfight.ts` | TypeScript wrapper: one file, no dependencies, runs with `bun` or `node` (22.18+). |
| `vendor/typescript/example.ts` | A small TypeScript bot using it. |
| `Dockerfile` | Submission image definition. |
| `tests/check.py` | Protocol conformance check (above). |
| `tests/fixtures/stream.jsonl.gz` | Recorded protocol stream used by the checker. |
| `assets/map/v2/` | Arena map (`world.json`, `terrain.i16`; protocol.md §11), mounted at `/arena` in a match. |

The wrappers are optional and deliberately thin. Copy one into your bot and change anything you want. They include no navigation, aiming, or physics. Building your own model of the world from `protocol.md` and from match observations is part of the challenge.

## Debug drawing

Attach `debug` to any action (protocol.md §6) to record lines, points, and labels for your own tools.

- The server only checks `debug` against the caps.
- Debug drawings are kept out of the public replay, never affect the game, and nothing on the track shows them to you.
- Python wrapper calls: `mf.draw.line(a, b, "#ff0")`, `mf.draw.point(p)`, `mf.draw.text("hi", p)`, `mf.draw.label("chasing 7")`.
- TypeScript wrapper calls: `draw.line(...)` and the matching helpers.
