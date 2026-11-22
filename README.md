# Mechfight Hackathon Submission

This project contains a bot for the 16-mech battle royale. The bot reads JSON lines on stdin and writes JSON lines on stdout. Its Docker image is built for submission to the hackathon.

## Repository Structure

```text
├── arena/
│   └── v2/
│       ├── terrain.i16
│       └── world.json
├── check/
│   ├── check.py
│   └── stream.jsonl.gz
├── python/
│   ├── bot.py
│   └── mechfight.py
├── ts/
│   ├── Dockerfile
│   ├── example.ts
│   ├── mechfight.ts
│   └── package.json
├── .gitignore
├── Dockerfile
├── mechfight.schema.json
├── protocol.md
└── README.md
```

### Folder and File Guide

- **arena/v2/** — Game map files used by the match runner.
- **check/** — Protocol conformance checker and recorded observation stream.
- **python/** — Python bot and its protocol wrapper.
- **ts/** — TypeScript wrapper, example bot, package metadata, and Dockerfile.
- **.gitignore** — Git ignore rules.
- **Dockerfile** — Builds the Python bot image from the repository root.
- **mechfight.schema.json** — JSON Schema for protocol messages.
- **protocol.md** — Game rules and bot communication protocol.
- **README.md** — Project overview, setup, and submission instructions.

## About the Bot

Read [protocol.md](protocol.md) for the full rules and message format.

## Getting Started

From the repository root, run the bot through the conformance check, build the image, and check it the way a match starts it:

```sh
python3 check/check.py -- python3 -u python/bot.py
docker build -t my-bot .
python3 check/check.py --map-dir /arena -- docker run --rm -i --network none --cpus 1 --memory 512m --read-only --tmpfs /tmp:rw,size=64m --user 65534:65534 -v "$PWD/arena:/arena:ro" my-bot
```

The last command is what the scorer runs before your matches: no network, 1 CPU, 512 MB, a read-only disk except `/tmp`, and an unprivileged user (uid 65534, not root). A bot that passes without those flags can still fail with them. The usual causes: writing a file anywhere but `/tmp`, and installing packages for root only (`pip install --user`, or into `/root`). Install them system-wide in the Dockerfile.

To submit, make a public GitHub repository with the bot files at its top level, so the `Dockerfile` is at the repository root. Push it and submit its address on the Mech Royale track page. The repository may hold 20 MB at the commit you submit. The scorer builds it on linux/amd64 with the network on, and the build must finish in 10 minutes. Each submission plays 48 practice matches against built-in bots.

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
| `protocol.md` | The rules and the protocol. Read this first. |
| `mechfight.schema.json` | JSON Schema for every message (draft-07). Generate types from it in any language. |
| `python/mechfight.py` | Python wrapper: one file, standard library only. Typed messages, the stdin/stdout loop, debug drawing. |
| `python/bot.py` | The Python battle royale bot. |
| `ts/mechfight.ts` | TypeScript wrapper: one file, no dependencies, runs with `bun` or `node` (22.18+). |
| `ts/example.ts` | A small TypeScript bot using it. |
| `Dockerfile`, `ts/Dockerfile` | Docker images for the Python and TypeScript bots. |
| `check/check.py` | Protocol conformance check (above). |
| `arena/` | The map (`arena/v2/world.json`, `arena/v2/terrain.i16`; protocol.md §11). Mounted at `/arena` in a match. |

The wrappers are optional and deliberately thin. They include no navigation, aiming, or physics. Building your own model of the world from `protocol.md` and match observations is part of the challenge.

## Debug drawing

Attach `debug` to any action (protocol.md §6) to record lines, points, and labels for your own tools.

- The server only checks `debug` against the caps.
- Debug drawings are kept out of the public replay, never affect the game, and nothing on the track shows them to you.
- Python wrapper calls: `mf.draw.line(a, b, "#ff0")`, `mf.draw.point(p)`, `mf.draw.text("hi", p)`, `mf.draw.label("chasing 7")`.
- TypeScript wrapper calls: `draw.line(...)` and the matching helpers.

## Links

- [LinkedIn](https://www.linkedin.com/in/moegamatsamsodien/)
- [Portfolio](https://moegamat-samsodien-portfolio.vercel.app/)
