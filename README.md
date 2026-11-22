# Mechfight Bot

A Python bot for the Mechfight battle royale. The bot reads protocol messages from standard input
and writes responses to standard output. See [the protocol guide](docs/protocol.md) for the complete
message format and game rules.

## Quick start

Run the bundled protocol conformance check:

```sh
python tests/check.py -- python -u src/bot.py
```

Build the submission image from the repository root:

```sh
docker build -t mechfight-bot .
```

## Repository layout

```text
├── assets/
│   ├── README.md
│   └── map/
│       ├── README.md
│       └── v2/
│           ├── terrain.i16
│           └── world.json
├── docs/
│   ├── ACKNOWLEDGEMENTS.md
│   ├── EXPERIMENTS.md
│   ├── MECHFIGHT-KIT.md
│   ├── SETUP.md
│   ├── STRATEGY.md
│   ├── TEAM.md
│   ├── USAGE.md
│   ├── mechfight.schema.json
│   ├── protocol.md
│   └── demo/
├── src/
│   ├── bot.py
│   ├── mechfight.py
│   ├── strategy/
│   │   ├── survival.py
│   │   ├── navigation.py
│   │   ├── combat.py
│   │   ├── targeting.py
│   │   ├── loot.py
│   │   └── upgrades.py
│   └── utils/
│       ├── geometry.py
│       └── map_utils.py
├── tests/
│   ├── README.md
│   ├── check.py
│   ├── fixtures/
│   │   └── stream.jsonl.gz
│   └── test_strategy.py
├── scripts/
│   ├── README.md
│   └── benchmark.py
├── vendor/
│   ├── README.md
│   ├── python/
│   │   └── Dockerfile
│   └── typescript/
│       ├── example.ts
│       ├── mechfight.ts
│       └── package.json
├── .dockerignore
├── .editorconfig
├── .gitattributes
├── .gitignore
├── Dockerfile
├── README.md
└── LICENSE
```

`src/mechfight.py` is the protocol wrapper used by the example bot. The strategy and utility
modules are starting points for the bot's decision-making code. The optional TypeScript reference
implementation is kept under `vendor/typescript/`. Existing GitHub issue templates and VS Code
settings remain in `.github/` and `.vscode/`.
