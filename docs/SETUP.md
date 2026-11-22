# Setup Instructions

The bot and protocol checker use only the Python standard library.

## Requirements

- Python 3.9 or later
- Docker (for building the submission image)

## Verify the bot

From the repository root, run the recorded protocol stream through the bot:

```sh
python tests/check.py -- python -u src/bot.py
```

## Build the image

Build from the repository root so the root Dockerfile can copy `src/`:

```sh
docker build -t mechfight-bot .
```

The scorer runs the image with restricted network, CPU, memory, and filesystem access. Do not rely
on network access or write files outside `/tmp`.
