# Usage Guide

## Run the conformance check

The checker feeds a recorded game stream to the bot and validates its protocol responses:

```sh
python tests/check.py -- python -u src/bot.py
```

## Build and run with Docker

```sh
docker build -t mechfight-bot .
docker run --rm -i mechfight-bot
```

The image expects the game protocol on standard input and writes protocol responses to standard
output. For the full message contract, see [protocol.md](protocol.md).

The bundled bot is an example baseline: it moves toward the next zone, picks nearby loot, and fires
at the nearest visible mech. Passing the checker confirms protocol compliance, not match performance.
