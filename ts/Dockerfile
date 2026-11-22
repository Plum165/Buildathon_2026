# Build from this folder: docker build -t my-bot .
# The match runs it with no network, a read-only root (writable /tmp), 1 CPU, 512 MB, and the map at /arena.
FROM python:3.13-slim
WORKDIR /bot
COPY . .
ENV PYTHONPATH=/bot:/bot/python
ENTRYPOINT ["python3", "-u", "/bot/python/bot.py"]
