FROM python:3.13-slim

WORKDIR /bot
COPY src/ /bot/src/

ENTRYPOINT ["python3", "-u", "/bot/src/bot.py"]
