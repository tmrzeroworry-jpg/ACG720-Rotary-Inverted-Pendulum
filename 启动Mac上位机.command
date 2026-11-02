#!/bin/zsh
cd "$(dirname "$0")/Mac上位机" || exit 1
exec /bin/zsh ./启动上位机.command
