#!/bin/zsh
cd "$(dirname "$0")" || exit 1
if ! /usr/bin/python3 launch.py; then
    echo '启动失败。请保留上方错误信息，按回车关闭。'
    read -r
fi
