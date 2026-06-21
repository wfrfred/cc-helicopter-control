#!/bin/sh
set -eu

find . -name '*.lua' \
    -not -path './.git/*' \
    -print \
    -exec luac -p {} \;
