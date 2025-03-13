#!/usr/bin/env bash

find . -name '*.py' \
    | xargs black --safe --line-length=79 --target-version=py312

pydoc-markdown -I ./ah_wrapper/ -m ah_serial_client > ./docs/AHSerialClient.md
pydoc-markdown -I ./ah_wrapper/ -m hand > ./docs/Hand.md
