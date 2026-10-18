#!/bin/sh
# Host tests for the encoder: ./test/run.sh (any C++17 compiler).
set -e
cd "$(dirname "$0")"
OUT=${TMPDIR:-/tmp}/sm16380_encoder_test
${CXX:-c++} -std=c++17 -O2 -Wall -Wno-unused-variable -o "$OUT" encoder_test.cpp
"$OUT"
