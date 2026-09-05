#!/usr/bin/env bash
set -e
python simulate.py --duration 40 --gait crawl --rpm 12 --camera beauty --inspect --regenerate-model
