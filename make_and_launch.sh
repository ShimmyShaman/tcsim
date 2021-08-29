#!/bin/bash
set -e

cd ~/proj/unigine/tennis_court/source

cmake .

make

cd ~/proj/unigine/tennis_court

./launch_release.sh