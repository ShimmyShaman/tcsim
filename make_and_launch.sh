#!/bin/bash
set -e

cd ~/proj/tennis_court/source

cmake .

make

cd ~/proj/tennis_court

./launch_release.sh