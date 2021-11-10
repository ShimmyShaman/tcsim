#!/bin/bash
set -e

cd ~/proj/tcsim/source

cmake .

make

cd ~/proj/tcsim

./launch_release.sh