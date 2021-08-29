cd "$( dirname "$0" )"

SCRIPT_DIR=$( pwd )
export LD_LIBRARY_PATH="$SCRIPT_DIR/bin/launcher:$LD_LIBRARY_PATH"

./bin/launcher/browser_x64.linux -config data/launcher/launcher.xml