SCRIPT_DIR=$( cd "$( dirname "$0" )" && pwd )
MAIN="$SCRIPT_DIR/bin/tennis_court_x64"
export LD_LIBRARY_PATH="$SCRIPT_DIR/bin:$LD_LIBRARY_PATH"
if [ -f "$MAIN" ]; then
	 "$MAIN"  -video_app auto -video_vsync 0 -video_refresh 0 -video_mode 1 -video_resizable 1 -video_fullscreen 0 -video_debug 0 -sound_app auto -data_path "../data/" -extern_plugin "FbxImporter,GLTFImporter,FbxExporter" -console_command "config_autosave 0 && world_load \"tennis_court\""
else
	echo "Application executable not found"
fi
