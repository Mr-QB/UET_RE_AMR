# One map saved by /amr/save_map (each map lives in its own directory).
# Name of the map and of its directory.
string name
# Absolute path to the map YAML; pass it to /amr/set_mode to start Nav2 on this map.
string map_yaml
# Which of the four files written by /amr/save_map exist.
bool has_image
bool has_posegraph
# Total size of the map's files in bytes.
uint64 size_bytes
# Time the map was last saved (modification time of the YAML).
builtin_interfaces/Time modified
