#! /bin/bash

colcon build --packages-select door_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
find build -name compile_commands.json -exec jq -s 'add' {} + > compile_commands.json
ln -sf $(pwd)/compile_commands.json build/compile_commands.json

