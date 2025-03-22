#!/bin/bash

# Base directory where the plugin directories are located
BASE_DIR="/home/beedo/SOFA/v23.06.00/plugins"

# Initialize PYTHONPATH variable
PYTHONPATH=""

export SOFA_ROOT="/home/beedo/SOFA/v23.06.00"

# Loop through each plugin directory in BASE_DIR
for plugin_dir in "$BASE_DIR"/*; do
    # Ensure it's a directory
    if [ -d "$plugin_dir" ]; then
        # Add the corresponding Python package path to PYTHONPATH
        PYTHONPATH="${plugin_dir}/lib/python3/site-packages:$PYTHONPATH"
    fi
done

# Export the PYTHONPATH
export PYTHONPATH="${PYTHONPATH}" # Remove the trailing colon

echo "Done Sourcing Sofa PYTHONPATH"
