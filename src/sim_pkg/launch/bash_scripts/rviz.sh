#!/bin/bash

RVIZ_CONFIG=$(rospack find sim_pkg)/launch/bash_scripts/KOUMEKATRONOM.rviz

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "RViz configuration file not found: $RVIZ_CONFIG"
    exit 1
fi

rviz -d "$RVIZ_CONFIG" 2> >(grep -v TF_REPEATED_DATA buffer_core)
