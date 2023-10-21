#!/bin/bash

# Path to the C++ executable
EXECUTABLE="./ebus_vns"

# Array of command-line arguments
ARGS=(
    "Cornwall"
    "Milton"
    "Mountain_Line"
    "Ann_Arbor"
    "LA_Go"
    "Cascades_East"
    "Denton_County"
    "Gold_Coast"
    "CityLink"
    "Intercity"
    "SCMTD"
    "Arlington"
)

# Loop through the arguments and run the program
for arg in "${ARGS[@]}"; do
    echo "Running $EXECUTABLE with argument: $arg"
    $EXECUTABLE "$arg"
    echo "Finished running $EXECUTABLE with argument: $arg"
done

# End of script
