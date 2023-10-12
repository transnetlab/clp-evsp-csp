#!/bin/bash

# Path to the C++ executable
EXECUTABLE="./ebus_vns"

# Array of command-line arguments
ARGS=(
    "Ann_Arbor"
    "Arlington"
    "Cascades_East"
    "CityLink"
    "Cornwall"
    "Denton_County"
    "Gold_Coast"
    "Intercity"
    "LA_Go"
    "Milton"
    "Mountain_Line"
    "SCMTD"
)

# Loop through the arguments and run the program
for arg in "${ARGS[@]}"; do
    echo "Running $EXECUTABLE with argument: $arg"
    $EXECUTABLE "$arg"
    echo "Finished running $EXECUTABLE with argument: $arg"
done

# End of script
