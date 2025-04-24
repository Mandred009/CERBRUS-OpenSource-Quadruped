#!/bin/bash

# Bash script to log the startup times and start the main script
echo "Log In: " `date`>>startup_log.txt

echo "Cerbrus Startup Script Active"

# comment this line to stop automatic cerbrus activation
python3 start.py



