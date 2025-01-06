#!/bin/bash
# ---------------------------------------------------------------------------------
# To be used inside docker container.
# Script that forwards x11 permissions from .Xauthority file in /root/.
# ---------------------------------------------------------------------------------

# used to cd to current again
DIRECTORY=$PWD
echo "Merging .Xauthority file in /root/.Xauthority ..."
cd /root/
xauth merge /root/.Xauthority
# xauth merge sometimes throws an error like xauth:  file /home/.Xauthority does not exist, even though it worked. 
# Check with xeyes if it actually worked.
echo "Done. If successful, xeyes shoud show on screen right now. Press Ctrl-C to interrupt"
xeyes
cd $DIRECTORY
