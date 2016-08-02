#!/bin/sh

# Draw whatever is in gcode.txt

cd /home/pi/drawbot/
/usr/bin/logger -t DRAWBOT Started pibot.py with `wc -l gcode.txt` commands.
/usr/bin/python /home/pi/drawbot/pibot.py > /tmp/log 2>&1
/usr/bin/logger -t DRAWBOT Drawing complete at `date +'%F %X'`
rm gcode.txt





