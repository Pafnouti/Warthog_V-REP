#!/bin/sh

catkin_make
source devel/setup.bash
rosrun keyboard_transcript kb_t.py 