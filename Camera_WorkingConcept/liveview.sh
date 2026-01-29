#!/bin/bash

while [ 1 ] ; do
  dd if=/dev/xillybus_read_32 bs=614400 iflag=fullblock count=1 2>/dev/null
done | mplayer -demuxer rawvideo -rawvideo w=640:h=480:format=uyvy:size=614400 -
