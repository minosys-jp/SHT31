#!/bin/bash
cd /tmp
if [ -f sht11.txt ]; then
	mv sht11.txt sht11.txt.1
fi
if [ -f sht31.txt ]; then
	mv sht31.txt sht31.txt.1
fi
