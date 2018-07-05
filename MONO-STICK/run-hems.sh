#!/bin/bash
command='python hemsproxy.py'
exit_mode='n'
child_process=0

sigterm () {
	exit_mode='y'
	echo "SIGTERM"
	if [ $child_process != 0 ]; then
		kill -KILL $child_process
	fi
}

#set -m
trap "sigterm" SIGTERM SIGINT
while [ $exit_mode = 'n' ]; do
	if [ $child_process = 0 ]; then
		$command &
		child_process=$!
		echo "child_process:" $child_process
	fi
	sleep 5
	kill -0 $child_process >/dev/null 2>&1
	if [ $? != 0 ]; then
		child_process=0
	fi
done
if [ $child_process != 0 ]; then
	kill -KILL $child_process
fi
