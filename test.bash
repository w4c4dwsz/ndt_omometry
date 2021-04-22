#!/bin/bash

while true;do
	case $1 in
		-b)
			echo '-b'
			shift 1
			;;
		-c)
			echo '-c'
			shift 1
			;;
		--)
			echo 'finish'
			if [ ! -z $2];
			then
				echo "invalid parameter:$2"
				exit 1
			fi

			break
			;;
		*)
			echo 'test'
			break
			;;
	esac
done

