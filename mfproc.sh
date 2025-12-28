#!/bin/bash
printf "\n"
printf "Script 3: mfproc\n"
printf "Max args : 2\n"
(( $# > 2 )) && exit 1
printf "Args given: $#\n"
printf "Usage: mfproc [-u username] [-s S|R|Z]"
printf "\n"
# 0 param
if (( $# == 0 )); then

#1 param
elif (( $# == 1)); then

#2 params
else

fi

