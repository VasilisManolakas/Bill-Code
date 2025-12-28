#!/bin/bash
printf "\n"
printf "Script 3: mfproc\n"
printf "Max args : 2\n"
if (( $# > 2 )); then
    printf "Maximum allowable parameters are 2. You entered $#. Exiting...\n"
    exit 1
fi
printf "Args given: $#\n"
printf "Usage: mfproc [-u username] [-s S|R|Z]"
printf "\n"
# 0 param
if (( $# == 0 )); then
echo "No parameters given. Instead, all active processes of the operating system will be listed."
cd /proc
find /proc -maxdepth 1 -type d -name '[0-9]*' > /dev/null 2>&1  #this finds all processes running on the OS.
for directory in /proc/[0-9]*; do
 pid=$(basename "$directory")
# cat /proc/"$pid"/status > /dev/null 2>&1 #now i have access to the pid's status txt file.
 name=$(grep -m1 '^Name:' /proc/"$pid"/status)
 ppid=$(grep -m1 '^PPid:' /proc/"$pid"/status)
 uid=$(grep -m1 '^Uid:'  /proc/"$pid"/status)
 gid=$(grep -m1 '^Gid:'  /proc/"$pid"/status)
 state=$(grep -m1 '^State:' /proc/"$pid"/status)
 write_locked_files=$(grep -c 'WRITE' /proc/locks)
 read_locked_files=$(grep -c 'READ' /proc/locks)
done

exit 0
#1 param
elif (( $# == 1 )); then
#  i will check for user validity here.
if ! id "$1" >/dev/null 2>&1; then
echo "User '$1' does not exist. Instead, all active processes of the operating system will be listed."
  exit 1


#2 params
else



exit 0
fi
fi
