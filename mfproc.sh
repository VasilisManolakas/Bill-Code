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
printf "\n"
cd /proc
find /proc -maxdepth 1 -type d -name '[0-9]*' > /dev/null 2>&1  #this finds all processes running on the OS.
for directory in /proc/[0-9]*; do
 pid=$(basename "$directory")
# cat /proc/"$pid"/status > /dev/null 2>&1 #now i have access to the pid's status txt file.
 name=$(grep -m1 '^Name:' /proc/"$pid"/status) name=${name#Name:}
 ppid=$(grep -m1 '^PPid:' /proc/"$pid"/status) ppid=${ppid#PPid:}
 uid=$(grep -m1 '^Uid:'  /proc/"$pid"/status)  uid=${uid#Uid:}
 gid=$(grep -m1 '^Gid:'  /proc/"$pid"/status)  gid=${gid#Gid:}
 state=$(grep -m1 '^State:' /proc/"$pid"/status) state=${state#State:}
 write_locked_files=$(grep -c 'WRITE' /proc/locks)
 read_locked_files=$(grep -c 'READ' /proc/locks)
 printf "%-35.35s %7s %7s %7s %10s %-20.20s %6s %6s\n" \
  "$name" "$pid" "$ppid" "$uid" "$gid" "$state" "$write_locked_files" "$read_locked_files"

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
