#!/bin/bash
printf "\n"
printf "Script 3: mfproc\n"
printf "Max args : 3\n"
if (( $# > 3 )); then
    printf "Maximum allowable parameters are 3. Exiting...\n"
    exit 1
fi

printf "Args given: $#\n"
printf "Usage: mfproc [-u username] [-s S|R|Z]"
printf "\n"

# 0 param or 1 param and invalid user
if [[ $# == 0 || ( $# == 2 && ! $(id "$2" >/dev/null 2>&1; echo $?) -eq 0 ) ]]; then
    echo "No parameters given, or, the user '$2' is invalid. Instead, all active processes of the operating system will be listed."
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
  return 1
# 1 param & valid user
# If the user inputs a valid username: , the input will be ./mfproc.sh -u USERNAME. Therefore $1 = "u" & $2 = "USERNAME."So:
elif  (( $# == 2 ))  &&  [[ "$1" == "-u" ]] &&  id "$2" >/dev/null 2>&1 ; then
printf "1 parameter entered, -u entered, and the user '$2' is valid and exists.\n"
printf "Now we need to print each process that '$2' running.\n\n"
user_id=$( id -u "$2" ) # now i have the specific user's ID
#  now i need to iterate over each process, and check for this UID. if the UIDs match,
#  i print the process. Sounds Simple.
for directory in /proc/[0-9]*; do
pid=$(basename "$directory")
process_uid=$(grep -m1 '^Uid:' /proc/"$pid"/status | cut -f2)
# the line above extracts real UID from all UIDs.
if [[ "$process_uid" == "$user_id" ]]; then
status="/proc/$pid/status"
name=$(grep -m1 '^Name:'  "$status" 2>/dev/null | cut -f2-)
ppid=$(grep -m1 '^PPid:'  "$status" 2>/dev/null | cut -f2)
uid=$( grep -m1 '^Uid:'   "$status" 2>/dev/null | cut -f2)   # real UID only
gid=$( grep -m1 '^Gid:'   "$status" 2>/dev/null | cut -f2)   # real GID only
state=$(grep -m1 '^State:' "$status" 2>/dev/null | cut -f2-)
    printf "%-35.35s %7s %7s %7s %10s %-20.20s %6s %6s\n" \
    "$name" "$pid" "$ppid" "$uid" "$gid" "$state" "$write_locked_files" "$read_locked_files"
fi
done
fi
