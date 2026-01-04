#!/bin/bash
printf "\n"
printf "Script 3: mfproc\n"
printf "Max args : 4\n"
if (( $# > 4 )); then
    printf "Maximum allowable parameters are 4. Exiting...\n"
    exit
fi
# 4 Cases:
#0 params -> print every process
#1 param -> could be either username or process state .
#2 params -> both username and states given.
printf "Number of args given: $#\n"
printf "Use as: mfproc [-u username] [-s S|R|Z]"
printf "\n"

# 0 param or 1 param and invalid user
if [[ $# == 0 || ( $# == 2 && ! $(id "$2" >/dev/null 2>&1; ) ) ]]; then
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
if (( $# == 2 ))  && !  id "$2" >/dev/null 2>&1; then
  exit 1  # Non existent user
elif (( $# == 0 )); then
  exit 0
fi
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
elif (( $# == 4 )) && [[ "$1" == "-u" ]] && [[ "$3" == "-s" ]] && id "$2" >/dev/null 2>&1; then
  # Correct input: -u USER -s S|R|Z
  uuid=$(id -u "$2")  # user's UID

  for directory in /proc/[0-9]*; do
    pid=$(basename "$directory")
    status="/proc/$pid/status"

    process_uid=$(grep -m1 '^Uid:' "$status" 2>/dev/null | cut -f2) || continue
    [[ "$process_uid" == "$uuid" ]] || continue

    state=$(grep -m1 '^State:' "$status" 2>/dev/null | cut -f2 | cut -c1) || continue
    [[ "$state" == "$4" ]] || continue   # $4 is S/R/Z

    name=$(grep -m1 '^Name:' "$status" 2>/dev/null | cut -f2-)
    ppid=$(grep -m1 '^PPid:' "$status" 2>/dev/null | cut -f2)
    uid=$process_uid
    gid=$(grep -m1 '^Gid:' "$status" 2>/dev/null | cut -f2)
#
    printf "%-35.35s %7s %7s %7s %10s %-20.20s\n" \
      "$name" "$pid" "$ppid" "$uid" "$gid" "$state"
  done

  exit 0
fi


