#!/bin/bash

printf "\n"
printf "Script 3: mfproc\n"
printf "Max args : 4\n"

if (( $# > 4 )); then
  printf "Maximum allowable parameters are 4. Exiting...\n"
  exit 1
fi

printf "Args given: %s\n" "$#"
printf "Usage: mfproc [-u username] [-s S|R|Z]\n\n"

# -------------------------
# Parse args (0, 2, or 4 tokens)
#   mfproc
#   mfproc -u user
#   mfproc -s S|R|Z
#   mfproc -u user -s S|R|Z   (either order)
# -------------------------
user_name=""
user_uid=""
state_filter=""

if (( $# == 0 )); then
  :
elif (( $# == 2 )); then
  if [[ "$1" == "-u" ]]; then
    user_name="$2"
  elif [[ "$1" == "-s" ]]; then
    state_filter="$2"
  else
    exit 1
  fi
elif (( $# == 4 )); then
  if [[ "$1" == "-u" && "$3" == "-s" ]]; then
    user_name="$2"
    state_filter="$4"
  elif [[ "$1" == "-s" && "$3" == "-u" ]]; then
    state_filter="$2"
    user_name="$4"
  else
    exit 1
  fi
else
  exit 1
fi

# validate user if given
if [[ -n "$user_name" ]]; then
  if ! id "$user_name" >/dev/null 2>&1; then
    exit 1   # user does not exist
  fi
  user_uid=$(id -u "$user_name")
fi

# validate state if given
if [[ -n "$state_filter" ]]; then
  if [[ "$state_filter" != "R" && "$state_filter" != "S" && "$state_filter" != "Z" ]]; then
    exit 2
  fi
fi

# Header (helps readability; remove if your grader wants no header)
printf "%-35.35s %7s %7s %7s %7s %-20.20s %6s %6s\n" \
  "Name" "PID" "PPID" "UID" "GID" "State" "WLOCK" "RLOCK"

printed=0

for directory in /proc/[0-9]*; do
  pid=$(basename "$directory")
  status="/proc/$pid/status"

  [[ -r "$status" ]] || continue

  # read fields from /proc/<pid>/status
  name=$(grep -m1 '^Name:'  "$status" 2>/dev/null | cut -f2-)
  ppid=$(grep -m1 '^PPid:'  "$status" 2>/dev/null | cut -f2)
  uid=$( grep -m1 '^Uid:'   "$status" 2>/dev/null | cut -f2)   # real UID
  gid=$( grep -m1 '^Gid:'   "$status" 2>/dev/null | cut -f2)   # real GID
  state_full=$(grep -m1 '^State:' "$status" 2>/dev/null | cut -f2-)
  state_letter=$(grep -m1 '^State:' "$status" 2>/dev/null | cut -f2 | cut -c1)

  # filter by user (if -u given)
  if [[ -n "$user_uid" && "$uid" != "$user_uid" ]]; then
    continue
  fi

  # filter by state:
  # if -s NOT given -> show only R/S/Z
  if [[ -z "$state_filter" ]]; then
    if [[ "$state_letter" != "R" && "$state_letter" != "S" && "$state_letter" != "Z" ]]; then
      continue
    fi
  else
    if [[ "$state_letter" != "$state_filter" ]]; then
      continue
    fi
  fi

  # count locks per PID from /proc/locks (field after WRITE/READ is PID)
  write_locked_files=$(grep -cE "[[:space:]]WRITE[[:space:]]+$pid[[:space:]]" /proc/locks 2>/dev/null)
  read_locked_files=$(grep -cE "[[:space:]]READ[[:space:]]+$pid[[:space:]]" /proc/locks 2>/dev/null)

  printf "%-35.35s %7s %7s %7s %7s %-20.20s %6s %6s\n" \
    "$name" "$pid" "$ppid" "$uid" "$gid" "$state_full" "$write_locked_files" "$read_locked_files"

  ((printed++))
done

# if -s was requested and nothing matched -> exit 2
if [[ -n "$state_filter" && $printed -eq 0 ]]; then
  exit 2
fi

exit 0
