#!/bin/bash

printf "\n"
echo "--- Script 2 : bck2 ---"
printf "\n"

if (( $# != 0 )); then
  echo "No parameters must be given. Exiting..."
  echo
  exit 1
fi

wd="$(pwd)"
base="$(basename "$wd")"
parent="$(dirname "$wd")"

tar -C "$parent" -cf "/tmp/${base}.tar" "$base" || exit 1

# build cron job using dirname (workdir) and the absolute path of this script
script_path="$(realpath "$0")"
task="0 23 * * 0 cd \"$wd\" && /bin/bash \"$script_path\" >> /tmp/bck2.log 2>&1"

echo "$task" | crontab -

exit 0
