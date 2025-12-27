#!/bin/bash

printf "\n"
echo "--- Script 2 : bck1 ---"
printf "\n"

# argc == 3
if (( $# < 4 )); then
  echo "Not Enough Parameters Given. Exiting... "
  echo
  exit 1
elif (( $# > 4 )); then
  echo "Too many Parameters Given. Exiting ... "
  echo
  exit 1
fi

# Parameter count ok, if reached this point.

# User check using id
if ! id "$1" >/dev/null 2>&1; then
  echo "User '$1' does not exist. Exiting... "
  echo
  exit 1
fi

# Arg2 check
if [[ -d "$2" ]]; then
  echo 'Second argument is a directory.'
  echo
elif [[ -f "$2" ]]; then
  echo 'Second argument is a file.'
  echo
else
  echo 'Second argument is neither a file, nor a directory. Script Failed! Exiting...'
  echo
  exit 1
fi

# Arg3 check
if [[ -d "$3" ]]; then
  echo 'Third argument is a directory.'
  echo

elif [[ -f "$3" ]]; then
  echo 'Third argument is a file.'
  echo
  if tar -tf "$3" > /dev/null 2>&1; then
    echo "Third argument is a tar file."
    echo
    fi
else
  echo 'Third argument is neither a file, nor a directory. Exiting... '
  echo
  exit 1
fi

#Check for 'at' time format validity
# Probably won't need that.

#####################################################################
# Case 1 : file -> folder
if [[ -f "$2" && -d "$3" ]]; then
  if printf 'tar -cvf %q -- %q\n' "$3/backup.tar" "$2" | at "$4"; then
  echo 'Valid time. Success'
  echo
  echo file "$2" will be copied as backup.tar to "$3" at "$4". Exiting ...
  echo
  exit 0
  else
  echo 'Invalid time. Exiting...'
  echo
  exit 1
  fi
fi
#####################################################################
# Case 2 : folder -> folder
if [[ -d "$2" && -d "$3" ]]; then
  if printf 'tar -cvf %q -- %q \n' "$3/backup.tar" "$2" | at "$4"; then
  echo 'Valid time. Success'
  echo
  else
  echo 'Invalid time. exiting...'
  echo
  exit 1
  fi
  echo Folder "$2" will be copied as backup.tar to "$3" at "$4". Exiting ...
  echo
  exit 0
fi
#####################################################################
# Case 3 : file -> file (destination is tar; create if empty)
if [[ -f "$2" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then
    if printf 'tar -cf %q -- %q\n' "$3" "$2" | at "$4"; then
    echo "File '$3' was empty; It will be created at '$4'."
    echo
    exit 0
    else
    echo 'Invalid time. Exiting...'
    echo
    exit 1
    fi
  fi

  if ! tar -tf "$3" >/dev/null 2>&1; then
    echo "file '$3' is not a tar file."
    echo
    exit 1
  fi

  if printf 'tar -rvf %q -- %q \n' "$3" "$2" | at "$4"; then
  echo 'Valid time. Success'
  echo
  echo File "$2" will be appended to "$3" at "$4". Exiting ...
  echo
  exit 0
  else
  echo 'Invalid time.'
  echo
  exit 1
  fi
fi

#####################################################################
# Case 4: folder -> tar file (destination is tar; create if empty)
if [[ -d "$2" && -f "$3" ]]; then #IF directory & file:
  if [[ ! -s "$3" ]]; then # if empty.
    if printf 'tar -cf %q -- %q\n' "$3" "$2" | at "$4"; then # Valid.
        echo "File '$3' was empty; it will be created at '$4'"
        echo
        exit 0
    else
        echo 'Invalid time. Exiting...'
        echo
        exit 1
     fi
    fi


if ! tar -tf "$3" >/dev/null 2>&1; then # if file is not tar:
    echo "File '$3' is not a tar archive. Exiting..."
    echo
    exit 1
  fi

  if printf 'tar -rvf %q -- %q\n' "$3" "$2" | at "$4"; then
  echo 'Valid time, Success.'
  echo
  echo Folder "$2" contents will be appended to "$3" at "$4". Exiting...
  echo
  exit 0
  else
  echo 'Invalid time'
  exit 1
  fi
fi

# Fallback: any unhandled combination
echo "Unsupported argument combination. Exiting..."
exit 1
