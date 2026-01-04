#!/bin/bash

printf "\n"
echo "--- Script 2 : bck1 ---"
printf "\n"

# argc == 4
if (( $# != 4 )); then
  echo "4 parameters must be given . Exiting... "
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
else
  echo 'Third argument is neither a file, nor a directory. Exiting... '
  echo
  exit 1
fi

# If reached this point, we have 3 args, the user is valid, arg 2 is either a file or a directory , and arg 3 is either a file or a directory.

home=$(getent passwd "$1" | cut -d: -f6) # gives us the user's home.

src=$(realpath "$2" 2>/dev/null)  #canonical absolute paths
home=$(realpath "$home" 2>/dev/null)

if [[ -z "$src" || -z "$home" || ( "$src" != "$home" && "$src" != "$home"/* ) ]]; then
  echo "'$src' does not belong to the user's home. Exiting..."
  exit 1
fi

#####################################################################
# Case 1 : file -> folder
if [[ -f "$src" && -d "$3" ]]; then
  if printf 'tar -cvf %q -- %q\n' "$3/backup.tar" "$src" | at "$4"; then
  echo 'Valid time. Success'
  echo
  echo file "$src" will be copied as backup.tar to "$3" at "$4". Exiting ...
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
if [[ -d "$src" && -d "$3" ]]; then
  if printf 'tar -cvf %q -- %q\n' "$3/backup.tar" "$src" | at "$4"; then
  echo 'Valid time. Success'
  echo
  else
  echo 'Invalid time. exiting...'
  echo
  exit 1
  fi
  echo Folder "$src" will be copied as backup.tar to "$3" at "$4". Exiting ...
  echo
  exit 0
fi
#####################################################################
# Case 3 : file -> file (destination is tar; create if empty)
if [[ -f "$src" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then
    if printf 'tar -cf %q -- %q\n' "$3" "$src" | at "$4"; then
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

  if printf 'tar -rf %q -- %q\n' "$3" "$src" | at "$4"; then
  echo 'Valid time. Success'
  echo
  echo File "$src" will be appended to "$3" at "$4". Exiting ...
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
if [[ -d "$src" && -f "$3" ]]; then #IF directory & file:
  if [[ ! -s "$3" ]]; then # if empty.
    if printf 'tar -cf %q -- %q\n' "$3" "$src" | at "$4"; then # Valid.
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

  if printf 'tar -rf %q -- %q\n' "$3" "$src" | at "$4"; then
  echo 'Valid time, Success.'
  echo
  echo Folder "$src" contents will be appended to "$3" at "$4". Exiting...
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
