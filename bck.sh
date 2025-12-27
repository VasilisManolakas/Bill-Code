#!/bin/bash

printf "\n"
echo "--- Script 2 : bck ---"
printf "\n"

# argc == 3
if (( $# < 3 )); then
  echo "Not Enough Parameters Given. Exiting... "
  exit 1
elif (( $# > 3 )); then
  echo "Too many Parameters Given. Exiting ... "
  exit 1
fi

# Parameter count ok, if reached this point.

# User check using id
if ! id "$1" >/dev/null 2>&1; then
  echo "User '$1' does not exist. Exiting... "
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
    fi
else
  echo 'Third argument is neither a file, nor a directory. Exiting... '
  exit 1
fi

#####################################################################
# Case 1 : file -> folder
if [[ -f "$2" && -d "$3" ]]; then
  tar -cvf "$3/backup.tar" -- "$2"
  echo file "$2" copied as backup.tar to "$3" successfully. Exiting ...
  exit 0
fi

#####################################################################
# Case 2 : folder -> folder
if [[ -d "$2" && -d "$3" ]]; then
  tar -cvf "$3/backup.tar" -- "$2"
  echo Folder "$2" copied as backup.tar to "$3" successfully. Exiting ...
  exit 0
fi
#####################################################################
# Case 3 : file -> file (destination is tar; create if empty)
if [[ -f "$2" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then
    tar -cf "$3" -- "$2"
    echo "File '$3' was empty; created with '$2'."
    exit 0
  fi

  if ! tar -tf "$3" >/dev/null 2>&1; then
    echo "file '$3' is not a tar file."
    exit 1
  fi

  tar -rvf "$3" -- "$2"
  echo File "$2" successfully appended to "$3". Exiting ,,,
  exit 0
fi

#####################################################################
# Case 4: folder -> file (destination is tar; create if empty)
if [[ -d "$2" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then
    tar -cf "$3" -- "$2"
    echo "File '$3' was empty; created with '$2'."
    exit 0
  fi

  if ! tar -tf "$3" >/dev/null 2>&1; then
    echo "File '$3' is not a tar archive. Exiting..."
    exit 1
  fi

  tar -rvf "$3" -- "$2"
  echo Folder "$2" contents successfully appended to "$3". Exiting...
  exit 0
fi

# Fallback: any unhandled combination
echo "Unsupported argument combination. Exiting..."
exit 1
