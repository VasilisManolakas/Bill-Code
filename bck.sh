#!/bin/bash

printf "\n"
echo "--- Script 2 : bck ---"
printf "\n"

# Arguments must be 3. (Username, file/folder, file/folder)
if (( $# != 3 )); then
  echo "3 parameters must be given. Exiting... "
  exit 1
fi
  #Param 1 checks
# User check using id
if ! id "$1" >/dev/null 2>&1; then #descriptor 1 -> stdout , descriptor 2-> stderr
  echo "User '$1' does not exist. Exiting... "
  exit 1
fi

# Param 2 check
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

# Param 3 check
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

# If reached this point, we have 3 args, the user is valid, arg 2 is either a file or a directory , and arg 3 is either a file or a directory.

home=$(getent passwd "$1" | cut -d: -f6) # gives us the user's home.

src=$(realpath "$2" 2>/dev/null)  # realpath ensures paths are canonical absolute paths.
home=$(realpath "$home" 2>/dev/null)

if [[ -z "$src" || -z "$home" || ( "$src" != "$home" && "$src" != "$home"/* ) ]]; then  # if source path or home path are empty strings, realpath failed. However, if src is not in /home, then src does not belong to /home.
  echo "'$2' does not belong to the user. Exiting..."
  exit 1
fi

#####################################################################
# Case 1 : file -> directory
if [[ -f "$src" && -d "$3" ]]; then
  echo "'$src' is a file and '$3' is a directory."
  tar -cf "$3/backup.tar" -- "$src"
  echo
  echo file "$src" created as backup.tar and copied to "$3" successfully. Exiting ...
  exit 0
fi
#####################################################################
# Case 2 : folder -> folder
if [[ -d "$src" && -d "$3" ]]; then
  tar -cf "$3/backup.tar" -- "$src"
  echo
  echo Folder "$src" copied as backup.tar to "$3" successfully. Exiting ...
  exit 0
fi
#####################################################################
# Case 3 : file -> file (destination is tar)
if [[ -f "$src" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then  # if file is empty, create it
    tar -cf "$3" -- "$src"
    echo
    echo "File '$3' was empty; created with '$src'."
    exit 0
  fi
  if ! tar -tf "$3" >/dev/null 2>&1; then
    echo "file '$3' is not a tar file. Exiting..."
    exit 1
  fi
  tar -rf "$3" -- "$src"
  echo File "$src" successfully appended to "$3". Exiting...
  exit 0
fi
#####################################################################
# Case 4: folder -> file (destination is tar; create if empty)
if [[ -d "$src" && -f "$3" ]]; then
  if [[ ! -s "$3" ]]; then
    tar -cf "$3" -- "$src"
    echo "File '$3' was empty; created with '$src'."
    echo
    exit 0
  fi
  if ! tar -tf "$3" >/dev/null 2>&1; then
    echo "File '$3' is not a tar archive. Exiting..."
    exit 1
  fi
  tar -rf "$3" -- "$src"
  echo
  echo Folder "$src" contents successfully appended to "$3". Exiting...
  exit 0
fi
# Fallback: any unhandled combination
echo "Unsupported argument combination. Exiting..."
exit 1
