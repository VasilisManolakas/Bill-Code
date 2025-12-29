#!/bin/bash

printf "\n"
echo "--- Script 1: Searching ---"
printf "\n"

if (( $# != 2 )); then
echo "Two arguments must be given, or else the script will not run."
printf "\n"
echo "For integer arguments x and y, make sure that the script is executed as : ./searching.sh x y"
exit 1
fi

if ! [[ "$1" =~ ^[0-7]{3,4}$ ]]; then
echo "$1" is not an octal permission value. Exiting...
exit 1
fi

if ! [[ "$2" =~ ^[0-9]+$ ]]; then
echo "$2" is not a valid number for days. Exiting...
exit 1
fi

echo Integer Parameter 1 is : $1
echo Integer Parameter 2 is : $2
echo

c1=0;c2=0;c3=0;c4=0;c5=0;

while :
do
read -r -p 'Directory (Press q to quit): '  directory
if [[ "$directory" == "q" ]]; then
printf "\n"
echo Exiting...
break
fi
if [[ ! -d "$directory" ]]; then
echo
echo "$directory" is not a directory. Try again :
echo
continue
fi
echo Searching $directory...
echo
#1
command_output_1=$(find "$directory" -type f -perm "$1" -print)
command_count_1=$(printf '%s\n' "$command_output_1" | grep -c .)
echo "$command_count_1 files will be printed for question 1."
echo
printf '%s\n' "$command_output_1"
(( c1 += command_count_1 ))

#2
command_output_2=$(find "$directory" -type f -mtime -"${2}" -print)
command_counter_2=$(printf '%s\n' "$command_output_2" | grep -c .)
echo "$command_counter_2 files will be printed for question 2."
echo
printf '%s\n' "$command_output_2"
(( c2 += command_counter_2 ))
#3
command_output_3=$(find "$directory" -type d -atime -"${2}" -print)
command_counter_3=$(printf '%s\n' "$command_output_3" | grep -c .)
echo "$command_counter_3 directories will be printed for question 3."
echo
printf '%s\n' "$command_output_3"
(( c3 += command_counter_3 ))

#4
command_output_4=$(ls -l "$directory" | grep -E '^-r..r..r..')
command_counter_4=$(printf '%s\n' "$command_output_4" | grep -c .)
echo "$command_counter_4 files will be printed for question 4."
echo
printf '%s\n' "$command_output_4"
(( c4 += command_counter_4 ))
#5
command_output_5=$(ls -l "$directory" | grep -E '^(d....w[xs]...|d.......w[xt])')
command_counter_5=$(printf '%s\n' "$command_output_5" | grep -c .)
echo "$command_counter_5 directories will be printed for question 5."
echo
printf '%s\n' "$command_output_5"
(( c5 += command_counter_5 ))

done

echo Case 1: $c1 tree files had octal right "$1".
echo Case 2: $c2 tree files have been modified the last "$2" days.
echo Case 3: $c3 subdirectories have been accessed in the last "$2" days.
echo Case 4: $c4 files grant read rights to all users.
echo "Case 5: $c5 subdirectories allow write/execute to group or others (not only the owner)".
