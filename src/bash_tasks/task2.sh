#!/bin/bash

ARG1=$1

if [ "$ARG1" == 'small_square' ]; then  
   rosrun linux_exam small_square.py

elif [ "$ARG1" == 'medium_square' ]; then     
     rosrun linux_exam medium_square.py

elif [ "$ARG1" == 'big_square' ]; then     
     rosrun linux_exam big_square.py

else 
echo "Please enter one of the following;
small_square
medium_square
big_square"

fi