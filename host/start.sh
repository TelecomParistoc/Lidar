#!/bin/bash

function usage {
  echo "$0 <input file>"
}

gcc -o essai main.c -lrt
rm *.map

if [[ $# != 1 ]]; then
  usage
  exit 1
fi

gnuplot -p plot1.cfg&
./essai $1
