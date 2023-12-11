#!/bin/bash

shopt -s nullglob

convert(){
  ## recursively traverses the directory and converts rst files to tex files and deletes the rst files
  target=${1:-.}
  for i in "$target"/*; do
    if [ -d "$i" ]; then
      convert "$i"
    else
      if [[ "$i" == *.rst ]]; then
         pandoc "$i" -o "${i%.*}".tex
         rm "$i"
      fi
    fi
  done
}

convert "$@"
