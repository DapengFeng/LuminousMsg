#!/bin/bash

directory=$1
directory=${directory%/}  # Remove trailing slash if present

if [ -z "$directory" ]; then
    echo "Usage: $0 <directory>"
    exit 1
fi

stem=$(basename "$directory")
mkdir -p "$directory/../Luminous/$stem"

find "$directory" -mindepth 1 -type d | while read -r folder; do
    echo "Processing $folder"
    relative_dir=$(realpath --relative-to="$directory" "$folder")
    echo "Relative directory: $relative_dir"
    python script/convert.py "$directory/$relative_dir" "$directory/../Luminous/$stem/$relative_dir"  --overwrite
    echo "Converted $folder to Luminous format at $directory/../Luminous/$stem/$relative_dir"
    echo "------------------------------"
done

# rm -r "$directory"
