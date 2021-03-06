#!/bin/bash

# Takes 4 postional arguments
# The first one is the basm file with all the string symbols
# The second one is the bst fle with only the system string symbols
# The third one is the associate bytecode header file
# The fourth one is the ouput header file 
 
# Delete existing file
echo "/* DO NOT EDIT */" > $4
echo "/* THIS IS A GENERATED FILE */" >> $4
echo "" >> $4

# Writing content
echo "#ifndef __EXTRACTED_BBZSTRINGS_H__" >> $4
echo "#define __EXTRACTED_BBZSTRINGS_H__" >> $4
echo "" >> $4

echo "#include \"$3\"" >> $4
echo "" >> $4

echo "#include <array>" >> $4
echo "#include <cstdint>" >> $4
echo "#include <utility>" >> $4
echo "" >> $4

IFS=$'\n' # make newlines the only separator

offset_size=$(($(grep "^\w" $2 | wc -l) - 1))
echo "#define BBZSTRING_OFFSET ($offset_size)" >> $4
echo "" >> $4

array_size=$(head -n 1 $1 | cut -c 2-)
echo "#define BBZSTRING_ARRAY_SIZE ($array_size - BBZSTRING_OFFSET)" >> $4
echo "" >> $4

i=0
echo "const std::array<const std::pair<const uint16_t, const char*>, BBZSTRING_ARRAY_SIZE> g_bbzStringResolverArray = {{"  >> $4
grep "^'" $1 | cut -c 2- | while read -r line
do
    if [[ "$i" -ge $offset_size ]]; then
        formatted_line=$(echo $line | sed -e "s/[^a-zA-Z0-9_]/_/g")
        echo "    {BBZSTRID_$formatted_line, \"$line\"}," >> $4
    fi
    ((i++))
done
echo "}};" >> $4

echo "" >> $4
echo "#endif // __EXTRACTED_BBZSTRINGS_H__" >> $4
