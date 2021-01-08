#!/bin/sh

# Delete existing file
echo "/* DO NOT EDIT */" > $4
echo "/* THIS IS A GENERATED FILE */" >> $4
echo "" >> $4

# Writing content
echo "#ifndef __EXTRACTED_BBZSTRINGS_H__" >> $4
echo "#define __EXTRACTED_BBZSTRINGS_H__" >> $4
echo "" >> $4

echo "#include \"$2\"" >> $4
echo "" >> $4

echo "#include <array>" >> $4
echo "#include <cstdint>" >> $4
echo "#include <utility>" >> $4
echo "" >> $4

offset_size=$(grep "^\w" $3 | wc -l)
echo "#define BBZSTRING_OFFSET ($offset_size)" >> $4
echo "" >> $4

array_size=$(grep "^'\w" $1 | wc -l)
echo "#define BBZSTRING_ARRAY_SIZE ($array_size)" >> $4
echo "" >> $4

i=0
echo "const std::array<const std::pair<const int, const char*>, BBZSTRING_ARRAY_SIZE> test = {{"  >> $4
grep "^'" $1 | cut -c 2- | while read -r line
do
    if [[ "$i" -gt $offset_size ]]; then
        formatted_line=$(echo $line | sed -e "s/[^a-zA-Z0-9_]/_/g")
        echo "    {BBZSTRID_$formatted_line, \"$line\"}," >> $4
    fi
    ((i++))
done
echo "}};" >> $4

echo "" >> $4
echo "#endif // __EXTRACTED_BBZSTRINGS_H__" >> $4
