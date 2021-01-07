#!/bin/sh

# Delete existing file
echo "/* DO NOT EDIT */" > $3
echo "/* THIS IS A GENERATED FILE */" >> $3
echo "" >> $3

# Writing content
echo "#ifndef __EXTRACTED_BBZSTRINGS_H__" >> $3
echo "#define __EXTRACTED_BBZSTRINGS_H__" >> $3
echo "" >> $3

echo "#include \"$2\"" >> $3
echo "" >> $3

echo "#include <array>" >> $3
echo "#include <cstdint>" >> $3
echo "#include <utility>" >> $3
echo "" >> $3

array_size=$(grep "^'\w" $1 | wc -l)
echo "#define BBZSTRING_ARRAY_SIZE ($array_size)" >> $3
echo "" >> $3

echo "const std::array<const std::pair<const int, const char*>, BBZSTRING_ARRAY_SIZE> test = {{"  >> $3
grep "^'" $1 | cut -c 2- | while read -r line
do
    formatted_line=$(echo $line | sed -e "s/[^a-zA-Z0-9_]/_/g")
    echo "    {BBZSTRID_$formatted_line, \"$line\"}," >> $3
done
echo "}};" >> $3

echo "" >> $3
echo "#endif // __EXTRACTED_BBZSTRINGS_H__" >> $3
