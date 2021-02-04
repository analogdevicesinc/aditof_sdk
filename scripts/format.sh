#!/bin/bash

############################################################################
# This script will format all source files (.h and .cpp) that are not
# mentioned in .clangformatignore
# Note that the script has to be ran from the same folder where 
# .clangformatignore is
###########################################################################

############################################################################
# Check if the file given as input has .h or .cpp extension
############################################################################
is_source_file() {
    local file="$1"

    EXTENSIONS=".h .cpp"

    for extension in $EXTENSIONS; do
        [[ "${file: -2}" == "$extension" || "${file: -4}" == "$extension" ]] && return 0
    done;

    return 1
}


############################################################################
# Check if the file passed as arguments is ignored or not by clang format
############################################################################
is_not_ignored() {
    local file="$1"

    fileData=`cat .clangformatignore`

    for entry in $fileData; do
        if [ -d "${entry}" ]; then
            pushd ${entry}
            fileName=`basename ${file}`
            found=$(find -name ${fileName} | wc -l)
            if [ ${found} -gt 0 ]; then
                popd
                return 1
            else
                popd
            fi
        else
            if [ -f "${entry}" ]; then
                if [ "${file}" == "${entry}" ]; then
                    return 1
                fi
            fi
        fi
    done;
    return 0
}


format_all() {
	git ls-tree -r --name-only HEAD | 
		while read -r file; do
        		if is_source_file "$file" && is_not_ignored "$file"
      			then
            			/usr/bin/clang-format-6.0 -i "$file"
			fi
	
			done;
}

format_all

