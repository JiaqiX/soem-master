#!/bin/bash

# clang-format, version v15 is required
find ./src -regex '.*\.cc\|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file

# cmake-format, apt install cmake-format
find ./ -regex '.*\.cmake\|.*CMakeLists\.txt$' -and -not -regex '\./build/.*\|\./cmake-build-debug/.*\|\./document/.*' | xargs cmake-format -c ./.cmake-format.py -i

# autopep8, apt install python3-autopep8
find ./ -regex '.*\.py' -and -not -regex '\./build/.*\|\./cmake-build-debug/.*\|\./document/.*' | xargs autopep8 -i --global-config ./.pycodestyle
