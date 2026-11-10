#!/usr/bin/env bash

PACKAGE="agv_mongodb"
BOLD=$(tput bold)
NORMAL=$(tput sgr0)
GREEN='\033[0;32m'
echo -e "${GREEN}${BOLD}Building package \"$PACKAGE\"${NORMAL}"
# Sử dụng rospack find để tìm đường dẫn đến package agv_common_library
AGV_COMMON_LIBRARY_PATH=$(rospack find agv_common_library)

# Kiểm tra xem AGV_COMMON_LIBRARY_PATH có tồn tại
if [ -z "$AGV_COMMON_LIBRARY_PATH" ]; then
    echo "Package 'agv_common_library' not found."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR
echo "Current pwd:" $(pwd)

mkdir -p release

# Nạp hàm build_python_file từ nuitka_build_diff.sh trong package agv_common_library
source "$AGV_COMMON_LIBRARY_PATH/scripts/nuitka_build_diff.sh"

# Gọi hàm build_python_file với đối số tương ứng
build_python_file $PACKAGE "scripts/mongodb.py" "--module"