#!/usr/bin/env bash

PACKAGE="agv_common_library"
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
build_python_file $PACKAGE "scripts/common_function.py" "--module"
build_python_file $PACKAGE "scripts/find_port_and_run.py"
build_python_file $PACKAGE "scripts/list_slot_serial.py"
build_python_file $PACKAGE "scripts/node_manager.py"
build_python_file $PACKAGE "scripts/module_manager.py" "--module"
build_python_file $PACKAGE "scripts/rviz_button_converter.py"
build_python_file $PACKAGE "scripts/get_hardware_license.py"
