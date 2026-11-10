#!/usr/bin/env bash

function are_files_equal() {
    local file1="$1"
    local file2="$2"

    # Kiểm tra xem tệp .last hoặc .lastso tồn tại
    if [[ ! -f "$file2" && ! -f "${file2}so" ]]; then
        return 1
    fi

    # Lựa chọn tệp .last hoặc .lastso tùy thuộc vào tùy chọn --module
    local last_file="${file2}"
    if [[ "$3" == "--module" ]]; then
        last_file="${file2}so"
    fi

    # Kiểm tra xem tệp .last hoặc .lastso tồn tại
    if [[ ! -f "$last_file" ]]; then
        return 1
    fi

    # Sử dụng diff để so sánh hai tệp và đếm số dòng khác nhau
    local diff_output=$(diff -U 0 "$file1" "$last_file" | tail -n +3 | grep -c "^@")

    # Nếu số dòng khác nhau là 0, hai tệp giống nhau
    if [ "$diff_output" -eq 0 ]; then
        return 0  # Trả về 0 nếu giống nhau
    else
        return 1  # Trả về 1 nếu khác nhau
    fi
}

# Function để kiểm tra và build một tệp Python cụ thể
function build_python_file() {
    local ROS_PACKAGE="$1"
    local PACKAGE_PATH=$(rospack find $ROS_PACKAGE)
    local PY_FILE="$2"
    local RELEASE_DIR="release"
    local FILE_NAME=$(basename "$PY_FILE")
    local LAST_FILE="$PACKAGE_PATH/$RELEASE_DIR/${FILE_NAME%.*}.last"
    local Nuitka_Options="--output-dir=$RELEASE_DIR"
    local FULL_PY_FILE="$PACKAGE_PATH/$PY_FILE"

    echo FULL_PY_FILE: $FULL_PY_FILE
    echo FILE_NAME: $FILE_NAME
    echo LAST_FILE: $LAST_FILE

    # Kiểm tra xem tệp .last hoặc .lastso tồn tại
    if [[ ! -f "$LAST_FILE" && ! -f "${LAST_FILE}so" ]]; then
        echo "Building $FILE_NAME (no .last or .lastso file found)"
    elif are_files_equal "$FULL_PY_FILE" "$LAST_FILE" "$3"; then
        echo "Skipping $FILE_NAME (no changes)"
        return
    else
        echo "Building $FILE_NAME"
    fi

    # Xác định tùy chọn --module nếu cần
    if [[ "$3" == "--module" ]]; then
        Nuitka_Options="$Nuitka_Options --module"
        LAST_FILE="$PACKAGE_PATH/$RELEASE_DIR/${FILE_NAME%.*}.lastso"  # Sử dụng tên .lastso khi sử dụng --module
    fi

    # Sử dụng Nuitka để build tệp Python với các tùy chọn
    nuitka3 $Nuitka_Options "$FULL_PY_FILE"

    # Sao chép nội dung của tệp Python vào tệp .last hoặc .lastso
    cp "$FULL_PY_FILE" "$LAST_FILE"
}