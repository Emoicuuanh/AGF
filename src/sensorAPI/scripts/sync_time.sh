#!/bin/bash

# Địa chỉ máy chủ NTP
NTP_SERVER="10.0.108.16"

# Mật khẩu sudo của bạn (không an toàn)
PASSWORD="mkac"

# Cung cấp mật khẩu để thực thi lệnh sudo
echo $PASSWORD | sudo -S ntpdate $NTP_SERVER
echo $PASSWORD | sudo -S hwclock --systohc
