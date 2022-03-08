@echo off

call C:\opt\ros\noetic\x64\setup.bat

for /f "tokens=3 delims=: " %%I in ('netsh interface IPv4 show addresses "vEthernet (WSL)" ^| findstr /C:"IP Address"') do set WSL_HOST_IP=%%I

set ROS_MASTER_URI=http://%WSL_HOST_IP%:11311
set ROS_IP=%WSL_HOST_IP%
echo ROS Initialized
echo ROS_MASTER_URI=%ROS_MASTER_URI%
echo ROS_MASTER_IP=%ROS_IP%