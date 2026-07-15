param(
    [string]$LeftBusId = '3-1',
    [string]$RightBusId = '3-2',
    [switch]$NoReleaseTorque
)

$ErrorActionPreference = 'Stop'
$Distro = 'Ubuntu-22.04'
$Usbipd = Join-Path $env:ProgramFiles 'usbipd-win\usbipd.exe'
$Workspace = '/mnt/d/openarm/ros2_ws'

& wsl.exe -d $Distro -u root -- true
$usbList = & $Usbipd list | Out-String
foreach ($busId in @($LeftBusId, $RightBusId)) {
    if ($usbList -notmatch "(?m)^$([regex]::Escape($busId))\s+.*\bAttached\s*$") {
        & $Usbipd attach --wsl $Distro --busid $busId
        if ($LASTEXITCODE -ne 0) { throw "Failed to attach USB device $busId" }
    }
}

$deadline = (Get-Date).AddSeconds(10)
do {
    & wsl.exe -d $Distro -u root -- test -e /dev/ttyUSB0 -a -e /dev/ttyUSB1
    if ($LASTEXITCODE -eq 0) { break }
    Start-Sleep -Milliseconds 200
} while ((Get-Date) -lt $deadline)
if ($LASTEXITCODE -ne 0) { throw 'WSL serial devices did not appear' }

$releaseArg = if ($NoReleaseTorque) { 'false' } else { 'true' }
$command = "source /opt/ros/humble/setup.bash && " +
           "cd $Workspace && " +
           "if [ ! -f install/setup.bash ]; then colcon build --symlink-install --packages-select openarm_servo_teleop; fi && " +
           "source install/setup.bash && " +
           "ros2 run openarm_servo_teleop servo_teleop_node --ros-args -p release_torque:=$releaseArg"

Write-Host 'Starting OpenArm v1 ROS 2 servo teleoperation.'
Write-Host 'Keep both leader arms still and both triggers fully released during zero calibration.'
Write-Host 'Then press both triggers fully and hold them still when prompted.'
& wsl.exe -d $Distro -u root -- bash -lc $command
exit $LASTEXITCODE
