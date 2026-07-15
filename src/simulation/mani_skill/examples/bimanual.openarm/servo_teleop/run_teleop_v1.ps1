param(
    [string]$LeftBusId = '3-1',
    [string]$RightBusId = '3-2',
    [switch]$Headless,
    [switch]$NoReleaseTorque,
    [Nullable[int]]$Duration
)

$ErrorActionPreference = 'Stop'

$Distro = 'Ubuntu-22.04'
$Usbipd = Join-Path $env:ProgramFiles 'usbipd-win\usbipd.exe'
$Python = '/opt/openarm-venv/bin/python'
$TeleopScript = '/mnt/d/openarm/servo_teleop/mujoco_teleop_v1.py'
$BusIds = @($LeftBusId, $RightBusId)

if (-not (Test-Path -LiteralPath $Usbipd)) {
    throw "usbipd-win was not found at $Usbipd"
}
if ($LeftBusId -eq $RightBusId) {
    throw 'LeftBusId and RightBusId must be different'
}

Write-Host "Starting $Distro for OpenArm v1..."
& wsl.exe -d $Distro -u root -- true
if ($LASTEXITCODE -ne 0) {
    throw "Could not start $Distro"
}

$usbList = & $Usbipd list | Out-String
foreach ($busId in $BusIds) {
    if ($usbList -notmatch "(?m)^$([regex]::Escape($busId))\s+.*\bAttached\s*$") {
        Write-Host "Attaching USB device $busId..."
        & $Usbipd attach --wsl $Distro --busid $busId
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to attach USB device $busId"
        }
    }
}

$deadline = (Get-Date).AddSeconds(10)
do {
    & wsl.exe -d $Distro -u root -- test -e /dev/ttyUSB0 -a -e /dev/ttyUSB1
    if ($LASTEXITCODE -eq 0) {
        break
    }
    Start-Sleep -Milliseconds 200
} while ((Get-Date) -lt $deadline)
if ($LASTEXITCODE -ne 0) {
    throw 'WSL did not create /dev/ttyUSB0 and /dev/ttyUSB1 within 10 seconds'
}

$teleopArgs = @($TeleopScript)
if ($NoReleaseTorque) {
    $teleopArgs += '--no-release-torque'
}
if ($Headless) {
    $teleopArgs += '--headless'
}
if ($PSBoundParameters.ContainsKey('Duration')) {
    $teleopArgs += @('--duration', $Duration.ToString())
}

Write-Host 'Starting OpenArm v1 MuJoCo teleoperation.'
Write-Host 'Keep both leader arms still during the two-second zero calibration.'
& wsl.exe -d $Distro -u root -- $Python @teleopArgs
exit $LASTEXITCODE
