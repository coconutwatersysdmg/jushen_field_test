param(
    [string]$RemoteShare = "",
    [string]$LocalPath = "",
    [double]$Interval = 10,
    [int]$ParentProcessId = 0,
    [int]$MaxScans = 0,
    [switch]$SkipAuthentication
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding
$RadarIP = "10.11.207.112"
$RadarUser = "Administrator"
$RadarPass = "123456"
if (-not $RemoteShare) {
    $RemoteShare = "\\$RadarIP\D$\IN\点云显示1\点云显示(1)\wave"
}
if (-not $LocalPath) {
    $LocalPath = Join-Path (Split-Path -Parent $PSScriptRoot) "存itl"
}
if ($Interval -le 0) { throw "Interval must be positive" }

function Write-SyncLog([string]$Message, [string]$Level = "INFO") {
    Write-Output "[RADAR_SYNC][$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')][$Level] $Message"
}

New-Item -ItemType Directory -Path $LocalPath -Force | Out-Null
Write-SyncLog "Sync started. Source: $RemoteShare"
Write-SyncLog "Target: $LocalPath"
Write-SyncLog "All filenames and file types; no startup-time cutoff. Checking every $Interval seconds."

$Authenticated = [bool]$SkipAuthentication
$Scan = 0
while ($MaxScans -le 0 -or $Scan -lt $MaxScans) {
    if ($ParentProcessId -gt 0 -and -not (Get-Process -Id $ParentProcessId -ErrorAction SilentlyContinue)) {
        Write-SyncLog "Parent application exited; stopping monitor."
        break
    }
    $Scan++
    $Copied = 0
    $Unchanged = 0
    $Failed = 0
    Write-SyncLog "Scan #$Scan starting: $RemoteShare"
    try {
        if (-not $Authenticated) {
            Write-SyncLog "Connecting SMB share \\$RadarIP\D$ as $RadarUser"
            # Keep net.exe's diagnostics, but never print the password/command line.
            $AuthOutput = & net.exe use "\\$RadarIP\D$" "/user:$RadarUser" $RadarPass 2>&1
            $AuthCode = $LASTEXITCODE
            foreach ($Line in $AuthOutput) { Write-SyncLog "SMB: $Line" }
            $Authenticated = ($AuthCode -eq 0)
            if (-not $Authenticated) {
                Write-SyncLog "SMB authentication failed, exit=$AuthCode; checking existing connection." "ERROR"
            }
        }
        if (-not (Test-Path -LiteralPath $RemoteShare -PathType Container)) {
            throw "Remote path not accessible: $RemoteShare"
        }
        $RemoteFiles = @(Get-ChildItem -LiteralPath $RemoteShare -File -Force -ErrorAction Stop)
        Write-SyncLog "Share accessible; discovered $($RemoteFiles.Count) files."
        foreach ($File in $RemoteFiles) {
            $TemporaryFile = $null
            $SourceStream = $null
            $TargetStream = $null
            try {
                $LocalFile = Join-Path $LocalPath $File.Name
                if (Test-Path -LiteralPath $LocalFile -PathType Leaf) {
                    $LocalItem = Get-Item -LiteralPath $LocalFile -Force
                    if ($File.Length -eq $LocalItem.Length -and $File.LastWriteTimeUtc -eq $LocalItem.LastWriteTimeUtc) {
                        $Unchanged++
                        continue
                    }
                }
                Write-SyncLog "Copying: $($File.Name) ($($File.Length) bytes)"
                # Deny concurrent writers/deletion for the entire transfer. A writer
                # that permits readers may otherwise publish an incomplete PCD.
                $SourceStream = [System.IO.File]::Open(
                    $File.FullName, [System.IO.FileMode]::Open,
                    [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read
                )
                $TemporaryFile = Join-Path $LocalPath (".radar-" + [guid]::NewGuid().ToString('N') + ".syncing")
                $TargetStream = [System.IO.File]::Open(
                    $TemporaryFile, [System.IO.FileMode]::CreateNew,
                    [System.IO.FileAccess]::Write, [System.IO.FileShare]::None
                )
                $SourceStream.CopyTo($TargetStream)
                $TargetStream.Dispose()
                $TargetStream = $null
                $AfterCopy = Get-Item -LiteralPath $File.FullName -Force -ErrorAction Stop
                if ($File.Length -ne $AfterCopy.Length -or $File.LastWriteTimeUtc -ne $AfterCopy.LastWriteTimeUtc) {
                    throw "Source is still changing; retry next scan: $($File.Name)"
                }
                Move-Item -LiteralPath $TemporaryFile -Destination $LocalFile -Force -ErrorAction Stop
                $TemporaryFile = $null
                (Get-Item -LiteralPath $LocalFile -Force).LastWriteTimeUtc = $File.LastWriteTimeUtc
                $Copied++
                Write-SyncLog "Copied: $($File.Name) -> $LocalFile"
            } catch {
                $Failed++
                Write-SyncLog "Copy failed: $($File.Name): $($_.Exception.Message); retry next scan." "ERROR"
            } finally {
                if ($TargetStream) { $TargetStream.Dispose() }
                if ($SourceStream) { $SourceStream.Dispose() }
                if ($TemporaryFile -and (Test-Path -LiteralPath $TemporaryFile)) {
                    Remove-Item -LiteralPath $TemporaryFile -Force -ErrorAction SilentlyContinue
                }
            }
        }
    } catch {
        $Failed++
        $Authenticated = [bool]$SkipAuthentication
        Write-SyncLog "Scan failed: $($_.Exception.Message); retry in $Interval seconds." "ERROR"
    }
    Write-SyncLog "Scan #$Scan complete: copied=$Copied unchanged=$Unchanged failed=$Failed"
    if ($MaxScans -gt 0 -and $Scan -ge $MaxScans) { break }
    Start-Sleep -Milliseconds ([int]($Interval * 1000))
}
Write-SyncLog "Sync stopped."
