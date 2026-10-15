param(
    [string]$SourcePath = '\\10.11.207.112\D$\IN\点云显示1\点云显示(1)\wave',
    [string]$DestinationPath = (Join-Path (Split-Path -Parent $PSScriptRoot) '存itl'),
    [int]$IntervalSeconds = 10,
    [double]$StableSeconds = 3,
    [switch]$SyncExisting,
    [int]$MaxScans = 0
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
$StartTime = [DateTime]::UtcNow
$SeenFiles = @{}
$Connected = $false
$Scans = 0
$DestinationPath = [IO.Path]::GetFullPath($DestinationPath)
New-Item -ItemType Directory -Path $DestinationPath -Force | Out-Null
Write-Output "ITL watcher ready. Source: $SourcePath; target: $DestinationPath; sync_existing=$SyncExisting"
while ($true) {
    $TempFile = $null
    try {
        if (-not (Test-Path -LiteralPath $SourcePath)) {
            # Preserve existing IPC credentials without logging them or changing remote files.
            if ($SourcePath -match '^\\\\([^\\]+)\\([^\\]+)') {
                $ShareRoot = "\\$($Matches[1])\$($Matches[2])"
                $RadarUser = if ($env:JUSHEN_RADAR_USER) { $env:JUSHEN_RADAR_USER } else { 'Administrator' }
                $RadarPass = if ($env:JUSHEN_RADAR_PASSWORD) { $env:JUSHEN_RADAR_PASSWORD } else { '123456' }
                & net.exe use $ShareRoot "/user:$RadarUser" $RadarPass 2>&1 | Out-Null
            }
        }
        if (-not (Test-Path -LiteralPath $SourcePath)) { throw "Remote path not accessible: $SourcePath" }
        if (-not $Connected) {
            Write-Output 'CONNECTED: ITL source is accessible; watching for stable new files.'
            $Connected = $true
        }
        $RemoteFiles = @(Get-ChildItem -LiteralPath $SourcePath -File | Where-Object { $_.Extension -ieq '.itl' })
        foreach ($File in $RemoteFiles) {
            try {
            if (-not $SyncExisting -and $File.LastWriteTimeUtc -le $StartTime) { continue }
            if ($File.Length -eq 0) { continue }
            $Signature = "$($File.Length):$($File.LastWriteTimeUtc.Ticks)"
            $Prior = $SeenFiles[$File.Name]
            if ($null -eq $Prior -or $Prior.Signature -ne $Signature) {
                $SeenFiles[$File.Name] = @{ Signature = $Signature; Since = [DateTime]::UtcNow }
                continue
            }
            if (([DateTime]::UtcNow - $Prior.Since).TotalSeconds -lt $StableSeconds) { continue }
            $LocalFile = Join-Path $DestinationPath $File.Name
            if (Test-Path -LiteralPath $LocalFile) {
                $Local = Get-Item -LiteralPath $LocalFile
                if ($Local.Length -eq $File.Length -and $Local.LastWriteTimeUtc.Ticks -eq $File.LastWriteTimeUtc.Ticks) { continue }
            }
            $TempFile = Join-Path $DestinationPath ('.' + [Guid]::NewGuid().ToString('N') + '.part')
            Copy-Item -LiteralPath $File.FullName -Destination $TempFile
            $After = Get-Item -LiteralPath $File.FullName
            $Copied = Get-Item -LiteralPath $TempFile
            if ("$($After.Length):$($After.LastWriteTimeUtc.Ticks)" -ne $Signature -or $Copied.Length -ne $File.Length) {
                Remove-Item -LiteralPath $TempFile
                $TempFile = $null
                $SeenFiles.Remove($File.Name)
                Write-Output "DEFER: $($File.Name) changed while copying."
                continue
            }
            # Rename within the destination directory hides partially copied ITL from the converter.
            Move-Item -LiteralPath $TempFile -Destination $LocalFile -Force
            $TempFile = $null
            Write-Output "COPIED: $($File.Name) ($($File.Length) bytes) -> $LocalFile"
            } catch {
                if ($TempFile -and (Test-Path -LiteralPath $TempFile)) { Remove-Item -LiteralPath $TempFile -ErrorAction SilentlyContinue }
                $TempFile = $null
                Write-Output "FILE_ERROR: $($File.Name): $($_.Exception.Message)"
            }
        }
    } catch {
        $Connected = $false
        if ($TempFile -and (Test-Path -LiteralPath $TempFile)) { Remove-Item -LiteralPath $TempFile -ErrorAction SilentlyContinue }
        Write-Output "ERROR: $($_.Exception.Message)"
    }
    $Scans++
    if ($MaxScans -gt 0 -and $Scans -ge $MaxScans) { break }
    Start-Sleep -Seconds ([Math]::Max(1, $IntervalSeconds))
}
