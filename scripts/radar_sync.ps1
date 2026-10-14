$RadarIP    = "10.11.207.112"
$RadarUser  = "Administrator"
$RadarPass  = "123456"
$RemoteShare = "\\$RadarIP\D$\IN\点云显示1\点云显示(1)\wave"
$LocalPath  = Join-Path (Split-Path -Parent $PSScriptRoot) "存itl"
$Interval   = 10
$StartTime = Get-Date

if (-not (Test-Path $LocalPath)) {
    New-Item -ItemType Directory -Path $LocalPath | Out-Null
}

Write-Host "Sync started. Source: $RemoteShare" -ForegroundColor Green
Write-Host "Target: $LocalPath" -ForegroundColor Green
Write-Host "StartTime: $StartTime (只同步此时间之后的新文件)" -ForegroundColor Yellow
Write-Host "Checking every $Interval seconds. Press Ctrl+C to stop.`n" -ForegroundColor Yellow

while ($true) {
    try {
        cmd /c "net use \\$RadarIP\D$ /user:$RadarUser $RadarPass" 2>$null | Out-Null

        if (Test-Path -LiteralPath $RemoteShare) {
            $RemoteFiles = Get-ChildItem -LiteralPath $RemoteShare -File -ErrorAction SilentlyContinue

            foreach ($f in $RemoteFiles) {
                if ($f.LastWriteTime -le $StartTime) {
                    continue
                }

                $LocalFile = Join-Path $LocalPath $f.Name

                $NeedCopy = $false
                if (-not (Test-Path -LiteralPath $LocalFile)) {
                    $NeedCopy = $true
                } else {
                    $LocalItem = Get-Item -LiteralPath $LocalFile
                    if ($f.LastWriteTime -gt $LocalItem.LastWriteTime) {
                        $NeedCopy = $true
                    }
                }

                if ($NeedCopy) {
                    Copy-Item -LiteralPath $f.FullName -Destination $LocalFile -Force
                    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Copied: $($f.Name)" -ForegroundColor Cyan
                }
            }
        } else {
            Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Remote path not accessible." -ForegroundColor Red
        }
    } catch {
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Error: $($_.Exception.Message)" -ForegroundColor Red
    }

    Start-Sleep -Seconds $Interval
}
