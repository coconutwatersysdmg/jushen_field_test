param(
    [ValidateSet('inspect', 'radar', 'hotspot')][string]$Mode = 'inspect',
    [Parameter(Mandatory = $true)][string]$ConfigPath,
    [string]$ResultPath
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding

function Invoke-Netsh {
    param([string[]]$CommandArguments)
    $output = & "$env:SystemRoot\System32\netsh.exe" @CommandArguments 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "netsh failed: $($output -join [Environment]::NewLine)"
    }
}

function Get-Ssid {
    # SSID and GUID are invariant labels even on Chinese Windows. Match the configured
    # adapter's GUID, rather than accidentally reading a second wireless adapter.
    $output = (& "$env:SystemRoot\System32\netsh.exe" wlan show interfaces 2>&1) -join "`n"
    if ($LASTEXITCODE -ne 0) { throw "Cannot read Wi-Fi interfaces: $output" }
    $adapter = Get-NetAdapter -Name $settings.interface -ErrorAction Stop
    foreach ($block in ($output -split '\r?\n\s*\r?\n')) {
        if ($block -match [regex]::Escape(([string]$adapter.InterfaceGuid).Trim('{}'))) {
            if ($block -match '(?m)^\s*SSID\s*:\s*(.*?)\s*$') { return $Matches[1] }
        }
    }
    return ''
}

function Get-NetworkState {
    $ssid = Get-Ssid
    $ipv4 = Get-NetIPInterface -InterfaceAlias $settings.interface -AddressFamily IPv4
    # Enumerate first, then filter: querying an interface with no IPv4 addresses
    # directly raises ObjectNotFound during DHCP convergence instead of returning [].
    $ipEntries = @(Get-NetIPAddress -AddressFamily IPv4 |
        Where-Object { $_.InterfaceAlias -eq $settings.interface -and $_.AddressState -eq 'Preferred' })
    $addresses = @($ipEntries | Select-Object -ExpandProperty IPAddress)
    $ready = $false
    $network = 'other'
    if ($ssid -eq $settings.radar_profile) {
        $network = 'radar'
        $matchingIp = @($ipEntries | Where-Object {
            $_.IPAddress -eq $settings.radar_address -and $_.PrefixLength -eq $radarPrefix
        })
        $routes = @(Get-NetRoute -AddressFamily IPv4 |
            Where-Object {
                $_.InterfaceAlias -eq $settings.interface -and
                $_.DestinationPrefix -eq '0.0.0.0/0' -and $_.NextHop -eq $settings.radar_gateway
            })
        $ready = ($ipv4.Dhcp -eq 'Disabled' -and $matchingIp.Count -gt 0 -and $routes.Count -gt 0)
    } elseif ($ssid -eq $settings.hotspot_profile) {
        $network = 'hotspot'
        $usable = @($addresses | Where-Object { $_ -notmatch '^(169\.254\.|127\.|0\.)' })
        $adapter = Get-NetAdapter -Name $settings.interface
        $guid = ([guid]$adapter.InterfaceGuid).ToString('B')
        $dnsKey = "HKLM:\SYSTEM\CurrentControlSet\Services\Tcpip\Parameters\Interfaces\$guid"
        $dnsSettings = Get-ItemProperty -LiteralPath $dnsKey -ErrorAction Stop
        $automaticDns = [string]::IsNullOrWhiteSpace([string]$dnsSettings.NameServer)
        $ready = ($ipv4.Dhcp -eq 'Enabled' -and $usable.Count -gt 0 -and $automaticDns)
    }
    return @{
        ssid = $ssid; interface = $settings.interface; addresses = $addresses
        dhcp = [string]$ipv4.Dhcp; network = $network; ready = [bool]$ready
    }
}

function Save-Result {
    param($Value)
    $json = $Value | ConvertTo-Json -Depth 6 -Compress
    if ($ResultPath) {
        # The unelevated GUI creates this file. Open it in place, keeping its owner,
        # ACL and integrity label; do not create/replace an elevated-owned file.
        $stream = [System.IO.File]::Open($ResultPath, [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
        try {
            $bytes = [System.Text.UTF8Encoding]::new($false).GetBytes($json)
            $stream.SetLength(0)
            $stream.Write($bytes, 0, $bytes.Length)
            $stream.Flush()
        } finally { $stream.Dispose() }
    } else { [Console]::WriteLine($json) }
}

try {
    $settings = Get-Content -LiteralPath $ConfigPath -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($key in @('interface', 'radar_profile', 'hotspot_profile', 'radar_address', 'radar_mask', 'radar_gateway')) {
        if ([string]::IsNullOrWhiteSpace([string]$settings.$key)) { throw "Missing configuration: $key" }
    }
    if ($settings.radar_profile -eq $settings.hotspot_profile) { throw 'Wi-Fi profiles must be different.' }
    foreach ($key in @('radar_address', 'radar_mask', 'radar_gateway')) {
        $parsedIp = [System.Net.IPAddress]::Parse([string]$settings.$key)
        if ($parsedIp.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) {
            throw "Invalid IPv4 configuration: $key"
        }
    }
    $maskBits = (([System.Net.IPAddress]::Parse($settings.radar_mask)).GetAddressBytes() |
        ForEach-Object { [Convert]::ToString($_, 2).PadLeft(8, '0') }) -join ''
    if ($maskBits -notmatch '^1+0*$') { throw 'Invalid subnet mask.' }
    $radarPrefix = ($maskBits -replace '0', '').Length
    if ($Mode -eq 'inspect') {
        Save-Result (Get-NetworkState)
        exit 0
    }
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = [Security.Principal.WindowsPrincipal]::new($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Administrator permission is required.'
    }
    $targetProfile = if ($Mode -eq 'radar') { $settings.radar_profile } else { $settings.hotspot_profile }
    # Preflight profile and adapter existence before mutating anything.
    $null = Get-NetAdapter -Name $settings.interface -ErrorAction Stop
    Invoke-Netsh @('wlan', 'show', 'profile', "name=$targetProfile", "interface=$($settings.interface)")
    Invoke-Netsh @('wlan', 'connect', "name=$targetProfile", "ssid=$targetProfile", "interface=$($settings.interface)")
    $deadline = (Get-Date).AddSeconds(30)
    while ((Get-Ssid) -ne $targetProfile) {
        if ((Get-Date) -gt $deadline) { throw "Wi-Fi connection timed out: $targetProfile. IP configuration was not applied." }
        Start-Sleep -Milliseconds 500
    }
    if ($Mode -eq 'radar') {
        # Preserve the exact, field-proven settings from the desktop radar BAT.
        Invoke-Netsh @('interface', 'ip', 'set', 'address', $settings.interface, 'static',
            $settings.radar_address, $settings.radar_mask, $settings.radar_gateway)
    } else {
        Invoke-Netsh @('interface', 'ip', 'set', 'address', $settings.interface, 'dhcp')
        Invoke-Netsh @('interface', 'ip', 'set', 'dns', $settings.interface, 'dhcp')
    }
    $deadline = (Get-Date).AddSeconds(45)
    do {
        $state = Get-NetworkState
        if ($state.network -eq $Mode -and $state.ready) { break }
        if ((Get-Date) -gt $deadline) { throw 'Wi-Fi or IPv4 configuration did not become ready in time.' }
        Start-Sleep -Milliseconds 750
    } while ($true)
    Save-Result @{ ok = $true; state = $state }
    exit 0
} catch {
    Save-Result @{ ok = $false; message = $_.Exception.Message }
    exit 1
}
