param(
    [Parameter(Mandatory)][ValidatePattern('^COM[1-9][0-9]*$')][string]$Port,
    [ValidateRange(1, 120)][int]$Seconds = 8,
    [ValidateRange(9600, 2000000)][int]$Baud = 115200,
    [string]$OutputPath
)
$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud)
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 250
$writer = $null
try {
    $serial.Open()
    if ($OutputPath) {
        $writer = [System.IO.StreamWriter]::new($OutputPath, $false, [Text.UTF8Encoding]::new($false))
    }
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        try {
            $line = $serial.ReadLine().Trim()
            if (!$line) { continue }
            if ($writer) { $writer.WriteLine($line) }
            $line
        } catch [TimeoutException] {}
    }
} finally {
    if ($writer) { $writer.Dispose() }
    $serial.Dispose()
}
