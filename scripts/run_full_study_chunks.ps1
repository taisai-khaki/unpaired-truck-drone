$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $repo
$config = Join-Path $repo "config\experiment_full_revised_usync_20261030_budgeted.yaml"
$outputDir = Join-Path $repo "outputs\full_revised_usync_20261030_matched120"
$checkpoint = Join-Path $outputDir "checkpoints\heuristic_study_rows.jsonl"
$runnerLog = Join-Path $outputDir "chunk_runner.log"
$python = (Get-Command python).Source

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$mutex = New-Object System.Threading.Mutex($false, "Local\UTDRP-Full-Study-Budgeted")
if (-not $mutex.WaitOne(0)) {
    Add-Content -LiteralPath $runnerLog -Value ("[{0}] runner already active; exiting" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
    exit 0
}

try {
    Add-Content -LiteralPath $runnerLog -Value ("[{0}] runner start pid={1}" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $PID)
    for ($chunk = 1; $chunk -le 800; $chunk++) {
        Add-Content -LiteralPath $runnerLog -Value ("[{0}] chunk={1} start" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $chunk)
        & $python -u -m src.cli --config $config run-heuristic-study
        if ($LASTEXITCODE -ne 0) {
            Add-Content -LiteralPath $runnerLog -Value ("[{0}] chunk={1} failed exit_code={2}" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $chunk, $LASTEXITCODE)
            exit $LASTEXITCODE
        }
        $rows = Get-Content -LiteralPath $checkpoint
        $completed = ($rows | Where-Object { $_ -match '"method_name"\s*:\s*"nils_r"' }).Count
        Add-Content -LiteralPath $runnerLog -Value ("[{0}] chunk={1} completed_instances={2}/720" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $chunk, $completed)
        if ($completed -ge 720) {
            break
        }
    }
} finally {
    Add-Content -LiteralPath $runnerLog -Value ("[{0}] runner end" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
    $mutex.ReleaseMutex()
    $mutex.Dispose()
}
