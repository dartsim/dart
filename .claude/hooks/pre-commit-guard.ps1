# DART native-Windows launcher for the shared commit guard.
$ErrorActionPreference = "Stop"

try {
  # Read stdin directly: Windows PowerShell's `$input` enumerator is not
  # reliable when this script is invoked through a nested `-Command` pipeline.
  $payload = [Console]::In.ReadToEnd()
  $pipelineInput = if ($payload.Length -gt 0) { @($payload) } else { @() }
  $repoRoot = (git rev-parse --show-toplevel)
  if (-not $repoRoot) {
    throw "git did not return a repository root"
  }
  $entrypoint = Join-Path $repoRoot "scripts/pretool_guard_bridge.py"
  $pixiPython = Join-Path $repoRoot ".pixi/envs/default/python.exe"

  # Windows PowerShell 5.1 promotes native stderr under Stop. Keep it
  # non-terminating only while the child runs so its exact exit code survives.
  # Use BOM-less UTF-8 when piping the hook JSON into the native interpreter.
  $previousErrorActionPreference = $ErrorActionPreference
  $previousOutputEncoding = $OutputEncoding
  $ErrorActionPreference = "Continue"
  $OutputEncoding = New-Object System.Text.UTF8Encoding($false)
  try {
    # Qualify the automatic variable explicitly. An unscoped assignment here
    # would create a script-scoped shadow on Windows PowerShell 5.1.
    $global:LASTEXITCODE = $null
    if (Test-Path -LiteralPath $pixiPython -PathType Leaf) {
      if ($pipelineInput.Count -gt 0) {
        $payload | & $pixiPython $entrypoint --root $repoRoot
      } else {
        & $pixiPython $entrypoint --root $repoRoot
      }
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
      if ($pipelineInput.Count -gt 0) {
        $payload | & py -3 $entrypoint --root $repoRoot
      } else {
        & py -3 $entrypoint --root $repoRoot
      }
    } elseif (Get-Command python -ErrorAction SilentlyContinue) {
      if ($pipelineInput.Count -gt 0) {
        $payload | & python $entrypoint --root $repoRoot
      } else {
        & python $entrypoint --root $repoRoot
      }
    } else {
      Write-Warning "DART pre-tool hook disabled until 'pixi run python scripts/setup_ai.py' creates Python"
      exit 0
    }

    $nativeExitCode = $global:LASTEXITCODE
  } finally {
    $OutputEncoding = $previousOutputEncoding
    $ErrorActionPreference = $previousErrorActionPreference
  }

  if ($null -eq $nativeExitCode -or $nativeExitCode -ne 0) {
    exit 2
  }
  exit 0
} catch {
  [Console]::Error.WriteLine("DART pre-tool hook: {0}", $_)
  exit 2
}
