# DART native-Windows launcher for the shared commit guard.
$ErrorActionPreference = "Stop"

try {
  $repoRoot = (git rev-parse --show-toplevel)
  if (-not $repoRoot) {
    throw "git did not return a repository root"
  }
  $entrypoint = Join-Path $repoRoot "scripts/pretool_guard_bridge.py"
  $pixiPython = Join-Path $repoRoot ".pixi/envs/default/python.exe"

  # Windows PowerShell 5.1 promotes native stderr under Stop. Keep it
  # non-terminating only while the child runs so its exact exit code survives.
  $previousErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  try {
    $LASTEXITCODE = $null
    if (Test-Path -LiteralPath $pixiPython -PathType Leaf) {
      & $pixiPython $entrypoint --root $repoRoot
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
      & py -3 $entrypoint --root $repoRoot
    } elseif (Get-Command python -ErrorAction SilentlyContinue) {
      & python $entrypoint --root $repoRoot
    } else {
      Write-Warning "DART pre-tool hook disabled until 'pixi run python scripts/setup_ai.py' creates Python"
      exit 0
    }

    $nativeExitCode = $LASTEXITCODE
  } finally {
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
