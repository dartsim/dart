function ExecuteCommand($command, $command_args)
{
  Write-Host $command $command_args
  Start-Process -FilePath $command -ArgumentList $command_args -Wait -Passthru
}

function InstallPrerequisites($work_dir, $install_dir)
{
  $old_dir = $pwd
  cd $work_dir

  $msi = "dart-prerequisites.msi"
  $uri = "https://github.com/dartsim/dart-prerequisites-windows/raw/bfb42e9e8271bc99f8b070715bd407cb5988c322/DART%205.0-prerequisites-msvc12-md-32bit.msi"
  Invoke-WebRequest $uri -OutFile $msi

  $install_command = "msiexec.exe"
  $install_args    = "/qn /log log.txt /i $msi TARGETDIR=$install_dir"
  ExecuteCommand $install_command $install_args

  cd $old_dir
}

function main()
{
  try 
  {
    InstallPrerequisites "C:/projects" "C:/Golems"
  }
  catch
  {
    throw
  }
}

main