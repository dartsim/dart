function ExecuteCommand($command, $command_args)
{
  Write-Host $command $command_args
  Start-Process -FilePath $command -ArgumentList $command_args -Wait -Passthru
}

function InstallPrerequisites($work_dir, $install_dir)
{
  $old_dir = $pwd
  cd $work_dir

  $msi = "dart-dependencies.msi"
  $uri = "https://github.com/dartsim/dart-prerequisites-windows-installers/raw/master/02/DART-dependencies-msvc14-md-32bit.msi"
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
