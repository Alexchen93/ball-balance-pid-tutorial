$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

# Python.org's Windows x64 installer and the SHA-256 published on its release page.
$url = 'https://www.python.org/ftp/python/3.13.17/python-3.13.17-amd64.exe'
$expectedSha256 = '5da012101ba2e17113a9a1cd901e9ce1441086f2b7ecb5de06ec94163899a190'
$target = Join-Path $PSScriptRoot '.python-win'
$python = Join-Path $target 'python.exe'
$installLog = Join-Path $PSScriptRoot 'python-install.log'
$installer = Join-Path $env:TEMP ('camera-vision-python-' + [guid]::NewGuid().ToString('N') + '.exe')

try {
    if (-not [Environment]::Is64BitOperatingSystem) {
        throw 'Automatic installation requires a 64-bit version of Windows.'
    }
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Write-Host 'Downloading Python from python.org...'
    Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $installer
    Write-Host 'Checking the downloaded installer...'
    $actualSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $installer).Hash.ToLowerInvariant()
    if ($actualSha256 -ne $expectedSha256) {
        throw 'Downloaded Python installer failed its SHA-256 check. It was not run.'
    }
    Write-Host 'Installing Python for this user. This may take several minutes...'
    $installerArguments = @(
        '/quiet',
        'InstallAllUsers=0',
        ('TargetDir="{0}"' -f $target),
        'Include_launcher=0',
        'Include_pip=1',
        'Include_test=0',
        'Include_doc=0',
        'Include_tcltk=0',
        'Shortcuts=0',
        'PrependPath=0',
        'AppendPath=0',
        '/log',
        ('"{0}"' -f $installLog)
    )
    $process = Start-Process -FilePath $installer -ArgumentList $installerArguments -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "Python installer failed with exit code $($process.ExitCode). Check python-install.log."
    }
    if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw 'Python was not installed in the expected folder. Check python-install.log.'
    }
    & $python --version
    if ($LASTEXITCODE -ne 0) {
        throw 'The installed Python executable could not be started.'
    }
    Write-Host 'Python installation completed.'
    exit 0
}
catch {
    Write-Host ("Automatic Python installation failed: " + $_.Exception.Message)
    exit 1
}
finally {
    Remove-Item -LiteralPath $installer -Force -ErrorAction SilentlyContinue
}
