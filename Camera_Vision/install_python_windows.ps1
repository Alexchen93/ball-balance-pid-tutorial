$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

# Full Windows runtime and SHA-256 from Python.org's 3.13.17 release manifest.
$url = 'https://www.python.org/ftp/python/3.13.17/python-3.13.17-amd64.zip'
$expectedSha256 = 'b26ca4b4ee51e157836049934d2b8eabd55939be296f3130254251b6df96ec2f'
$target = Join-Path $PSScriptRoot '.python-runtime-3.13.17'
$nonce = [guid]::NewGuid().ToString('N')
$archive = Join-Path $env:TEMP ('camera-vision-python-' + $nonce + '.zip')
$staging = Join-Path $PSScriptRoot ('.python-runtime-setup-' + $nonce)

try {
    if (-not [Environment]::Is64BitOperatingSystem) {
        throw 'Automatic setup requires a 64-bit version of Windows.'
    }
    if (Test-Path -LiteralPath $target) {
        throw 'The Python runtime folder already exists but is incomplete. Rename or remove .python-runtime-3.13.17 and retry.'
    }
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Write-Host 'Downloading the Python runtime from python.org...'
    Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $archive
    Write-Host 'Checking the downloaded runtime...'
    $actualSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
    if ($actualSha256 -ne $expectedSha256) {
        throw 'Downloaded Python archive failed its SHA-256 check. It was not extracted.'
    }
    Write-Host 'Extracting Python locally. This may take a few minutes...'
    Expand-Archive -LiteralPath $archive -DestinationPath $staging
    $python = Join-Path $staging 'python.exe'
    if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw 'The Python archive did not contain python.exe.'
    }
    & $python -c 'import ensurepip, venv; print("Python runtime is ready.")'
    if ($LASTEXITCODE -ne 0) {
        throw 'The extracted Python runtime could not be started.'
    }
    if (Test-Path -LiteralPath $target) {
        throw 'The Python runtime folder appeared during setup. Close other launchers and retry.'
    }
    Move-Item -LiteralPath $staging -Destination $target
    Write-Host 'Python runtime setup completed.'
    exit 0
}
catch {
    Write-Host ("Automatic Python setup failed: " + $_.Exception.Message)
    exit 1
}
finally {
    Remove-Item -LiteralPath $archive -Force -ErrorAction SilentlyContinue
    if (Test-Path -LiteralPath $staging) {
        Remove-Item -LiteralPath $staging -Recurse -Force -ErrorAction SilentlyContinue
    }
}
