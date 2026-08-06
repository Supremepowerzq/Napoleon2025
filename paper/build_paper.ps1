param(
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'
$miktexBin = Join-Path $env:LOCALAPPDATA 'Programs\MiKTeX\miktex\bin\x64'
$perlRoot = Join-Path $env:LOCALAPPDATA 'Programs\StrawberryPerlPortable'
$toolPaths = @(
    (Join-Path $perlRoot 'perl\site\bin'),
    (Join-Path $perlRoot 'perl\bin'),
    (Join-Path $perlRoot 'c\bin'),
    $miktexBin
)

foreach ($path in $toolPaths) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required LaTeX tool path is missing: $path"
    }
}

$env:Path = ($toolPaths + @($env:Path)) -join ';'
$env:LC_ALL = $null
$env:LC_CTYPE = $null
$env:LANG = $null

Push-Location $PSScriptRoot
try {
    if ($Clean) {
        & latexmk -C 'main_ral_layered_autonav.tex'
        if ($LASTEXITCODE -ne 0) {
            throw "latexmk cleanup failed with exit code $LASTEXITCODE"
        }
    }

    & latexmk -pdf -interaction=nonstopmode -file-line-error -halt-on-error `
        'main_ral_layered_autonav.tex'
    if ($LASTEXITCODE -ne 0) {
        throw "LaTeX build failed with exit code $LASTEXITCODE"
    }

    $pdf = Get-Item -LiteralPath 'main_ral_layered_autonav.pdf'
    Write-Host "Built: $($pdf.FullName) ($($pdf.Length) bytes)"
}
finally {
    Pop-Location
}
