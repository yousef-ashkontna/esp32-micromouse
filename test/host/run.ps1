$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '..\..')
try {
    New-Item -ItemType Directory -Force .pio/host-tests | Out-Null
    & g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Iinclude test/host/navigation_test.cpp -o .pio/host-tests/navigation_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation test compilation failed.' }
    & ./.pio/host-tests/navigation_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation tests failed.' }
    & g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Iinclude -Itest/host/mocks test/host/firmware_test.cpp -o .pio/host-tests/firmware_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Firmware test compilation failed.' }
    & ./.pio/host-tests/firmware_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Firmware tests failed.' }
} finally { Pop-Location }
