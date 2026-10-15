# ==============================================================================
# run_tests.ps1  –  DMA Controller Test Runner (PowerShell / Windows)
#
# Run from e:\dma\sim\:
#   cd e:\dma\sim
#   .\run_tests.ps1
#
# Individual test:
#   .\run_tests.ps1 -Test tb_dma_registers
# ==============================================================================

param(
    [string]$Test = ""
)

$IVERILOG = "C:\iverilog\bin\iverilog.exe"
$VVP      = "C:\iverilog\bin\vvp.exe"
$FLAGS    = "-g2005-sv"

$RTL = @(
    "..\rtl\dma_fifo.v",
    "..\rtl\dma_address_generator.v",
    "..\rtl\dma_arbiter.v",
    "..\rtl\dma_interrupt_controller.v",
    "..\rtl\dma_axi_lite_slave.v",
    "..\rtl\dma_burst_engine.v",
    "..\rtl\dma_controller.v",
    "..\rtl\dma_top.v"
)

$MEM_MODEL = "..\tb\axi_memory_model.v"

# Tests and their required source files
$TESTS = [ordered]@{
    "tb_dma_registers"       = @("..\rtl\dma_axi_lite_slave.v")
    "tb_dma_single_transfer" = $RTL + $MEM_MODEL
    "tb_dma_burst_transfer"  = $RTL + $MEM_MODEL
    "tb_dma_errors"          = $RTL + $MEM_MODEL
    "tb_dma_multichannel"    = $RTL + $MEM_MODEL
}

$pass = 0
$fail = 0
$results = @()

function Run-Test {
    param([string]$name, [string[]]$srcs)

    Write-Host ""
    Write-Host ("=" * 60) -ForegroundColor Cyan
    Write-Host " Compiling: $name" -ForegroundColor Cyan
    Write-Host ("=" * 60) -ForegroundColor Cyan

    $tb_file = "..\tb\$name.sv"
    $vvp_out = "$name.vvp"

    $compile_args = @($FLAGS) + $srcs + @($tb_file, "-s", $name, "-o", $vvp_out)

    $proc = Start-Process -FilePath $IVERILOG `
                          -ArgumentList $compile_args `
                          -NoNewWindow -Wait -PassThru `
                          -RedirectStandardError "compile_err.tmp"

    $stderr = Get-Content "compile_err.tmp" -ErrorAction SilentlyContinue
    if ($stderr) { Write-Host $stderr -ForegroundColor Yellow }

    if ($proc.ExitCode -ne 0) {
        Write-Host "COMPILE FAILED ($name)" -ForegroundColor Red
        return "COMPILE_FAIL"
    }

    Write-Host " Running:   $name" -ForegroundColor Cyan
    $output = & $VVP $vvp_out 2>&1
    Write-Host $output

    if ($output -match "TEST PASS") {
        Write-Host ">>> PASS: $name <<<" -ForegroundColor Green
        return "PASS"
    } elseif ($output -match "TEST FAIL") {
        Write-Host ">>> FAIL: $name <<<" -ForegroundColor Red
        return "FAIL"
    } else {
        Write-Host ">>> UNKNOWN: $name (no PASS/FAIL found) <<<" -ForegroundColor Yellow
        return "UNKNOWN"
    }
}

Set-Location $PSScriptRoot

# Select which tests to run
if ($Test -ne "") {
    if ($TESTS.Contains($Test)) {
        $to_run = [ordered]@{ $Test = $TESTS[$Test] }
    } else {
        Write-Host "Unknown test: $Test" -ForegroundColor Red
        Write-Host "Available: $($TESTS.Keys -join ', ')"
        exit 1
    }
} else {
    $to_run = $TESTS
}

foreach ($name in $to_run.Keys) {
    $result = Run-Test -name $name -srcs $to_run[$name]
    $results += [PSCustomObject]@{ Test=$name; Result=$result }
    if ($result -eq "PASS") { $pass++ }
    elseif ($result -eq "FAIL" -or $result -eq "COMPILE_FAIL") { $fail++ }
}

Write-Host ""
Write-Host ("=" * 60) -ForegroundColor Cyan
Write-Host " SUMMARY" -ForegroundColor Cyan
Write-Host ("=" * 60) -ForegroundColor Cyan
foreach ($r in $results) {
    $color = if ($r.Result -eq "PASS") { "Green" } `
             elseif ($r.Result -eq "FAIL" -or $r.Result -eq "COMPILE_FAIL") { "Red" } `
             else { "Yellow" }
    Write-Host ("  {0,-40} [{1}]" -f $r.Test, $r.Result) -ForegroundColor $color
}
Write-Host ""
Write-Host "  Passed: $pass   Failed: $fail   Total: $($results.Count)" -ForegroundColor Cyan
Write-Host ""

if ($fail -gt 0) { exit 1 } else { exit 0 }
