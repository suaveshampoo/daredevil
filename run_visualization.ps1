param(
    [Parameter(Mandatory = $true)]
    [string]$Port
)

$python = "C:\Users\Phanb\AppData\Local\Programs\Python\Python313\python.exe"

& $python ".\visualization.py" --port $Port
