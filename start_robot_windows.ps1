param (
    [Parameter(Mandatory=$true, Position=0)]
    [string]$IP,

    [ValidateSet("prod", "dev")]
    [string]$Mode = "prod"
)

function Show-Help {
    Write-Host "Usage: .\start_robot.ps1 <IP_ADDRESS> [-Mode <prod|dev>]"
    Write-Host ""
    Write-Host "Positional:"
    Write-Host "  IP_ADDRESS        IP of the SmartBot"
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -Mode <prod|dev>  Mode for start.bash (default: prod)"
}

if ($IP -eq "-h" -or $IP -eq "--help") {
    Show-Help
    exit 0
}

$RemoteScript = "/home/smartbot/agent_repos/smartbot3_ws/docker/start.bash"
$Container = "hardware_software_prod"

ssh -t -t "smartbot@$IP" @"
    echo 'Checking container status...'
    if docker ps --format '{{.Names}}' | grep -q $Container; then
        echo 'Container already running.'
    else
        echo 'Starting container: ${Container} in mode: $Mode'
        bash $RemoteScript $Mode >/dev/null 2>&1 &

        echo -n 'Waiting for container to be running'
        for i in {1..30}; do
            if docker ps --format '{{.Names}}' | grep -q $Container; then
                echo
                break
            fi
            echo -n '.'
            sleep 1
        done

        if ! docker ps --format '{{.Names}}' | grep -q $Container; then
            echo
            echo 'Container failed to start'
            exit 1
        fi
    fi
"@
