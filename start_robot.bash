#!/usr/bin/env bash
# LLM generated

show_help() {
    cat <<EOF
Usage: $0 <IP_ADDRESS> [-m <MODE>]

Positional:
  IP_ADDRESS        IP of the SmartBot

Options:
  -m <MODE>         Mode for start.bash [prod|dev] (default: prod)
  -h                Show this help message

Examples:
  $0 192.168.33.10
  $0 192.168.33.10 -m dev
EOF
}

MODE="prod"

while getopts "m:h" opt; do
    case $opt in
        m) MODE="$OPTARG" ;;
        h) show_help; exit 0 ;;
        *) show_help; exit 1 ;;
    esac
done
shift $((OPTIND - 1))

if [[ $# -lt 1 ]]; then
    echo "Error: missing IP address"
    echo
    show_help
    exit 1
fi

IP="$1"
REMOTE_SCRIPT="/home/smartbot/agent_repos/smartbot3_ws/docker/start.bash"
CONTAINER="hardware_software_prod"

ssh -tt "smartbot@${IP}" "
    CONTAINER=hardware_software_prod

    echo 'Checking container status...'
    if docker ps --format '{{.Names}}' | grep -q \$CONTAINER; then
        echo 'Container already running.'
    else
        echo 'Starting container: \${CONTAINER} in mode: ${MODE}'
        bash ${REMOTE_SCRIPT} ${MODE}& >/dev/null 2>&1

        echo -n 'Waiting for container to be running'
        for i in {1..30}; do
            if docker ps --format '{{.Names}}' | grep -q \$CONTAINER; then
                echo
                break
            fi
            echo -n '.'
            sleep 1
        done

        if ! docker ps --format '{{.Names}}' | grep -q \$CONTAINER; then
            echo
            echo 'Container failed to start'
            exit 1
        fi
    fi
"

