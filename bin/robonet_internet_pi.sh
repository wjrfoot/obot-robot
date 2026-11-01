#!/usr/bin/env bash
set -e

ACTION="${1:-}"
LAPTOP_IP="${2:-}"

if [[ "$1" == "start" && -z "$LAPTOP_IP" ]]; then
    LAPTOP_IP=$(getent ahostsv4 ros-24.local | awk 'NR==1 {print $1}')
    if [[ -z "$LAPTOP_IP" ]]; then
        echo "ERROR: Cannot resolve ros-24.local"
        echo "Specify the laptop IP manually:"
        echo "  $0 start 192.168.8.194"
        exit 1
    fi
fi
PI_IF="${PI_IF:-wlan0}"

case "$ACTION" in
    start)
        echo "Using laptop $LAPTOP_IP as Internet gateway"

        sudo ip route replace default via "$LAPTOP_IP" dev "$PI_IF" metric 50
        sudo resolvectl dns "$PI_IF" 8.8.8.8
        sudo resolvectl domain "$PI_IF" '~.'

        echo
        echo "Routes:"
        ip route
        echo
        echo "DNS:"
        resolvectl status "$PI_IF"
        ;;

    stop)
        # Remove the temporary gateway without needing to resolve the laptop.
        # Match only the default route on this interface with metric 50.
        while ip -4 route show default dev "$PI_IF" metric 50 | grep -q '^default '; do
            sudo ip route del default dev "$PI_IF" metric 50
        done
        sudo resolvectl revert "$PI_IF"

        echo "Temporary Internet route removed."
        ;;

    *)
        echo "Usage: $0 {start|stop} [laptop-robonet-IP]"
        exit 1
        ;;
esac
