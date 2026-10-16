#!/usr/bin/env bash
# Generate a self-signed certificate for the XR bridge.
#
# WebXR only runs in a secure context, so the page MUST be served over HTTPS
# even on a LAN. The Quest browser will warn once about the self-signed cert;
# accept it and the session works from then on.
#
# The IP goes in subjectAltName, not just the common name: modern browsers
# ignore CN entirely and will refuse the cert without a matching SAN.
#
#   ./make_cert.sh 192.168.1.50 [output_dir]
set -euo pipefail

IP="${1:?usage: make_cert.sh <ip-address> [output_dir]}"
OUT="${2:-$HOME/.config/moveit_teleop}"
mkdir -p "$OUT"

openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
  -keyout "$OUT/xr_key.pem" -out "$OUT/xr_cert.pem" \
  -subj "/CN=$IP" \
  -addext "subjectAltName=IP:$IP,IP:127.0.0.1,DNS:localhost" 2>/dev/null

chmod 600 "$OUT/xr_key.pem"
echo "wrote $OUT/xr_cert.pem"
echo "wrote $OUT/xr_key.pem"
echo
echo "On the Quest, open:  https://$IP:8443"
echo "Accept the certificate warning once, then press Enter VR."
