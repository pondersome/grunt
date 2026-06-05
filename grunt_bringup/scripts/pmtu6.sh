#!/bin/bash
# Find the largest IPv6 packet that survives the path unfragmented (DF set).
# This is the carrier's effective MTU ceiling — the Thread-B number.
#
#   src/grunt/grunt_bringup/scripts/pmtu6.sh [dest]   (run from the ws root)
#
# Default dest = Cloudflare public v6 (measures barney -> internet over the hotspot).
# To probe a specific ZT peer's underlay, pass that peer's carrier v6 address.
dest="${1:-2606:4700:4700::1111}"
echo "probing IPv6 path MTU to $dest (DF set, no fragmentation allowed)"
found=""
for sz in 1452 1432 1400 1372 1350 1300 1280 1232 1200; do
  if ping6 -M do -s "$sz" -c1 -W2 "$dest" >/dev/null 2>&1; then
    echo "  OK   payload=$sz  => effective MTU ~ $((sz + 48))"
    found=$((sz + 48)); break
  else
    echo "  DROP payload=$sz  (MTU < $((sz + 48)))"
  fi
done
if [ -n "$found" ]; then
  echo "effective IPv6 MTU to $dest: ~${found} bytes"
else
  echo "NOTHING got through even at 1200 — path is badly broken or ICMPv6 fully filtered"
fi
