#!/usr/bin/env bash
# (보스 10/30) the VS Code Claude Code session is 마리코 in ~/orchestra-chat (server started with MARICO_BRIDGE=1):
# exits with the request path as soon as the server leaves a turn for 마리코 (data/bridge/req_<id>.json without its
# res_<id>.txt), or after --max_min minutes.  Run in the background; the session answers in res_<id>.txt and re-arms.
#   bash /home/pan/pan/6D_pose/collab/watch_marico.sh --max_min 25
BR="$HOME/orchestra-chat/data/bridge"
MAX_MIN=25
[ "$1" = "--max_min" ] && MAX_MIN="$2"
end=$(( $(date +%s) + MAX_MIN * 60 ))
while [ "$(date +%s)" -lt "$end" ]; do
  for req in "$BR"/req_*.json; do
    [ -e "$req" ] || continue
    id=$(basename "$req" .json); id=${id#req_}
    if [ ! -e "$BR/res_$id.txt" ]; then
      echo "MARICO turn: $req"
      exit 0
    fi
  done
  sleep 2
done
echo "MARICO timeup ${MAX_MIN}min: no turn"
