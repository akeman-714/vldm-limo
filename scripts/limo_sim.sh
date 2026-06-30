#!/usr/bin/env bash
# limo_sim.sh — 管理本机 mentor Limo Gazebo/ROS2 仿真栈 (DGX 同机)。
#
# 为什么有这个脚本:
#   同一台 DGX 上多次 `ros2 launch demo.launch.py` 会叠出多套仿真，它们
#   在同一个 ROS_DOMAIN_ID 上抢 /map、/cmd_vel、/tf，并共享一块 GPU,
#   导致相机掉到 2Hz、Nav2 到达永远不收尾。launch 父进程死后 gz sim 还会
#   变孤儿残留。这个脚本把「单套仿真」「干净拆除(不留孤儿)」各做成一条命令。
#
# 子命令:
#   status  统计在跑的仿真组件 + 探 VLM 服务端口(12181-12186)
#   down    拆掉所有仿真栈(gz sim/RTAB-Map/Nav2/bridge/rviz)，
#           但绝不动 VLFM 的 VLM 服务(vlfm.vlm.*)
#   up      带保护的启动: 已有仿真就拒绝启动，否则起一套干净仿真
#           (用 GUI=false / RVIZ=false 环境变量可改无头)
#
# 注意: 把仿真识别字样放在脚本文件里(而非命令行)，所以本脚本进程的命令行
#       只是 "bash limo_sim.sh down"，pkill/pgrep 不会自匹配把自己打断。

set -uo pipefail

# ---- 识别「仿真栈」进程的字样 (不含 VLM 服务) ----
SIM_PATTERNS=(
  'demo.launch' 'sim.launch' 'nav2.launch' 'slam_3d.launch' 'mission.launch'
  'gz sim' 'ros_gz' 'parameter_bridge'
  'rtabmap' 'icp_odometry' 'robot_state_publisher' 'rviz2'
  'controller_server' 'planner_server' 'bt_navigator' 'behavior_server'
  'velocity_smoother' 'waypoint_follower' 'lifecycle_manager'
  'map_server' 'smoother_server' 'collision_monitor'
)

VLM_PORTS=(12181 12182 12183 12184 12185 12186)
VLM_NAME=( [12181]=GDINO [12182]=ITM [12183]=SAM [12184]=YOLO [12185]=VQA [12186]=AttrVerifier )

count() { local n; n=$(pgrep -fc "$1" 2>/dev/null); echo "${n:-0}"; }
count_gz_server() { local n; n=$(pgrep -af '^gz sim .* -s' 2>/dev/null | wc -l); echo "${n:-0}"; }

cmd_status() {
  echo "== 仿真组件 =="
  printf "  gz sim server : %s\n" "$(count_gz_server)"
  printf "  gz sim gui    : %s\n" "$(count 'gz sim gui')"
  printf "  rtabmap       : %s\n" "$(count rtabmap)"
  printf "  nav2 ctrl     : %s\n" "$(count controller_server)"
  printf "  parameter_brg : %s\n" "$(count parameter_bridge)"
  printf "  rviz2         : %s\n" "$(count rviz2)"
  echo "== VLM 服务 =="
  for p in "${VLM_PORTS[@]}"; do
    if ss -ltn 2>/dev/null | grep -q ":$p[[:space:]]"; then
      printf "  :%s %-12s UP\n"   "$p" "${VLM_NAME[$p]}"
    else
      printf "  :%s %-12s down\n" "$p" "${VLM_NAME[$p]}"
    fi
  done
}

cmd_down() {
  echo ">> 拆除仿真栈 (保留 vlfm.vlm 服务) ..."
  local self=$$ ppid=$PPID
  local -a pids=()
  for pat in "${SIM_PATTERNS[@]}"; do
    while read -r pid; do
      [ -z "$pid" ] && continue
      [ "$pid" = "$self" ] && continue
      [ "$pid" = "$ppid" ] && continue
      # 双保险: 任何含 vlfm.vlm 的进程都跳过(VLM 服务)
      if ps -o args= -p "$pid" 2>/dev/null | grep -q 'vlfm\.vlm'; then continue; fi
      pids+=("$pid")
    done < <(pgrep -f "$pat" 2>/dev/null)
  done

  if [ "${#pids[@]}" -eq 0 ]; then
    echo "   没有仿真进程在跑。"
  else
    local uniq; uniq=$(printf '%s\n' "${pids[@]}" | sort -un | tr '\n' ' ')
    echo "   SIGTERM -> $uniq"
    kill -TERM $uniq 2>/dev/null
    # 不等待(避免 sleep)，直接对存活者补 SIGKILL；已退的 KILL 是无害 no-op
    echo "   SIGKILL -> $uniq"
    kill -KILL $uniq 2>/dev/null
  fi
  ros2 daemon stop >/dev/null 2>&1 || true
  echo ">> 完成。当前状态:"
  cmd_status
}

cmd_up() {
  local running; running=$(count_gz_server)
  if [ "$running" -gt 0 ]; then
    echo "!! 拒绝启动: 已有 $running 个 gz sim server 在跑。"
    echo "   先 'bash $0 down'，或确认你真的要并行(那就先 export ROS_DOMAIN_ID 隔离)。"
    cmd_status
    return 1
  fi
  local env="/home/asong/vlfm/scripts/source_limo_ros_env.sh"
  # shellcheck source=/dev/null
  set +u
  source "$env"
  set -u
  echo ">> 启动一套干净仿真 (mission=false rviz=${RVIZ:-true} gui=${GUI:-true}) ..."
  exec ros2 launch limo_pro_sim demo.launch.py \
    mission:=false rviz:="${RVIZ:-true}" gui:="${GUI:-true}"
}

case "${1:-}" in
  status) cmd_status ;;
  down)   cmd_down ;;
  up)     cmd_up ;;
  *) echo "用法: bash $0 {status|down|up}"; exit 2 ;;
esac
