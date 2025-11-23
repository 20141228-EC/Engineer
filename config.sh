#!/usr/bin/env bash
# 为指定目录中的已追踪文件设置或取消 git skip-worktree 标记
# git bash终端:
# chmod +x config.sh 加入命令, ./config.sh 执行脚本
# powershell: 
# bash ./config.sh -h 查看命令帮助（必须先在git bash 终端加入命令）

set -e

dirs=(
  "Targets/DM-MC02-keil/MDK-ARM"
  "Targets/DM-MC02/MDK-ARM"
  ".vscode"
)

usage() {
  echo "用法: ./config.sh [--unset]"
  echo "无参数: 默认设置 skip-worktree"
  echo "--unset : 取消这些文件的 skip-worktree 标记"
  echo "-h|--help: 显示帮助"
  echo "执行位置: 仓库根目录"
}

case "${1:-}" in
  --unset) mode=unset ;;
  -h|--help) usage; exit 0 ;;
  "") mode=set ;;
  *) echo "参数错误"; usage; exit 1 ;;
esac

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || { echo "非 git 仓库"; exit 1; }

files=$(git ls-files "${dirs[@]}")
[ -z "$files" ] && { echo "无已追踪文件"; exit 0; }

if [ "$mode" = set ]; then
  echo "设置 skip-worktree..."
  printf '%s\n' "$files" | xargs -I{} git update-index --skip-worktree "{}"
else
  echo "取消 skip-worktree..."
  printf '%s\n' "$files" | xargs -I{} git update-index --no-skip-worktree "{}"
fi

echo "完成"
