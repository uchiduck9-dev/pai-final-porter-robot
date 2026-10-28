# 最終課題用コンテナ（初回のみ作成。2回目以降は docker start pai_final）
# デスクトップ: http://localhost:6081  /  共有フォルダ: このフォルダ -> /workspace
docker run -d --name pai_final `
  --gpus all `
  -p 6081:80 --shm-size=2g `
  -v "${PSScriptRoot}:/workspace" `
  airobotbook/ros2-desktop-ai-robot-book-humble:latest
