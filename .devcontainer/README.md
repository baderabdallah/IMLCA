This `.devcontainer` configuration allows Visual Studio Code to open the repository inside a container built from `.devcontainer/Dockerfile`.

Key points
- The devcontainer uses the Dockerfile at `.devcontainer/Dockerfile`.
- Workspace is mounted into `/workspace` inside the container.
- Common VS Code extensions for Python, C/C++ and ROS are recommended and will be suggested when you open the folder in a container.

How to use
1. Install the "Dev Containers" extension in VS Code (Remote - Containers).
2. In VS Code: Command Palette -> Remote-Containers: Open Folder in Container... and choose the repository root.

Notes / tips
- All Docker-related files now live under `.devcontainer/` (Dockerfile, docker-compose.yml, ros_entrypoint.sh).
- The devcontainer forwards the ROS default port `11311`. If you use a different ROS master or additional services, adjust the compose file accordingly.
- The container runs as `root` (see `remoteUser` in `devcontainer.json`). Adjust if needed.
