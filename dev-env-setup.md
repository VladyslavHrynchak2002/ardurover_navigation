# Optional: attach VS Code / Cursor to the container

You can do the whole assignment from a terminal. This only helps if you want the editor running inside Docker.

Written for Cursor; VS Code is the same except the Dev Containers extension id is `ms-vscode-remote.remote-containers` (Cursor: `anysphere.remote-containers`).

1. Open this repo and install the recommended extensions (prompt, or from `.vscode/extensions.json`). Also install **Dev Containers**.
2. Start the container: `./docker/build.sh` then `./docker/run.sh`.
3. `Ctrl+Shift+P` → **Dev Containers: Attach to Running Container** → `ardurover-navigation`.
4. `Ctrl+Shift+P` → **Dev Containers: Open Attached Container Configuration File**. Replace the contents with [`.devcontainer/attach-template.json`](.devcontainer/attach-template.json).

   If that command fails in Cursor, copy the template to:

   `~/.config/Cursor/User/globalStorage/anysphere.remote-containers/nameConfigs/ardurover-navigation.json`

   VS Code: `~/.config/Code/User/globalStorage/ms-vscode-remote.remote-containers/nameConfigs/ardurover-navigation.json`

5. Close the attached window, attach again. The workspace should be `/home/developer/ardurover_navigation`.

Use a separate terminal (`./docker/attach.sh`) to run Gazebo / ROS.
