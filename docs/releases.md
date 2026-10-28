# DEB 内测发布

当前自动构建目标是 Ubuntu 22.04 / amd64。源码版本来自 `CMakeLists.txt`；例如 `3.2.0-rc.1` 对应 Git 标签 `v3.2.0-rc.1`、Debian 版本 `3.2.0~rc.1`（确保内测版本低于同号正式版）。

## 本地构建

```bash
git submodule update --init --recursive
sudo apt-get install cmake g++ make dpkg-dev patchelf libboost-all-dev libyaml-cpp-dev libeigen3-dev liborocos-kdl-dev liburdfdom-dev libtinyxml-dev libtinyxml2-dev libarchive-dev libssl-dev
cmake -S . -B build/deb-release -DCMAKE_BUILD_TYPE=Release
cmake --build build/deb-release --parallel 2 --target rocosAppMain http_control_rights_test app_config_test tool_package_test tooling_http_test
ctest --test-dir build/deb-release --output-on-failure -R '^(http_control_rights_test|app_config_test|tool_package_test|tooling_http_test)$'
python3 packaging/build_deb.py
```

使用干净的 Ubuntu 构建环境，不要加载 ROS 的 `LD_LIBRARY_PATH`/`CMAKE_PREFIX_PATH`；打包器拒绝非系统路径的外部动态库。运行依赖由 `dpkg-shlibdeps` 从实际 ELF 推导，私有动态库放入 `/opt/rocos/app/lib`，使用相对 RPATH。`build/packages/` 生成 DEB、`release.json` 和 `SHA256SUMS`。

## 安装与配置

```bash
sha256sum -c SHA256SUMS
sudo apt install ./rocos-app_3.2.0~rc.1_amd64.deb
/opt/rocos/app/bin/rocosAppMain --check_config --config=/opt/rocos/config/app.yaml
```

程序位于 `/opt/rocos/app/bin/rocosAppMain`，沿用 `/opt/rocos/config/app.yaml` 入口。只打包 Git 跟踪的默认配置、模型和工具资源，不收集本地标定文件、日志、上传工具快照。YAML、URDF、XML 注册为 Debian conffile，升级时由 dpkg 保留或提示处理本机修改。实际设备配置仍需部署人员核对。

Viz 单独安装到 `/opt/rocos/web`，与 `app.yaml` 中 `web_root: ../web` 对齐；App 包仅创建该空目录。安装 Viz 后，由 App 的 HTTP 服务提供页面。安装和升级均不自动启动、重启或启用机器人；本次仅完成离线验证。

## 原仓库 Release 与自动汇总

1. 更新版本、完成验证并提交到 `v3`。
2. 在该提交上创建同号标签并推送，例如：

   ```bash
   git tag -a v3.2.0-rc.1 -m 'Internal release 3.2.0-rc.1'
   git push origin v3 v3.2.0-rc.1
   ```

3. `Build and release DEB` 工作流构建、运行离线测试、检查解包配置，并上传安装包、`release.json`、`SHA256SUMS` 到本仓库 Release。带 rc 的标签发布为 **Prerelease**。先上传到草稿，校验 GitHub 返回的 SHA256 后公开。
4. `rocos-sia/rocos-releases` 每小时自动收录完整发布，也可在它的 Actions 页面手动运行同步。

工作流的手动运行只生成 Actions 构建产物，不发布 Release。标签和源码版本必须一致；已发布的版本不覆盖，需要新版本和新标签。发布中心不把组件最新版本自动认定为已验证的兼容套装。
