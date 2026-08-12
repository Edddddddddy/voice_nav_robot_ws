# 随仓 ROS 包 schema

`package_format3.xsd` 与 `package_common.xsd` 是 REP-149 引用的 schema。它们随仓保存，
使 `ament_xmllint` 在本地或 CI 测试中不依赖 HTTP 响应是否可用或完整。

来源：

- 仓库：`https://github.com/ros-infrastructure/rep`
- 提交：`11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`
- 路径：`xsd/package_format3.xsd`、`xsd/package_common.xsd`
- 许可依据：REP-149 将规范置于公共领域，并将 `package_format3.xsd` 链接为其 schema；
  固定的仓库快照没有单独的仓库级许可证文件。

`catalog.xml` 将 `package.xml` 使用的规范 ROS schema URL 映射到本地副本。更新时须同时
更新两份 XSD，并记录新的来源提交。再分发来源见根目录
[THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md)。
