# Lesson 0006 学习记录：建立可审查的工程基线

状态：Completed

学习者完成了生成物净化、仓库边界、package 元数据、统一验证入口和工程治理文档，并按修改理由拆分提交。

## 验收证据

- `git status --short` 无输出，工作区干净。
- `git ls-files build install log` 无输出，colcon 生成物不再被跟踪。
- 完整验证摘要：
  - `Summary: 6 packages finished [19.6s]`
  - `Summary: 27 tests, 0 errors, 0 failures, 1 skipped`
  - `VoiceNav Robot verification passed.`
- package metadata 提交：`dbaefad chore(repo): complete package metadata`。
- Work Item 关闭提交：`bd93dff docs(work-item): close engineering baseline`。
- GitHub 公开仓库已选择，maintainer 为 `Edddddddddy <983166955@qq.com>`。

## 复盘

- `.gitignore` 只阻止未来未跟踪文件进入 index，不能自动移除已经跟踪的生成物。
- `git rm --cached` 只更新 index，保留本地构建目录，适合修正仓库边界。
- Work Item 定义一次变更的目标和验收；Commit 是一个可评审的修改理由；Release 是经过质量门禁后的不可变交付物。

后续所有课程都必须留下 Work Item、测试证据、文档影响和可审查提交。
