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
- package metadata 提交：公开重写身份
  `08fd6e3 chore(repo): complete package metadata`；原始 bundle 身份为
  `dbaefad`。
- Work Item 关闭提交：公开重写身份
  `a9ac21f docs(work-item): close engineering baseline`；原始 bundle 身份为
  `bd93dff`。
- GitHub 公开仓库已选择，maintainer 为 `Edddddddddy <983166955@qq.com>`。

v0.1 按批准方案删除了所有可达历史中的 `build/install/log`，所以 Git
提交身份发生变化。VN-0007 保存完整 old→new 映射，外部恢复 bundle
保留原始对象；上面的“公开重写身份”才是清理后仓库使用的稳定证据。

## 复盘

- `.gitignore` 只阻止未来未跟踪文件进入 index，不能自动移除已经跟踪的生成物。
- `git rm --cached` 只更新 index，保留本地构建目录，适合修正仓库边界。
- Work Item 定义一次变更的目标和验收；Commit 是一个可评审的修改理由；Release 是经过质量门禁后的不可变交付物。

后续所有课程都必须留下 Work Item、测试证据、文档影响和可审查提交。
