# VoiceNav Robot 课程

`course/catalog.toml` 是课程索引的唯一数据源：

- `lessons/`：学习者逐步复现的课程契约；
- `records/`：真实完成证据与复盘；
- `reference/`：跨课程复用的技术速查。

源码只有一份，`main` 始终保存参考 solution。新课程从 `course/NNNN-start` annotated tag 创建独立 `learn/NNNN` 分支或 worktree，完成后与 `course/NNNN-solution` tag 比较；不要在课程目录复制一套源码。

每课至少包含：单一成果、范围外事项、最小纵向切片、故障定位、自动验收、提交证据和复盘问题。Lesson 0001–0006 是迁移后的历史课程；从 Lesson 0007 开始严格执行 start/solution tag 双轨。

当前 Lesson 0001–0008 为 `completed`。Lesson 0009
[独立 MotionGate](lessons/0009-build-independent-motion-gate.md) 为
`in_progress`，对应 VN-0010 / Issue #11；其
[学习记录](records/0009-independent-motion-gate.md) 已填入教师参考实现的
local-GREEN 证据，学习者复现仍为 Pending；PR、required CI、merge 与 solution
tag 尚未闭环。Lesson 0010 才验收进程 crash-stop 与 Gazebo pause/resume，
不能提前写入 Lesson 0009 完成结论。

本课程中的“停止”均指仿真项目的高优先级 operational stop，不是经过功能安全认证的急停。
