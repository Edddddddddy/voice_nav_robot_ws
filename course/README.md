# VoiceNav Robot 课程

`course/catalog.toml` 是课程索引的唯一数据源：

- `lessons/`：学习者逐步复现的课程契约；
- `records/`：真实完成证据与复盘；
- `reference/`：跨课程复用的技术速查。

重复出现的故障模式统一登记在
[工程踩坑速查](reference/engineering-pitfalls.md)；一次事故的完整证据仍留在
对应 Work Item，不在课件之间复制日志。

源码只有一份，`main` 始终保存参考 solution。新课程从 `course/NNNN-start` annotated tag 创建独立 `learn/NNNN` 分支或 worktree，完成后与 `course/NNNN-solution` tag 比较；不要在课程目录复制一套源码。

不可变 solution tag 只代表发布当时的 reviewed snapshot，不能吸收后续
errata。Lesson 0009 的 `course/0009-solution` 不包含后来交付并分别完成治理
闭环的 C1/C2 修正；原课复现仍与该 tag 比较，修正版累计对照点则等待
`course/0010-start` 正式发布。发布前只使用矫正 Work Item 中记录的精确提交，
不能把移动的 `main` 当成不可变答案。参见
[PIT-0023](reference/engineering-pitfalls.md#pit-0023-an-immutable-course-solution-tag-cannot-absorb-later-errata)。

每课至少包含：单一成果、范围外事项、最小纵向切片、故障定位、自动验收、提交证据和复盘问题。Lesson 0001–0006 是迁移后的历史课程；从 Lesson 0007 开始严格执行 start/solution tag 双轨。

当前 Lesson 0001–0009 为 `completed`。Lesson 0009
[独立 MotionGate](lessons/0009-build-independent-motion-gate.md) 对应已关闭的
VN-0010 / Issue #11；[学习记录](records/0009-independent-motion-gate.md)
保存 exact-head 本地门禁、required CI、独立评审、rebase identity map 与
`course/0009-solution` 不可变身份。学习者复现仍为 Pending。Lesson 0010
验收独立 test authority/candidate 的进程 crash-stop、controller consumer
deadman 与 Gazebo managed/unmanaged pause，不把这些能力倒写为 Lesson 0009
已经完成。

本课程中的“停止”均指仿真项目的高优先级 operational stop，不是经过功能安全认证的急停。
