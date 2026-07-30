# 建立可审查的工程基线

Lesson 0006 · 目标时间 40–60 分钟

**本课唯一成果：** 把当前工作区整理成一个只跟踪源码、文档和配置，能用统一命令验证，并具有清晰提交历史的 Git 仓库。

本课暂不添加 ROS bridge。工程基线一旦建立，后续每一课都按 Work Item、短分支、测试、文档、评审和提交的同一流程推进。先阅读 [工程变更生命周期](../../docs/process/change-lifecycle.md) 和仓库的 [CONTRIBUTING.md](../../CONTRIBUTING.md)。

> 历史/复盘课程说明：本课记录的是
> `chore/0006-engineering-baseline` 分支当时完成的净化过程，不再提供
> 可重放的 start tag。v0.1 随后按批准方案清除了全部可达历史中的
> `build/install/log`，因此包含 1139 个生成物的精确旧树只存在于仓库外
> 的校验 bundle。不要在当前 checkout 重新执行本课的破坏性 index
> 清理命令。Lesson 0001–0006 是明确的历史例外；从 Lesson 0007 开始才
> 严格提供 `course/NNNN-start` 与 `course/NNNN-solution`。

## 1. 理解当前问题

审计时仓库共有 1178 个已跟踪文件，其中：

| 目录 | 已跟踪文件 | 性质 |
| --- | --- | --- |
| build/ | 780 | 本机构建中间产物 |
| install/ | 255 | 本机安装空间 |
| log/ | 104 | 本机运行与测试日志 |

 这 1139 个文件约占全部已跟踪文件的 96.7%，包含本机绝对路径和 WSL 符号链接。它们会淹没源码 diff，而且让 Windows Git 无法稳定处理状态。

## 2. 确认本课分支与治理文件

教师已经创建分支和治理基线。先检查，不要直接提交：

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws

git branch --show-current
git status --short -- . ':!build' ':!install' ':!log'

sed -n '1,240p' README.md
sed -n '1,280p' CONTRIBUTING.md
sed -n '1,260p' docs/architecture.md
sed -n '1,260p' docs/quality-policy.md
sed -n '1,240p' docs/testing-strategy.md
```

当前分支必须是 `chore/0006-engineering-baseline`。如果文档描述的实际状态、依赖方向或规则与你理解不一致，先问教师，不要为了完成清单而接受错误文档。

上面是当时 checkout 中真实存在的路径。v0.1 迁移后的当前路径对照为：

| Lesson 0006 历史路径 | 当前路径 |
| --- | --- |
| `docs/architecture.md` | `docs/architecture/overview.md` |
| `docs/quality-policy.md` | `docs/process/quality-policy.md` |
| `docs/testing-strategy.md` | `docs/process/testing-strategy.md` |

复盘当前仓库时读右列；恢复 bundle 中的 Lesson 0006 checkpoint 时读左列，
不要把两套路径拼成一个从未存在过的 checkout。

## 3. 只停止跟踪生成物，不删除本地文件

操作前先取得基线：

```
git ls-files build install log | wc -l

test -d build   && echo "build exists"
test -d install && echo "install exists"
test -d log     && echo "log exists"
```

在原始历史 checkpoint 中，第一条预期为 `1139`。以下命令只用于理解
当时的操作，**不得在当前净化后的 checkout 重跑**：

```
git rm -r -f --cached --ignore-unmatch -- \
  build install log
```

**关键区别：** `--cached` 只从 Git index 移除，工作区文件仍保留。
不要运行省略 `--cached` 的命令。当前仓库已经完成清理，学习者只需
审计 `git ls-files build install log` 没有输出。

立即验证：

```
git ls-files build install log | wc -l

test -d build   && echo "build still exists"
test -d install && echo "install still exists"
test -d log     && echo "log still exists"

git check-ignore -v build install log
```

已跟踪数量必须变为 `0`，三个目录仍须存在，`git check-ignore` 应指出根目录 `.gitignore` 的规则。

## 4. 运行统一质量门禁

仓库新增的 `scripts/verify.sh` 是唯一完整入口：它检查 rosdep、Xacro、URDF、SDF 语义契约、全量构建和全部测试。

```
bash scripts/verify.sh
```

 最后必须出现 `VoiceNav Robot verification passed.`， 且 `colcon test-result` 为零错误、零失败。 后续开发中的快速循环可以只传改变的 package：

```
bash scripts/verify.sh voice_nav_sim
```

## 5. 按修改理由拆分提交

 不使用 `git add .`。每次暂存后都先阅读总体 `git diff --cached --stat`。对于大量生成物，只核对删除状态与数量； 对源码、配置和文档则阅读完整 staged diff。

**提交 1：仓库边界与统一门禁**

```
git add \
  .gitignore \
  .gitattributes \
        .editorconfig \
        scripts/verify.sh \
        tools/schema

git diff --cached --stat
git diff --cached --name-status -- build install log \
  | awk '{count[$1]++} END {for (status in count) print status, count[status]}'
git diff --cached -- . ':!build' ':!install' ':!log'

git commit -m "chore(repo): establish clean workspace boundaries"
```

这一提交也包含前面由 `git rm --cached` 暂存的生成物删除。`awk` 汇总应只有 `D 1139`；随后一条命令完整审阅非生成物变更。生成物从历史的当前快照中消失，但本地目录没有被删。

**提交 2：机器人仿真实现**

```
git add src/voice_nav_sim
git diff --cached --stat
git diff --cached
git commit -m "feat(sim): add physical differential-drive robot"
```

**提交 3：课程与验证记录**

```
git add \
  assets \
  learning-records \
  lessons \
  reference \
  NOTES.md \
  RESOURCES.md

git diff --cached --stat
git diff --cached
git commit -m "docs(course): record robot modeling and motion lessons"
```

**提交 4：治理、架构与质量策略**

```
git add \
  README.md \
  CONTRIBUTING.md \
  CHANGELOG.md \
  CONTEXT.md \
  LICENSE \
  THIRD_PARTY_NOTICES.md \
  docs

git diff --cached --stat
git diff --cached
git commit -m "docs(repo): establish engineering governance baseline"
```

## 6. 验收仓库状态

```
git status --short
git ls-files build install log | wc -l
git log --oneline --decorate -6
bash scripts/verify.sh
```

预期：工作区干净；生成物已跟踪数量为 0；日志能够看出四个不同修改理由；完整质量门禁通过。若 `git status` 仍有文件，先判断它属于遗漏的源码、文档还是不应提交的本地数据，不要直接全部暂存。

## 7. 本课之后的外部操作

Lesson 0006 当时不执行远程写和历史重写，因为它们会改变协作边界。学习者随后明确选择了 GitHub 公开仓库，配置 maintainer 为 `Edddddddddy <983166955@qq.com>`，并在后续 v0.1 Work Item 中单独完成 CI、分支保护与历史净化。原始身份、重写身份和恢复 bundle 的映射记录在
[VN-0007](../../docs/work-items/0007-v01-foundation.md)。不要把这些高影响操作塞进清理生成物的同一个提交，也不要把本课当成当前仓库的清理脚本执行。

**验收：** 源码仓库不再跟踪 colcon 生成物；本地生成物未被删除；分支与四个提交职责清晰；统一门禁通过；能够解释 Work Item、Commit、Release 和 ADR 的不同职责。

## 提交给教师

1. 清理前后的两份 `git ls-files build install log | wc -l`。
1. 证明三个本地目录仍存在的输出。
1. `git status --short`。
1. `git log --oneline --decorate -6`。
1. `bash scripts/verify.sh` 的最终摘要。
1. 回答三个问题：为什么只添加 `.gitignore` 还不够；为什么这次必须使用 `--cached`；Work Item、Commit 和 Release 分别解决什么问题。
1. 选择远程仓库：GitHub 私有、GitHub 公开、Gitee 私有或暂不创建。
1. 给出准备写入 `package.xml` 的 maintainer 名称与邮箱，或明确暂时保留为待办；邮箱可使用专门的开发地址。

## 主要资料

阅读 [ROS REP-2004 Package Quality Categories](https://reps.openrobotics.org/rep-2004/)、[gitignore 文档](https://git-scm.com/docs/gitignore)、[git rm 文档](https://git-scm.com/docs/git-rm)、[Conventional Commits 1.0.0](https://www.conventionalcommits.org/en/v1.0.0/) 和 [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html)。

任一步对 Git index、staged diff 或命令效果不清楚时，先问教师再继续。
