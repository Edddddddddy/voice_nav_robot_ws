## Issue 链接

Closes #

## 结果

描述本 PR 交付的可观察行为或仓库能力，以及最终结果。

## 验收映射

将 Issue 的每个验收标准映射到修改文件和证据。

- [ ] AC-001：

## 最终测试摘要

列出聚焦检查和最终完整门禁的精确命令、真实退出状态及简洁结果。完整
`bash scripts/verify.sh` 仅在最终 PR HEAD 运行一次。

```text
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
bash scripts/verify.sh
```

## 接口影响

- [ ] 无 Stable Interface 或 ROS/runtime 接口变化。
- [ ] Stable Interface 影响已在 Issue 和相关文档中说明，并有对应测试。
- [ ] 需要时已链接 ADR。

## 回滚

说明最小安全 revert，以及需要保留的证据。

## 剩余风险

列出剩余的安全、兼容性、数据、隐私、依赖和运维风险。
