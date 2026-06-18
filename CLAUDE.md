# CodeGraph

本项目已建 CodeGraph 索引。

- 查符号定义/调用链/依赖/影响范围 → 先调 CodeGraph MCP 工具
- ❌ 禁止在未查图谱前对整个 repo 做 Grep/Glob 全量扫描
- ✅ 仅图谱定位到文件+行号后再 Read 具体文件
