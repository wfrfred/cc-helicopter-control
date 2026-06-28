# AGENTS.md

## 工程规范

本项目为基于cc:tweaked/cc:sable的游戏内飞控项目.

1. 代码需要简洁, 可维护.

2. 每次修改需要基于需求重新设计，而不是基于历史架构补丁式修改。不需要保留任何 legacy 层。

3. 命名要简洁、无歧义，且无需额外注释即可表达自身含义，即尽量自文档化。不要引入调用点很少的 helper、变量或包装层来增加理解负担

4. 按照conventional commits规范commit, 并符合原子化commit规范.

## 项目契约

### Flight controller

1. 项目中共有三种坐标语义: world, body, navigation. 后两者为FRD坐标系, 不要混用语义, 不要引入含糊的坐标命名.

2. 模块边界需要清晰. 模块接口不得耦合(如controller的接口形状不应依赖于mode的需求等).

3. terms为统一的debug/ui显示字段, 不得被其它用途依赖.

4. 控制效果有变动时, 需要询问用户并得到明确许可. 禁止静默改动控制律.
