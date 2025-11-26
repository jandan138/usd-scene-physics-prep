from .normalize import main

if __name__ == "__main__":
    main()
"""
模块入口 __main__

整体介绍：
- 支持通过模块方式运行规范化导出工具：`python -m specs_normalizer`。
- 该入口仅调用 `normalize.main`，不包含具体逻辑，确保包结构清晰。
"""

# 从同包的 normalize 模块导入主函数，用于作为入口调用
from .normalize import main

# 当以模块运行时，执行主函数
if __name__ == "__main__":
    main()

