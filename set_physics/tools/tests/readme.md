# 介绍
animation 是原始的一些物品资产，当前是修复 joint limit range 后的一个版本

home_scenes 是处理前的基础场景，其中物体一部分是 reference 以上 animation 里的

clean_data.py 是对 home_scenes 中的物体进行抽取、重新组织层级并分类的处理脚本，运行后会生成 target 目录（包含 Materials、models、scenes 三个子目录），target/scenes/{MXXXX_usd}/start_result_new.usd 或 target/scenes/{MXXXX_usd}/start_result_fix.usd 即资产分类后的场景

set_physics/preprocess_for_interaction.py 是对上述的资产分类后场景（即 target/scenes/{MXXXX_usd}/start_result_new.usd 或 target/scenes/{MXXXX_usd}/start_result_fix.usd）做进一步绑定处理的脚本，包含设置 collider、rigid body、enable joint 等操作，最终处理后的场景为 target/scenes/{MXXXX_usd}/start_result_dynamic.usd


# 如何使用
1. 复制 /g0433_data/tianshihan/ 目录到个人路径
2. 运行 clean_data.py 脚本，会在当前目录下生成 target 目录（需确保当前 conda 环境中包含 numpy、usd-core 等包）
3. 运行 set_physics/preprocess_for_interaction.py 脚本（需修改脚本中的场景路径为自己的），会在 target/scenes/{MXXXX_usd} 目录下生成 start_result_dynamic.usd 文件
4. 然后可用 isaac sim 打开上述 start_result_dynamic.usd 文件进行仿真测试