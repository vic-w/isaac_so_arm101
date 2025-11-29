import os
import shutil

# i= 0 到 11200 step 50
for i in range(40200, 45501, 200):
    cmd = "python scripts/rsl_rl/play.py --task=SO-ARM100-Camera-Lift-Cube-v0 --num_envs=1 --video --video_length=1000 --checkpoint=logs/rsl_rl/lift/2025-11-23_23-14-44/model_%d.pt" % i
    os.system(cmd)

    # 将action_log.txt 复制到新目录并重命名
    new_name = "action_logs/action_log_%d.txt" % i
    os.makedirs("action_logs", exist_ok=True)
    shutil.copy("action_log.txt", new_name)  

