"""Patch run_mc_turn_180deg_compare.m to support imm_3in1"""
import re

path = r'D:\Desktop\single_target_with-turn\run_mc_turn_180deg_compare.m'
with open(path, 'r', encoding='utf-8', errors='replace') as f:
    content = f.read()

# 1. Add u==4 params block after u==3 block (R1)
content = content.replace(
    "if u == 3\n            params_r1.imm_turn_rate_rad_per_sec = omega;\n        end\n\n        % ===== 创建 UKF 模板 + R1 跟踪 =====",
    "if u == 3\n            params_r1.imm_turn_rate_rad_per_sec = omega;\n        end\n        if u == 4\n            params_r1.imm_turn_rate_rad_per_sec = omega;\n            params_r1.imm_adapt_mode = '3in1';\n        end\n\n        % ===== 创建 UKF 模板 + R1 跟踪 ====="
)

# 2. Add imm_3in1 case after imm case (R1)
content = content.replace(
    "case 'imm'\n                ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...\n                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);\n        end\n\n        [snaps_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, ...",
    "case 'imm'\n                ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...\n                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);\n            case 'imm_3in1'\n                ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, ...\n                    params.radar1_lat, params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);\n                ukf1_tpl.filter_type = 'imm_3in1';\n        end\n\n        [snaps_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, ..."
)

# 3. Same for R2
content = content.replace(
    "if u == 3\n            params_r2.imm_turn_rate_rad_per_sec = omega;\n        end\n\n        % ===== 创建 UKF 模板 + R2 跟踪 =====",
    "if u == 3\n            params_r2.imm_turn_rate_rad_per_sec = omega;\n        end\n        if u == 4\n            params_r2.imm_turn_rate_rad_per_sec = omega;\n            params_r2.imm_adapt_mode = '3in1';\n        end\n\n        % ===== 创建 UKF 模板 + R2 跟踪 ====="
)

content = content.replace(
    "case 'imm'\n                ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...\n                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);\n        end\n\n        [snaps_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, ...",
    "case 'imm'\n                ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...\n                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);\n            case 'imm_3in1'\n                ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, ...\n                    params.radar2_lat, params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);\n                ukf2_tpl.filter_type = 'imm_3in1';\n        end\n\n        [snaps_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, ..."
)

# 4. Update IMM专属 condition
content = content.replace("if u == 3", "if u >= 3")

with open(path, 'w') as f:
    f.write(content)

print("Patched run_mc_turn_180deg_compare.m")
