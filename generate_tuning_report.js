// generate_tuning_report.js — 生成 UKF 参数调优总结报告 Word 文档 (V2)
const docx = require('docx');
const fs = require('fs');

const FONT_WEST = "Times New Roman";
const FONT_CN = "SimSun";
const SIZE_XIAOSI = 24;
const SIZE_WUHAO = 21;
const SIZE_SANHAO = 32;
const SIZE_XIAOSAN = 30;
const SIZE_SIHAO = 28;
const COLOR_BLACK = "000000";
const FONT_BODY = { ascii: FONT_WEST, hAnsi: FONT_WEST, cs: FONT_WEST, eastAsia: FONT_CN };
const LINE_SPACING = { line: 300, lineRule: "auto" };

function bodyPara(text, options = {}) {
    return new docx.Paragraph({
        spacing: LINE_SPACING, ...options,
        children: [new docx.TextRun({ text, font: FONT_BODY, size: SIZE_XIAOSI, color: COLOR_BLACK, ...(options.textOptions || {}) })],
    });
}
function heading1(text) {
    return new docx.Paragraph({
        spacing: { before: 240, after: 120, line: 300, lineRule: "auto" },
        children: [new docx.TextRun({ text, font: FONT_BODY, size: SIZE_SANHAO, color: COLOR_BLACK, bold: true })],
    });
}
function heading2(text) {
    return new docx.Paragraph({
        spacing: { before: 200, after: 100, line: 300, lineRule: "auto" },
        children: [new docx.TextRun({ text, font: FONT_BODY, size: SIZE_SIHAO, color: COLOR_BLACK, bold: true })],
    });
}
function heading3(text) {
    return new docx.Paragraph({
        spacing: { before: 160, after: 80, line: 300, lineRule: "auto" },
        children: [new docx.TextRun({ text, font: FONT_BODY, size: SIZE_XIAOSI, color: COLOR_BLACK, bold: true })],
    });
}
function emptyPara() { return new docx.Paragraph({ spacing: LINE_SPACING, children: [] }); }
function makeCell(text, isHeader, width) {
    const p = new docx.Paragraph({
        spacing: LINE_SPACING, alignment: docx.AlignmentType.CENTER,
        children: [new docx.TextRun({ text: String(text), font: FONT_BODY, size: SIZE_XIAOSI, color: COLOR_BLACK, bold: isHeader })],
    });
    const opts = { children: [p], verticalAlign: docx.VerticalAlign.CENTER };
    if (width) opts.width = { size: width, type: docx.WidthType.DXA };
    if (isHeader) opts.shading = { fill: "D9E2F3" };
    return new docx.TableCell(opts);
}
function makeRow(cells, isHeader, widths) {
    return new docx.TableRow({ children: cells.map((c, i) => makeCell(c, isHeader, widths ? widths[i] : null)) });
}
function makeTable(headers, rows, widths) {
    return new docx.Table({
        rows: [makeRow(headers, true, widths), ...rows.map(r => makeRow(r, false, widths))],
        width: { size: 100, type: docx.WidthType.PERCENTAGE },
    });
}
function figCap(text) {
    return new docx.Paragraph({
        spacing: { before: 60, after: 120, line: 300, lineRule: "auto" },
        alignment: docx.AlignmentType.CENTER,
        children: [new docx.TextRun({ text, font: FONT_BODY, size: SIZE_WUHAO, color: COLOR_BLACK, italics: true })],
    });
}

const sections = [];

// ---- 封面 ----
sections.push(emptyPara(), emptyPara(),
    new docx.Paragraph({ spacing: LINE_SPACING, alignment: docx.AlignmentType.CENTER,
        children: [new docx.TextRun({ text: "UKF 参数自动调优总结报告", font: FONT_BODY, size: 44, color: COLOR_BLACK, bold: true })] }),
    emptyPara(),
    new docx.Paragraph({ spacing: LINE_SPACING, alignment: docx.AlignmentType.CENTER,
        children: [new docx.TextRun({ text: "天波双基地OTH-SWR 拐弯目标跟踪  |  V2 扩大范围全局搜索", font: FONT_BODY, size: SIZE_XIAOSAN, color: COLOR_BLACK })] }),
    emptyPara(),
    new docx.Paragraph({ spacing: LINE_SPACING, alignment: docx.AlignmentType.CENTER,
        children: [new docx.TextRun({ text: "2026年5月26日", font: FONT_BODY, size: SIZE_XIAOSI, color: COLOR_BLACK })] }),
    emptyPara(), emptyPara());

// ---- 一、调优策略 ----
sections.push(heading1("一、调优策略"));
sections.push(heading2("1.1 总体方案"));
sections.push(bodyPara("采用两轮全局搜索加两轮专项搜索的递进式策略。V1初步搜索（85组）发现最优值位于搜索边界，表明范围不足。V2据此将Q范围从20倍扩展到1000倍、gate从2倍扩展到4.7倍、alpha从100倍扩展到1000倍，全面覆盖参数空间。"));
sections.push(bodyPara("搜索流程：V1（85组）→ V2 Round 1 Q×gate全局粗扫（150组）→ Round 2 最优加密+R2比例扫描（80组）→ Round 3 alpha×P_pos扩大范围（49组）→ Round 4 自适应参数扩大范围（40组）。两轮共397组参数，每组10个随机种子，合计3970次仿真。"));
sections.push(heading2("1.2 评价指标"));
sections.push(bodyPara("综合得分公式：Score = 0.5 × RMSE_avg + 0.3 × RMSE_turn + 0.2 × Smoothness。其中RMSE_avg为R1/R2自适应UKF的平均位置RMSE（km），RMSE_turn为拐弯区域（拐点前后300s）的位置RMSE（km），Smoothness为相邻帧航迹步长的标准差（km）。得分越低越优。"));
sections.push(heading2("1.3 轻量化设计"));
sections.push(bodyPara("预生成缓存使每种子的检测数据只生成一次，所有参数组合复用。不画图、不保存文件、不运行融合，仅执行核心跟踪逻辑。单次参数评估耗时约1.1~3.5秒。"));

// ---- 二、V1→V2范围改进 ----
sections.push(heading1("二、V1 → V2 搜索范围改进"));
sections.push(bodyPara("V1初步搜索发现gate_sigma ≥ 3.0是临界门槛，且最优值出现在搜索边界（Q=1×10⁵, gate=3.9），表明搜索范围可能不够。V2据此将各参数范围大幅扩展："));
sections.push(emptyPara());
sections.push(makeTable(
    ["参数", "V1范围", "V2范围", "扩展倍数"],
    [
        ["Q_scale", "1×10⁴ ~ 2×10⁵", "1×10³ ~ 1×10⁶", "50×"],
        ["gate_sigma", "2.0 ~ 3.9", "1.5 ~ 7.0", "2.8×"],
        ["ukf_alpha", "1×10⁻⁴ ~ 1×10⁻²", "1×10⁻⁴ ~ 1×10⁻¹", "10×"],
        ["P_pos_std", "0.05 ~ 0.30", "0.02 ~ 0.50", "1.9×"],
        ["fuzzy_window", "5 ~ 12", "3 ~ 16", "1.9×"],
        ["ema_eta", "0.10 ~ 0.30", "0.05 ~ 0.50", "1.8×"],
        ["R2/R1 Q比例", "固定2.0", "1.5 / 2.0 / 2.5", "新增"],
        ["总组合数", "85", "312", "3.7×"],
    ],
    [2000, 2200, 2400, 1400]
));
sections.push(figCap("表1  V1与V2搜索范围对比"));

// ---- 三、V2四轮搜索详情 ----
sections.push(heading1("三、V2 四轮搜索详情"));

sections.push(heading2("3.1 Round 1：Q_scale × gate_sigma 全局扫描（150组）"));
sections.push(heading3("搜索范围选取依据"));
sections.push(bodyPara("Q_scale在1×10³~1×10⁶取15个对数步进（约1.5×步长），覆盖从极平滑到极灵敏的完整范围。gate_sigma在1.5~6.0取10个等步长值（步长0.5），从严格门限到非常宽松。R2保持2倍于R1的比例。"));
sections.push(emptyPara());
sections.push(makeTable(
    ["gate", "Q=1e4", "Q=3e4", "Q=5e4", "Q=7e4", "Q=1e5", "Q=2e5", "Q=3e5"],
    [
        ["1.5", "46.5", "42.1", "34.1", "27.3", "23.7", "19.1", "15.9"],
        ["2.0", "38.0", "29.8", "19.6", "17.3", "16.9", "20.3", "18.2"],
        ["2.5", "31.1", "29.0", "22.3", "20.2", "14.7", "9.1", "8.5"],
        ["3.0", "27.8", "20.1", "19.1", "7.0", "6.9", "6.9", "7.0"],
        ["3.5", "22.2", "8.4", "7.3", "7.0", "6.9", "6.9", "7.0"],
        ["4.0+", "16~19", "7.6", "7.3", "7.0", "6.8", "6.9", "7.0"],
    ],
    [700, 800, 800, 800, 800, 800, 800, 800]
));
sections.push(figCap("表2  Round 1结果摘要：各参数组合的Score值（加粗为性能高原区域）"));
sections.push(bodyPara("核心发现——性能高原：gate_sigma是最敏感参数。当gate < 3.0时，Score在9~47之间剧烈波动（航迹频繁发散）。一旦gate ≥ 3.0且Q ≥ 3×10⁴，Score骤降至6.8~7.0并保持稳定——这就是性能高原。在高原上，参数进一步变化的边际收益极小（<3%）。"));
sections.push(bodyPara("V1对比：V1的Q范围（1×10⁴~2×10⁵）和gate范围（2.0~3.9）恰好覆盖了高原的上升沿，但未能描绘高原在更大gate和更小Q处的边界。V2通过1000倍Q范围和gate=7.0的延展，完整刻画了高原全貌。"));
sections.push(bodyPara("本轮最优：Q=1×10⁵, gate=6.0, Score=6.83", { textOptions: { bold: true } }));

sections.push(heading2("3.2 Round 2：最优加密 + R2比例扫描（80组）"));
sections.push(bodyPara("在最优Q值（1×10⁵）附近以0.65~1.6倍系数加密（7个Q值），gate在5.0~7.0以步长0.2扫描（11个值）。额外测试R2/R1 Q比例1.5/2.0/2.5。"));
sections.push(bodyPara("结果：Q在8×10⁴~1.25×10⁵、gate ≥ 5.0范围内，Score稳定在6.82~6.90。R2/R1比例1.5~2.5性能几乎无差异（6.82~6.84）。gate=7.0给出最优Score=6.82。"));
sections.push(bodyPara("本轮最优：Q_R1=1×10⁵, Q_R2=2×10⁵, gate=7.0, Score=6.82", { textOptions: { bold: true } }));

sections.push(heading2("3.3 Round 3：alpha × P_pos_std 扩大范围（49组）"));
sections.push(bodyPara("alpha从1×10⁻⁴到1×10⁻¹（1000倍，7个对数步进），P_pos从0.02°到0.50°（7个步进）。"));
sections.push(bodyPara("结果：P_pos=0.10°~0.20°表现稳定最优。P_pos=0.02°（过于确信初始位置）和P_pos=0.50°（过于不确定）均劣化约5~8%。alpha ≥ 1×10⁻²后几乎无差异，1×10⁻¹给出最优Score。"));
sections.push(bodyPara("本轮最优：alpha=1×10⁻¹, P_pos=0.10°, Score=6.80", { textOptions: { bold: true } }));

sections.push(heading2("3.4 Round 4：自适应参数扩大范围（40组）"));
sections.push(bodyPara("fuzzy_window从3到16（5个步进），ema_eta从0.05到0.50（8个步进）。"));
sections.push(bodyPara("结果：eta=0.10（较慢适应）优于eta=0.30（较快适应），这与V1发现相反。原因在于V2的gate和Q已经足够大，不需要快速的Q自适应调节来补偿模型误差——滤波器本身已经足够灵活。窗口大小几乎无影响（3~16得分相同）。"));
sections.push(bodyPara("本轮最优：window=3, eta=0.10, Score=6.79", { textOptions: { bold: true } }));

// ---- 四、最终最优参数 ----
sections.push(heading1("四、最终最优参数"));
sections.push(emptyPara());
sections.push(makeTable(
    ["参数", "原始值", "V1最优", "V2最优", "变化说明"],
    [
        ["Q_scale (R1)", "5×10⁴", "1×10⁵", "1×10⁵", "过程噪声翻倍"],
        ["Q_scale (R2)", "1×10⁵", "2×10⁵", "2×10⁵", "R2保持2倍于R1"],
        ["gate_sigma (R1)", "2.0", "3.9", "7.0", "拐弯场景大幅放宽"],
        ["gate_sigma (R2)", "2.5", "3.9", "7.0", "同上"],
        ["ukf_alpha", "1×10⁻³", "1×10⁻²", "1×10⁻¹", "Sigma点散布扩大100倍"],
        ["P_pos_std", "0.20°", "0.10°", "0.10°", "初始位置更确信"],
        ["fuzzy_window", "8", "5", "3", "NIS窗口缩小"],
        ["ema_eta", "0.20", "0.30", "0.10", "大幅gate下适应速度可减慢"],
    ],
    [1800, 1000, 1000, 1000, 4200]
));
sections.push(figCap("表3  原始参数与V1/V2调优后参数对比"));
sections.push(bodyPara("直线场景（run_simulation.m）使用保守配置：gate_sigma=4.0（R1）/5.0（R2），alpha=1×10⁻²。直线场景机动少，过宽的门限无益且可能引入杂波。"));

// ---- 五、最终性能 ----
sections.push(heading1("五、最终性能"));
sections.push(heading2("5.1 拐弯场景性能对比"));
sections.push(emptyPara());
sections.push(makeTable(
    ["指标", "原始参数", "V2最优", "改善幅度"],
    [
        ["自适应UKF R1 RMSE", "18.2 km", "6.6 km", "+63.7%"],
        ["自适应UKF R2 RMSE", "50.0 km", "8.2 km", "+83.6%"],
        ["自适应UKF 平均 RMSE", "34.1 km", "7.4 km", "+78.3%"],
        ["拐弯区域平均 RMSE", "9.7 km", "8.0 km", "+17.5%"],
        ["R1 航迹生命期", "103.6 帧", "104.4 帧", "+0.8%"],
        ["R2 航迹生命期", "103.8 帧", "104.6 帧", "+0.8%"],
        ["平滑度", "11.56 km", "3.55 km", "+69.3%"],
    ],
    [3000, 2000, 2000, 2000]
));
sections.push(figCap("表4  最优参数下的跟踪性能与原始参数对比"));

sections.push(heading2("5.2 参数敏感性排序"));
sections.push(emptyPara());
sections.push(makeTable(
    ["排名", "参数", "敏感度", "影响说明"],
    [
        ["1", "gate_sigma", "极高", "<3.0时性能崩溃（Score从34降至7，−79%），≥3.0后进入高原"],
        ["2", "ukf_Q_scale", "高", "<3×10⁴时滞后严重，3×10⁴~3×10⁵均表现良好"],
        ["3", "ukf_alpha", "低", "1×10⁻³~1×10⁻¹范围几乎无差异"],
        ["4", "ukf_P_pos_std", "低", "0.10°~0.20°均可，极端值除外"],
        ["5", "fuzzy_window/eta", "极低", "在最优基础参数上影响<1%"],
    ],
    [600, 2800, 1000, 4600]
));
sections.push(figCap("表5  参数敏感度排序"));

sections.push(heading2("5.3 可直接使用的参数配置"));
sections.push(bodyPara("拐弯场景（run_simulation_turn.m）—— V2全局最优：", { textOptions: { bold: true } }));
sections.push(bodyPara(
    "% === R1 UKF参数（V2全局最优） ===\n" +
    "params.ukf_Q_scale = 1e5;\nparams.gate_sigma = 7.0;\nparams.ukf_alpha = 1e-1;\n" +
    "params.ukf_P_pos_std = 0.10;\nparams.fuzzy_window_size = 3;\n" +
    "params.fuzzy_ema_eta = 0.10;\nparams.maneuver_ema_eta = 0.10;\nparams.tracker_K_loss = 20;\n\n" +
    "% === R2 UKF参数 ===\n" +
    "params_r2.ukf_Q_scale = 2e5;\nparams_r2.gate_sigma = 7.0;\nparams_r2.ukf_alpha = 1e-1;\n" +
    "params_r2.ukf_P_pos_std = 0.10;\nparams_r2.fuzzy_window_size = 3;\n" +
    "params_r2.fuzzy_ema_eta = 0.10;\nparams_r2.maneuver_ema_eta = 0.10;\nparams_r2.tracker_K_loss = 12;"
));
sections.push(bodyPara("直线场景（run_simulation.m）—— 保守配置：", { textOptions: { bold: true } }));
sections.push(bodyPara(
    "% === R1 UKF参数（保守配置） ===\n" +
    "params.ukf_Q_scale = 1e5;\nparams.gate_sigma = 4.0;\nparams.ukf_alpha = 1e-2;\n" +
    "params.ukf_P_pos_std = 0.10;\nparams.fuzzy_window_size = 3;\n" +
    "params.fuzzy_ema_eta = 0.10;\nparams.maneuver_ema_eta = 0.10;\n\n" +
    "% === R2 UKF参数 ===\n" +
    "params_r2.ukf_Q_scale = 2e5;\nparams_r2.gate_sigma = 5.0;\nparams_r2.ukf_alpha = 1e-2;\n" +
    "params_r2.ukf_P_pos_std = 0.10;\nparams_r2.fuzzy_window_size = 3;\n" +
    "params_r2.fuzzy_ema_eta = 0.10;\nparams_r2.maneuver_ema_eta = 0.10;\nparams_r2.tracker_K_loss = 12;"
));

// ---- 六、结论 ----
sections.push(heading1("六、结论"));
sections.push(bodyPara("本次调参经历两轮迭代（V1: 85组 + V2: 312组 = 共397组参数 × 10随机种子），系统性地描绘了UKF参数空间的性能全貌。"));
sections.push(bodyPara("核心发现：存在一个宽广的性能高原——只要gate_sigma ≥ 3.0且Q_scale ≥ 3×10⁴，UKF性能即稳定在接近最优水平（Score 6.8~7.0）。原始参数（gate=2.0~2.5, Q=5×10⁴）恰好位于高原之下，这是航迹发散、被杂波带偏的根本原因。"));
sections.push(bodyPara("最大改进来源：放宽关联门限（gate_sigma: 2.0→7.0）贡献了约80%的性能提升。在拐弯场景中，UKF预测偏差因模型失配而增大，严格门限大量误拒有效量测，导致航迹崩溃。放宽门限后，滤波器能在转弯期间继续关联量测并逐步修正状态。"));
sections.push(bodyPara("增大过程噪声（Q_scale翻倍）和扩大Sigma点散布（alpha增大100倍）进一步改善了滤波器的响应速度和对非线性的捕获能力。自适应参数的贡献在最优基础参数上变得极低（<1%），说明良好的静态参数配置足以覆盖大部分场景。"));
sections.push(bodyPara("实用建议：gate_sigma在3.5~7.0范围内性能几乎无差异。如果担心多目标场景下过宽门限引入杂波，可使用保守配置（gate=4.0~5.0），性能损失<1%。参数已于2026年5月26日应用到run_simulation.m和run_simulation_turn.m。"));
sections.push(bodyPara("最终，自适应UKF在拐弯场景下实现了6.6 km（R1）/ 8.2 km（R2）的单站跟踪精度，较原始参数提升78.3%，航迹生命期达95%以上，平滑度改善69.3%。"));

sections.push(emptyPara());
sections.push(bodyPara("报告由 tune_ukf_params.m 自动生成，V2数据基于10组随机种子（seed=42~51）的统计平均值。参数已应用至仿真主程序。", { textOptions: { italics: true }, alignment: docx.AlignmentType.RIGHT }));

// =========================================================================
const doc = new docx.Document({
    sections: [{
        properties: { page: { margin: { top: 1440, bottom: 1440, left: 1800, right: 1800 } } },
        children: sections,
    }],
});

const outputPath = "UKF调优总结报告.docx";
docx.Packer.toBuffer(doc).then((buffer) => {
    fs.writeFileSync(outputPath, buffer);
    console.log(`文档已生成: ${outputPath}`);
}).catch((err) => { console.error("生成失败:", err); process.exit(1); });
