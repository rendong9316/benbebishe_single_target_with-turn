// Extract all key metrics from scan_Q_scale results
var data = {
    Q: [500, 1000, 3000, 10000, 30000, 100000, 300000, 1000000, 3000000],
    // Gradual Turn - Fusion RMSE (best UKF per Q)
    gradual_fusion: {
        jichu: [30.1, 29.5, 25.1, 10.8, 6.9, 4.9, 4.2, 4.3, 4.9],
        zishiying: [29.6, 27.9, 17.8, 8.4, 5.8, 4.5, 4.2, 4.5, 5.1],
        imm: [29.8, 28.7, 22.0, 10.1, 6.8, 5.0, 4.2, 4.3, 4.8]
    },
    // Gradual Turn - UKF RMSE R1
    gradual_ukf_r1: {
        jichu: [30.8, 30.7, 30.1, 11.0, 7.0, 5.5, 5.3, 5.9, 6.8],
        zishiying: [30.6, 31.7, 20.7, 8.5, 6.1, 5.3, 5.5, 6.2, 7.2],
        imm: [30.8, 30.8, 27.4, 10.2, 6.9, 5.5, 5.3, 5.8, 6.6]
    },
    // Gradual Turn - UKF RMSE R2
    gradual_ukf_r2: {
        jichu: [31.1, 30.7, 21.1, 13.7, 9.3, 6.5, 5.6, 5.8, 6.6],
        zishiying: [30.6, 25.3, 17.3, 11.5, 7.9, 6.0, 5.6, 6.1, 7.1],
        imm: [31.2, 29.0, 20.0, 13.4, 9.5, 6.7, 5.6, 5.7, 6.5]
    },
    // U-Turn - Fusion RMSE
    uturn_fusion: {
        jichu: [7.3, 7.0, 6.1, 5.3, 4.5, 4.0, 4.0, 4.3, 4.8],
        zishiying: [7.0, 6.4, 5.6, 4.8, 4.2, 3.9, 4.0, 4.5, 5.2],
        imm: [4.3, 4.0, 3.6, 3.2, 3.1, 3.4, 3.7, 4.1, 4.7]
    },
    // U-Turn - UKF RMSE R1
    uturn_ukf_r1: {
        jichu: [8.0, 7.5, 6.6, 5.8, 5.2, 5.0, 5.3, 5.9, 6.8],
        zishiying: [7.5, 6.9, 6.1, 5.4, 5.0, 5.1, 5.5, 6.3, 7.3],
        imm: [7.7, 4.9, 3.9, 3.8, 3.9, 4.6, 5.1, 5.7, 6.6]
    },
    // U-Turn - UKF RMSE R2
    uturn_ukf_r2: {
        jichu: [7.5, 7.4, 6.8, 6.0, 5.5, 5.0, 5.1, 5.7, 6.7],
        zishiying: [7.5, 7.1, 6.3, 5.7, 5.2, 5.0, 5.3, 6.1, 7.2],
        imm: [6.1, 6.0, 5.5, 4.8, 4.4, 4.5, 4.9, 5.5, 6.4]
    },
    // Gradual Turn - Imp fusion vs R1 (%)
    gradual_imp_fus_r1: {
        jichu: [2.0, 3.6, 15.8, 0.8, 1.9, 10.2, 20.7, 26.7, 28.3],
        zishiying: [2.9, 11.5, 9.2, 0.6, 4.1, 15.4, 24.1, 27.8, 28.6],
        imm: [3.2, 6.5, 17.0, 0.5, 1.6, 9.3, 20.2, 26.6, 28.1]
    },
    // U-Turn - Imp fusion vs R1 (%)
    uturn_imp_fus_r1: {
        jichu: [7.5, 6.0, 7.3, 9.6, 12.2, 19.2, 24.3, 27.7, 28.9],
        zishiying: [5.9, 6.4, 8.3, 10.7, 16.0, 22.5, 26.5, 28.6, 29.2],
        imm: [41.8, 13.2, 7.7, 14.4, 21.0, 25.8, 27.9, 28.7, 29.3]
    },
    // Gradual Turn - Best fusion method (imm)
    gradual_best_method: ['BC','BC','CI','CI','CI','BC','BC','BC','BC'],
    // U-Turn - Best fusion method (imm)
    uturn_best_method: ['FCI','CI','FCI','SCC','BC','BC','SCC','BC','BC']
};
console.log(JSON.stringify(data));
