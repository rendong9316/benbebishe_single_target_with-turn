# ADS-B 数据文件格式说明

## 文件规范

- **格式**：CSV（逗号分隔），无表头行
- **编码**：UTF-8
- **文件名**：`YYYY-MM-DD HH-MM-SS.csv`，表示数据采集起始时间
- **每行**：一条 ADS-B 记录，19 个字段

## 字段定义

| 列序 | 字段 | 示例 | 说明 |
|------|------|------|------|
| 1 | ICAO Address | `88510D` | 航迹批号，24-bit 十六进制，唯一标识目标 |
| 2 | Latitude | `32.62` | 纬度（°） |
| 3 | Longitude | `126.04` | 经度（°） |
| 4 | Heading | `199` | 航向（°） |
| 5 | Altitude | `43000` | 高度（ft） |
| 6 | Ground Speed | `464` | 地速（kt） |
| 7 | _(保留)_ | | 空字段 |
| 8 | Receiver | `F-BDWY1` | 接收站 ID |
| 9 | Aircraft Type | `A359` | ICAO 机型代码 |
| 10 | Registration | `HS-THM` | 飞机注册号 |
| 11 | Timestamp | `2026-04-27 09:29:58` | 时间戳（UTC） |
| 12 | Origin | `ICN` | 起飞机场 IATA 代码 |
| 13 | Destination | `BKK` | 目的机场 IATA 代码 |
| 14 | Flight No. | `TG659` | IATA 航班号 |
| 15 | Flag | `0` | 标志位 |
| 16 | Vertical Rate | `320` | 垂直速率（ft/min） |
| 17 | ICAO Flight No. | `THA659` | ICAO 航班号 |
| 18 | Flag | `0` | 标志位 |
| 19 | Airline | `THA` | 航司 ICAO 代码 |

## 示例数据

```csv
88510D,32.62,126.04,199,43000,464,,F-BDWY1,A359,HS-THM,2026-04-27 09:29:58,ICN,BKK,TG659,0,320,THA659,0,THA
899126,27.67,125.37,50,41000,528,,F-BDWY1,A35K,B-58551,2026-04-27 09:29:59,TPE,NRT,JX800,0,0,SJX800,0,SJX
```

## 关键字段说明

- **ICAO Address**（列 1）：作为航迹批号使用，同一架飞机在不同时间点的记录通过此字段聚合为一条航迹
- **Latitude / Longitude**（列 2-3）：WGS-84 坐标系
- **Ground Speed**（列 6）：地速，单位为节（knots），1 kt ≈ 1.852 km/h
- **Vertical Rate**（列 16）：正值为爬升，负值为下降，单位 ft/min
