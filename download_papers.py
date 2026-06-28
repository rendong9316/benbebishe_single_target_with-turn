"""Download the most relevant papers, prioritizing open access sources."""
import sys
import os
import json
import io
import urllib.request
import urllib.error
import ssl
from pathlib import Path

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

OUTPUT_DIR = Path(r"D:\Desktop\single_target_with-turn\papers")

# Read search results
with open(OUTPUT_DIR / "search_results.json", 'r', encoding='utf-8') as f:
    results = json.load(f)

# Top priority papers with their URLs - focused on approaches applicable to instantaneous turns
PRIORITY_PAPERS = [
    # 1. Comprehensive survey of maneuvering target tracking (arXiv - open)
    {
        "title": "A survey of manoeuvring target tracking methods",
        "url": "https://arxiv.org/pdf/1503.07828.pdf",
        "source": "arXiv",
        "note": "综合综述 - 各种机动跟踪方法"
    },
    # 2. EKF/UKF with CT models polar vs Cartesian velocity
    {
        "title": "EKF/UKF maneuvering target tracking using coordinated turn models with polar/Cartesian velocity",
        "url": "https://ieeexplore.ieee.org/document/6916122/",
        "source": "IEEE",
        "note": "CT模型的EKF/UKF实现 - 极坐标vs笛卡尔速度"
    },
    # 3. Adaptive turn rate estimation
    {
        "title": "An adaptive turn rate estimation for tracking a maneuvering target",
        "url": "https://ieeexplore.ieee.org/document/9096293/",
        "source": "IEEE Access (OA)",
        "note": "自适应转弯率估计 - 三帧几何法估计转弯率"
    },
    # 4. Constant speed changing rate + constant turn rate model (MDPI - open)
    {
        "title": "A constant speed changing rate and constant turn rate model for maneuvering target tracking",
        "url": "https://www.mdpi.com/1424-8220/14/3/5239",
        "source": "MDPI Sensors",
        "note": "CSCR+CT模型 - 同时估计速度和转弯率变化"
    },
    # 5. Models and algorithms for CT motion (Wiley)
    {
        "title": "Models and algorithms for tracking target with coordinated turn motion",
        "url": "https://onlinelibrary.wiley.com/doi/abs/10.1155/2014/649276",
        "source": "Hindawi/Wiley",
        "note": "CT运动模型与算法综述 - 5种CT模型对比"
    },
    # 6. Adaptive UKF for target tracking (MDPI - open)
    {
        "title": "Adaptive unscented Kalman filter for target tracking with unknown time-varying noise covariance",
        "url": "https://www.mdpi.com/1424-8220/19/6/1371",
        "source": "MDPI Sensors",
        "note": "自适应UKF - Sage-Husa噪声协方差估计"
    },
    # 7. Multiple-model estimators for sharply maneuvering targets (IEEE)
    {
        "title": "Multiple-model estimators for tracking sharply maneuvering ground targets",
        "url": "https://ieeexplore.ieee.org/document/8255572/",
        "source": "IEEE",
        "note": "IMM对付锐角机动 - Matei, Bar-Shalom, Willett"
    },
    # 8. Strong tracking cubature Kalman filter (ScienceDirect)
    {
        "title": "A novel strong tracking cubature Kalman filter and its application in maneuvering target tracking",
        "url": "https://www.sciencedirect.com/science/article/pii/S1000936119302948",
        "source": "ScienceDirect",
        "note": "强跟踪CKF - 渐消因子自适应调节"
    },
    # 9. Intelligent tracking for aerial maneuvering target based on UKF (MDPI - open)
    {
        "title": "Intelligent tracking method for aerial maneuvering target based on unscented Kalman filter",
        "url": "https://www.mdpi.com/2072-4292/16/17/3301",
        "source": "MDPI Remote Sensing",
        "note": "智能UKF跟踪 - 2024年新论文"
    },
    # 10. Improved IMM based on STSRCKF (IEEE Access - OA)
    {
        "title": "An improved IMM algorithm based on STSRCKF for maneuvering target tracking",
        "url": "https://ieeexplore.ieee.org/document/8698226/",
        "source": "IEEE Access",
        "note": "IMM+强跟踪SRCKF - 自适应TPM"
    },
    # 11. Adaptive strong tracking square-root CKF (IEEE Access - OA)
    {
        "title": "Adaptive strong tracking square-root cubature Kalman filter for maneuvering aircraft tracking",
        "url": "https://ieeexplore.ieee.org/document/8301027/",
        "source": "IEEE Access",
        "note": "自适应强跟踪SRCKF - 机动检测+渐消因子"
    },
    # 12. Variational nonlinear Kalman filtering with unknown process noise covariance (IEEE)
    {
        "title": "Variational nonlinear Kalman filtering with unknown process noise covariance",
        "url": "https://ieeexplore.ieee.org/document/10247583/",
        "source": "IEEE",
        "note": "变分贝叶斯估计未知Q - 2023新方法"
    },
]

def download_direct(url, filepath):
    """Direct HTTP download with browser-like headers."""
    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
        "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8",
    }
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    req = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(req, context=ctx, timeout=60) as resp:
            data = resp.read()
            # Check if it's actually a PDF
            content_type = resp.headers.get('Content-Type', '')
            if 'pdf' in content_type or url.endswith('.pdf'):
                with open(filepath, 'wb') as f:
                    f.write(data)
                return True, f"Downloaded ({len(data)} bytes)"
            elif len(data) > 10000 and data[:4] == b'%PDF':
                with open(filepath, 'wb') as f:
                    f.write(data)
                return True, f"Downloaded PDF ({len(data)} bytes)"
            else:
                return False, f"Not a PDF (Content-Type: {content_type}, size: {len(data)})"
    except Exception as e:
        return False, str(e)

def try_mdpi_pdf(url, filepath):
    """MDPI papers: HTML page → find PDF link → download."""
    # MDPI PDF URL pattern: https://www.mdpi.com/1424-8220/19/6/1371/pdf
    if '/htm' in url:
        pdf_url = url.replace('/htm', '/pdf')
    else:
        pdf_url = url.rstrip('/') + '/pdf'
    return download_direct(pdf_url, filepath)

def try_arxiv_pdf(url, filepath):
    """arXiv: https://arxiv.org/abs/XXXX → https://arxiv.org/pdf/XXXX.pdf"""
    if '/abs/' in url:
        pdf_url = url.replace('/abs/', '/pdf/') + '.pdf'
    elif not url.endswith('.pdf'):
        pdf_url = url + '.pdf' if not url.endswith('/') else url + 'pdf'
    else:
        pdf_url = url
    return download_direct(pdf_url, filepath)

# Download papers
print("=" * 80)
print("DOWNLOADING PRIORITY PAPERS")
print("=" * 80)

downloaded = []
failed = []

for i, paper in enumerate(PRIORITY_PAPERS):
    title = paper['title']
    url = paper['url']
    source = paper['source']
    note = paper['note']

    # Generate safe filename
    safe_title = "".join(c if c.isalnum() or c in ' -_' else '_' for c in title)[:80]
    filename = f"{i+1:02d}_{safe_title}.pdf"
    filepath = OUTPUT_DIR / filename

    print(f"\n[{i+1}/{len(PRIORITY_PAPERS)}] {title[:80]}...")
    print(f"  Source: {source} | {note}")

    success = False
    msg = ""

    # Try strategies based on source
    if 'arxiv' in url.lower():
        success, msg = try_arxiv_pdf(url, filepath)
    elif 'mdpi.com' in url:
        success, msg = try_mdpi_pdf(url, filepath)
    elif url.endswith('.pdf'):
        success, msg = download_direct(url, filepath)
    else:
        success, msg = download_direct(url, filepath)

    if success:
        print(f"  ✓ {msg}")
        downloaded.append(paper)
    else:
        print(f"  ✗ {msg}")
        failed.append((paper, msg))

# Try to download open access versions of failed papers
print(f"\n{'='*80}")
print(f"RESULTS: {len(downloaded)} downloaded, {len(failed)} failed")
print(f"{'='*80}")

# Save download status
status = {
    "downloaded": [{"title": p['title'], "url": p['url'], "note": p['note']} for p in downloaded],
    "failed": [{"title": p['title'], "url": p['url'], "note": p['note'], "error": e} for p, e in failed]
}
with open(OUTPUT_DIR / "download_status.json", 'w', encoding='utf-8') as f:
    json.dump(status, f, ensure_ascii=False, indent=2)

print(f"\nDownload status saved to: {OUTPUT_DIR / 'download_status.json'}")

# List downloaded files
print(f"\n{'='*80}")
print("DOWNLOADED PDFs:")
print(f"{'='*80}")
for pdf in sorted(OUTPUT_DIR.glob("*.pdf")):
    size_kb = pdf.stat().st_size / 1024
    print(f"  {pdf.name} ({size_kb:.1f} KB)")
