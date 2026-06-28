"""Download papers using paper-downloader with Playwright browser automation."""
import sys
import os
import io
import asyncio
import json
from pathlib import Path

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, r"D:\Desktop\MCP-tools\mcp-tools\paper-downloader")

from sites import arxiv, ieee, mdpi, acm, sciencedirect, wiley

OUTPUT_DIR = Path(r"D:\Desktop\single_target_with-turn\papers")

# Papers to download with their URLs and expected site handler
PAPERS = [
    # Open access / easier downloads first
    {
        "title": "An adaptive turn rate estimation for tracking a maneuvering target",
        "url": "https://ieeexplore.ieee.org/document/9096293/",
        "handler": ieee,
        "note": "自适应转弯率估计 - IEEE Access (OA)"
    },
    {
        "title": "A constant speed changing rate and constant turn rate model for maneuvering target tracking",
        "url": "https://www.mdpi.com/1424-8220/14/3/5239",
        "handler": mdpi,
        "note": "CSCR+CT模型 - MDPI Sensors (OA)"
    },
    {
        "title": "Adaptive unscented Kalman filter for target tracking with unknown time-varying noise covariance",
        "url": "https://www.mdpi.com/1424-8220/19/6/1371",
        "handler": mdpi,
        "note": "自适应UKF - MDPI Sensors (OA)"
    },
    {
        "title": "Intelligent tracking method for aerial maneuvering target based on unscented Kalman filter",
        "url": "https://www.mdpi.com/2072-4292/16/17/3301",
        "handler": mdpi,
        "note": "智能UKF跟踪 - MDPI Remote Sensing 2024 (OA)"
    },
    {
        "title": "Multiple-model estimators for tracking sharply maneuvering ground targets",
        "url": "https://ieeexplore.ieee.org/document/8255572/",
        "handler": ieee,
        "note": "IMM锐角机动 - Matei, Bar-Shalom"
    },
    {
        "title": "EKF/UKF maneuvering target tracking using coordinated turn models with polar/Cartesian velocity",
        "url": "https://ieeexplore.ieee.org/document/6916122/",
        "handler": ieee,
        "note": "CT模型EKF/UKF - Roth, Hendeby"
    },
    {
        "title": "Models and algorithms for tracking target with coordinated turn motion",
        "url": "https://onlinelibrary.wiley.com/doi/10.1155/2014/649276",
        "handler": wiley,
        "note": "CT运动模型综述 - 5种模型对比"
    },
    {
        "title": "An improved IMM algorithm based on STSRCKF for maneuvering target tracking",
        "url": "https://ieeexplore.ieee.org/document/8698226/",
        "handler": ieee,
        "note": "IMM+强跟踪SRCKF - IEEE Access"
    },
    {
        "title": "Variational nonlinear Kalman filtering with unknown process noise covariance",
        "url": "https://ieeexplore.ieee.org/document/10247583/",
        "handler": ieee,
        "note": "变分贝叶斯估计Q - 2023新方法"
    },
]


async def download_one(paper, idx, total):
    title = paper['title']
    url = paper['url']
    handler = paper['handler']
    note = paper['note']

    print(f"\n[{idx}/{total}] {title[:80]}...")
    print(f"  {note}")

    try:
        result = await handler.download(url, str(OUTPUT_DIR))
        print(f"  Result: {result}")
        return True, title, result
    except Exception as e:
        print(f"  ERROR: {e}")
        return False, title, str(e)


async def main():
    print("=" * 80)
    print("DOWNLOADING PAPERS WITH PLAYWRIGHT BROWSER")
    print("=" * 80)
    print(f"Output: {OUTPUT_DIR}")

    # Process papers sequentially to avoid browser conflicts
    downloaded = []
    failed = []

    for i, paper in enumerate(PAPERS):
        success, title, msg = await download_one(paper, i + 1, len(PAPERS))
        if success:
            downloaded.append({"title": title, "result": msg})
        else:
            failed.append({"title": title, "error": msg})
        # Small delay between downloads
        await asyncio.sleep(1)

    print(f"\n{'='*80}")
    print(f"RESULTS: {len(downloaded)} downloaded, {len(failed)} failed")
    print(f"{'='*80}")

    if downloaded:
        print("\nDownloaded:")
        for d in downloaded:
            print(f"  ✓ {d['title'][:80]}")

    if failed:
        print("\nFailed:")
        for f in failed:
            print(f"  ✗ {f['title'][:80]}: {f['error'][:100]}")

    # Find all PDFs
    print(f"\n{'='*80}")
    print("PDFs IN DIRECTORY:")
    print(f"{'='*80}")
    for pdf in sorted(OUTPUT_DIR.glob("*.pdf")):
        size_kb = pdf.stat().st_size / 1024
        print(f"  {pdf.name} ({size_kb:.1f} KB)")


if __name__ == "__main__":
    asyncio.run(main())
