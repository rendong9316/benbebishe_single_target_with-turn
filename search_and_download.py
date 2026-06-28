"""Search Google Scholar for maneuvering target tracking papers and download PDFs."""
import sys
import json
import os
import io

# Fix encoding for Windows terminal
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, r"D:\Desktop\MCP-tools\mcp-tools\google-scholar")
sys.path.insert(0, r"D:\Desktop\MCP-tools\mcp-tools\paper-downloader")

from google_scholar_web_search import google_scholar_search

# Define search queries - focused on the core problem
QUERIES = [
    # Core: UKF + maneuvering + adaptive
    "UKF unscented Kalman filter maneuvering target adaptive process noise covariance",
    # IMM for coordinated turns
    "interacting multiple model coordinated turn turn rate estimation maneuvering",
    # Strong tracking filters
    "strong tracking Kalman filter fading factor maneuvering target",
    # Multiple model approaches
    "variable structure multiple model maneuvering target tracking survey",
    # Current statistical / jerk models
    "current statistical model jerk model maneuvering target tracking",
    # OTH radar tracking
    "over-the-horizon radar target tracking Kalman filter maneuvering",
    # Turn rate augmentation
    "coordinated turn model state augmentation turn rate unscented Kalman",
]

OUTPUT_DIR = r"D:\Desktop\single_target_with-turn\papers"
os.makedirs(OUTPUT_DIR, exist_ok=True)

all_results = []

for query in QUERIES:
    print(f"\n{'='*80}")
    print(f"Searching: {query}")
    print(f"{'='*80}")
    try:
        results = google_scholar_search(query, num_results=5)
        for r in results:
            title = r.get('Title', 'N/A')
            authors = r.get('Authors', 'N/A')[:150] if r.get('Authors') else 'N/A'
            url = r.get('URL', 'N/A')[:120] if r.get('URL') else 'N/A'
            # Clean non-ASCII for safe printing
            print(f"\n  Title: {title}")
            print(f"  Authors: {authors}")
            print(f"  URL: {url}")
            all_results.append(r)
    except Exception as e:
        print(f"  ERROR: {e}")

# Save search results
results_file = os.path.join(OUTPUT_DIR, "search_results.json")
seen_titles = set()
unique_results = []
for r in all_results:
    t = r.get('Title', '').lower().strip()
    if t and t not in seen_titles:
        seen_titles.add(t)
        unique_results.append(r)

with open(results_file, 'w', encoding='utf-8') as f:
    json.dump(unique_results, f, ensure_ascii=False, indent=2)

print(f"\n\n{'='*80}")
print(f"Total unique papers found: {len(unique_results)}")
print(f"Results saved to: {results_file}")

# Print top candidates
priority_keywords = [
    "improved ukf", "adaptive unscented", "maneuvering target",
    "coordinated turn", "abrupt", "instantaneous", "strong tracking",
    "current statistical", "variable structure", "IMM", "multiple model",
    "turn rate", "process noise", "fuzzy adaptive", "sigma point",
    "cubature kalman", "robust", "model uncertainty"
]

def relevance_score(paper):
    title = paper.get('Title', '').lower()
    abstract = paper.get('Abstract', '').lower()
    score = 0
    for kw in priority_keywords:
        if kw.lower() in title:
            score += 3
        if kw.lower() in abstract:
            score += 1
    return score

unique_results.sort(key=relevance_score, reverse=True)
print(f"\n{'='*80}")
print("TOP 20 MOST RELEVANT PAPERS:")
print(f"{'='*80}")
for i, r in enumerate(unique_results[:20]):
    print(f"\n{i+1}. [{relevance_score(r)}] {r.get('Title','')}")
    print(f"   URL: {r.get('URL','')[:120]}")
