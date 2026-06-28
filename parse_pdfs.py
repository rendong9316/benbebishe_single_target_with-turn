"""Extract text from downloaded PDFs using pdfplumber."""
import sys
import io
import os
from pathlib import Path

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

import pdfplumber

PDF_DIR = Path(r"D:\Desktop\single_target_with-turn\papers")
TEXT_DIR = PDF_DIR / "extracted"
TEXT_DIR.mkdir(exist_ok=True)

pdf_files = sorted(PDF_DIR.glob("*.pdf"))
print(f"Found {len(pdf_files)} PDFs\n")

for pdf_path in pdf_files:
    txt_path = TEXT_DIR / (pdf_path.stem + ".txt")
    print(f"Parsing: {pdf_path.name}")

    try:
        full_text = []
        with pdfplumber.open(pdf_path) as pdf:
            for i, page in enumerate(pdf.pages):
                text = page.extract_text()
                if text:
                    full_text.append(f"\n--- Page {i+1} ---\n{text}")
                else:
                    full_text.append(f"\n--- Page {i+1} (no text extracted) ---\n")

        combined = "\n".join(full_text)
        with open(txt_path, 'w', encoding='utf-8') as f:
            f.write(combined)

        # Print stats
        lines = combined.count('\n')
        chars = len(combined)
        print(f"  → {len(full_text)} pages, {lines} lines, {chars} chars → {txt_path.name}")

    except Exception as e:
        print(f"  ERROR: {e}")

print(f"\nDone. Text files saved to: {TEXT_DIR}")
