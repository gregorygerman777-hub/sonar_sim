"""Short PDF summary for e-mail: page 1 validation against ground truth, page 2 the pool data outputs.

    python SLAM/validation/build_report_pdf.py

Text comes from report_summary.json (written by hand after reading the results); numbers in the table come from
results/summary.csv; figures from figures/. Output: REPORT_summary.pdf.
Each entry of "pages" has a heading, paragraphs, an optional results table ("table": true) and rows of figures;
a row with one figure spans the page width, a row with two splits it.
"""

import csv
import json
import tempfile
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.lib.utils import ImageReader
from reportlab.platypus import Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

HERE = Path(__file__).resolve().parent
PAGE_W = 7.6 * inch

TITLE = ParagraphStyle("t", fontName="Helvetica-Bold", fontSize=13, leading=16, spaceAfter=2)
HEAD = ParagraphStyle("h", fontName="Helvetica-Bold", fontSize=10.5, leading=13, spaceBefore=2, spaceAfter=3)
SUB = ParagraphStyle("s", fontName="Helvetica", fontSize=8, leading=10, textColor=colors.HexColor("#4b5563"))
BODY = ParagraphStyle("b", fontName="Helvetica", fontSize=8.2, leading=10.4, spaceAfter=3)
CAP = ParagraphStyle("c", fontName="Helvetica", fontSize=7, leading=8.5, textColor=colors.HexColor("#374151"))


def fmt(v, spec):
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return "n/a"


def trimmed(path, pad=12):
    """Copy of an image with its uniform white border cropped (3D matplotlib axes leave a wide margin)."""
    from PIL import Image as PILImage, ImageChops
    im = PILImage.open(path).convert("RGB")
    diff = ImageChops.difference(im, PILImage.new("RGB", im.size, (255, 255, 255)))
    box = diff.point(lambda v: 255 if v > 8 else 0).getbbox()
    if box:
        box = (max(box[0] - pad, 0), max(box[1] - pad, 0), min(box[2] + pad, im.width), min(box[3] + pad, im.height))
        im = im.crop(box)
    out = Path(tempfile.gettempdir()) / f"trim_{abs(hash(str(path)))}.png"
    im.save(out)
    return out


def results_table(cfg):
    rows = list(csv.DictReader(open(HERE / "results" / "summary.csv")))
    data = [["Dataset", "Method", "Frames posed", "ATE RMSE", "ATE / path", "Map error (median)"]]
    for r in rows:
        if r["dataset"] not in cfg["table_datasets"] or r["run"] in cfg.get("table_exclude_runs", []):
            continue
        ate = f"{fmt(r['ate_rmse_m'], '.3f')} m" if r["ate_rmse_m"] else "no ground truth"
        pct = f"{fmt(r['ate_pct'], '.2f')} %" if r["ate_pct"] else ""
        mp = f"{fmt(r['map_surface_median_m'], '.3f')} m" if r["map_surface_median_m"] else ""
        method = (r["method"].replace("(sequential)", "(seq.)").replace("(exhaustive)", "(exh.)")
                  .replace(", 1 frame in ", ", 1 in "))
        data.append([cfg["dataset_labels"].get(r["dataset"], r["dataset"]), method, f"{r['posed']}/{r['frames']}",
                     ate, pct, mp])
    table = Table(data, colWidths=[1.6 * inch, 1.45 * inch, 0.85 * inch, 1.0 * inch, 0.75 * inch, 1.1 * inch],
                  repeatRows=1)
    table.setStyle(TableStyle([
        ("FONT", (0, 0), (-1, 0), "Helvetica-Bold", 7), ("FONT", (0, 1), (-1, -1), "Helvetica", 7),
        ("LINEBELOW", (0, 0), (-1, 0), 0.6, colors.black), ("LINEBELOW", (0, -1), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f3f4f6")]),
        ("TOPPADDING", (0, 0), (-1, -1), 0.8), ("BOTTOMPADDING", (0, 0), (-1, -1), 0.8)]))
    return table


def figure(path, width, max_height):
    path = trimmed(HERE / path)
    iw, ih = ImageReader(str(path)).getSize()
    h = min(width * ih / iw, max_height)
    return Image(str(path), width=h * iw / ih, height=h)


def figure_rows(rows):
    story = []
    for row in rows:
        width = PAGE_W / len(row) - 0.1 * inch
        cells = [figure(f["path"], width, f.get("max_height_in", 3.0) * inch) for f in row]
        caps = [Paragraph(f["caption"], CAP) for f in row]
        g = Table([cells, caps], colWidths=[PAGE_W / len(row)] * len(row))
        g.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("ALIGN", (0, 0), (-1, 0), "CENTER"),
                               ("LEFTPADDING", (0, 0), (-1, -1), 2), ("RIGHTPADDING", (0, 0), (-1, -1), 2),
                               ("TOPPADDING", (0, 0), (-1, -1), 0), ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
        story += [g, Spacer(1, 4)]
    return story


def main():
    cfg = json.loads((HERE / "report_summary.json").read_text())
    story = [Paragraph(cfg["title"], TITLE), Paragraph(cfg["subtitle"], SUB), Spacer(1, 5)]
    for k, page in enumerate(cfg["pages"]):
        if k:
            story.append(PageBreak())
        story.append(Paragraph(page["heading"], HEAD))
        for para in page.get("paragraphs", []):
            story.append(Paragraph(para, BODY))
        if page.get("table"):
            story += [Spacer(1, 2), results_table(cfg), Spacer(1, 6)]
        story += figure_rows(page.get("figures", []))
        if page.get("footer"):
            story.append(Paragraph(page["footer"], SUB))
    out = HERE / "REPORT_summary.pdf"
    doc = SimpleDocTemplate(str(out), pagesize=letter, leftMargin=0.45 * inch, rightMargin=0.45 * inch,
                            topMargin=0.4 * inch, bottomMargin=0.35 * inch, title=cfg["title"],
                            author=cfg.get("author", ""))
    doc.build(story)
    print("wrote", out)


if __name__ == "__main__":
    main()
