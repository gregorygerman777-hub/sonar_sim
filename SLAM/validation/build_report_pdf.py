"""One page PDF summary for e-mail: headline, metrics table, key figures.

    python SLAM/validation/build_report_pdf.py

Text comes from report_onepage.json (written by hand after reading the results); numbers come from
results/summary.csv; figures from figures/. Output: REPORT_onepage.pdf.
"""

import csv
import json
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import Image, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

HERE = Path(__file__).resolve().parent


def fmt(v, spec):
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return "n/a"


def main():
    cfg = json.loads((HERE / "report_onepage.json").read_text())
    rows = list(csv.DictReader(open(HERE / "results" / "summary.csv")))
    title = ParagraphStyle("t", fontName="Helvetica-Bold", fontSize=13, leading=16, spaceAfter=2)
    sub = ParagraphStyle("s", fontName="Helvetica", fontSize=8, leading=10, textColor=colors.HexColor("#4b5563"))
    body = ParagraphStyle("b", fontName="Helvetica", fontSize=8.2, leading=10.4, spaceAfter=3)
    cap = ParagraphStyle("c", fontName="Helvetica", fontSize=7, leading=8.5, textColor=colors.HexColor("#374151"))
    story = [Paragraph(cfg["title"], title), Paragraph(cfg["subtitle"], sub), Spacer(1, 5)]
    for para in cfg["headline"]:
        story.append(Paragraph(para, body))

    header = ["Dataset", "Method", "Frames posed", "ATE RMSE", "ATE / path", "Map error (median)"]
    data = [header]
    for r in rows:
        if r["dataset"] not in cfg["table_datasets"]:
            continue
        ate = f"{fmt(r['ate_rmse_m'], '.3f')} m" if r["ate_rmse_m"] else "no ground truth"
        pct = f"{fmt(r['ate_pct'], '.2f')} %" if r["ate_pct"] else ""
        mp = f"{fmt(r['map_surface_median_m'], '.3f')} m" if r["map_surface_median_m"] else ""
        data.append([cfg["dataset_labels"].get(r["dataset"], r["dataset"]), r["method"],
                     f"{r['posed']}/{r['frames']}", ate, pct, mp])
    table = Table(data, colWidths=[1.55 * inch, 1.05 * inch, 0.85 * inch, 1.05 * inch, 0.75 * inch, 1.1 * inch],
                  repeatRows=1)
    table.setStyle(TableStyle([
        ("FONT", (0, 0), (-1, 0), "Helvetica-Bold", 7.2), ("FONT", (0, 1), (-1, -1), "Helvetica", 7.2),
        ("LINEBELOW", (0, 0), (-1, 0), 0.6, colors.black), ("LINEBELOW", (0, -1), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f3f4f6")]),
        ("TOPPADDING", (0, 0), (-1, -1), 1.2), ("BOTTOMPADDING", (0, 0), (-1, -1), 1.2)]))
    story += [Spacer(1, 3), table, Spacer(1, 5)]

    cells, caps = [], []
    for fig in cfg["figures"]:
        path = HERE / fig["path"]
        img = Image(str(path), width=3.6 * inch, height=3.6 * inch * fig.get("aspect", 0.62))
        cells.append(img)
        caps.append(Paragraph(fig["caption"], cap))
    grid = []
    for i in range(0, len(cells), 2):
        grid.append(cells[i:i + 2])
        grid.append(caps[i:i + 2])
    g = Table(grid, colWidths=[3.7 * inch, 3.7 * inch])
    g.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 1),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 1), ("TOPPADDING", (0, 0), (-1, -1), 0),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
    story.append(g)
    if cfg.get("footer"):
        story += [Spacer(1, 3), Paragraph(cfg["footer"], sub)]
    doc = SimpleDocTemplate(str(HERE / "REPORT_onepage.pdf"), pagesize=letter, leftMargin=0.45 * inch,
                            rightMargin=0.45 * inch, topMargin=0.4 * inch, bottomMargin=0.35 * inch,
                            title=cfg["title"], author=cfg.get("author", ""))
    doc.build(story)
    print("wrote", HERE / "REPORT_onepage.pdf")


if __name__ == "__main__":
    main()
