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


def trimmed(path, pad=12):
    """Copy of an image with its uniform white border cropped (3D matplotlib axes leave a wide margin)."""
    import tempfile
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
        if r["dataset"] not in cfg["table_datasets"] or r["run"] in cfg.get("table_exclude_runs", []):
            continue
        ate = f"{fmt(r['ate_rmse_m'], '.3f')} m" if r["ate_rmse_m"] else "no ground truth"
        pct = f"{fmt(r['ate_pct'], '.2f')} %" if r["ate_pct"] else ""
        mp = f"{fmt(r['map_surface_median_m'], '.3f')} m" if r["map_surface_median_m"] else ""
        method = (r["method"].replace("(sequential)", "(seq.)").replace("(exhaustive)", "(exh.)")
                  .replace(", 1 frame in ", ", 1 in "))
        data.append([cfg["dataset_labels"].get(r["dataset"], r["dataset"]), method,
                     f"{r['posed']}/{r['frames']}", ate, pct, mp])
    table = Table(data, colWidths=[1.5 * inch, 1.35 * inch, 0.8 * inch, 1.0 * inch, 0.7 * inch, 1.05 * inch],
                  repeatRows=1)
    table.setStyle(TableStyle([
        ("FONT", (0, 0), (-1, 0), "Helvetica-Bold", 6.8), ("FONT", (0, 1), (-1, -1), "Helvetica", 6.8),
        ("LINEBELOW", (0, 0), (-1, 0), 0.6, colors.black), ("LINEBELOW", (0, -1), (-1, -1), 0.4, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f3f4f6")]),
        ("TOPPADDING", (0, 0), (-1, -1), 0.6), ("BOTTOMPADDING", (0, 0), (-1, -1), 0.6)]))
    story += [Spacer(1, 3), table, Spacer(1, 5)]

    cells, caps = [], []
    for fig in cfg["figures"]:
        path = HERE / fig["path"]
        w = cfg.get("figure_width_in", 3.6) * inch
        path = trimmed(path)
        from reportlab.lib.utils import ImageReader
        iw, ih = ImageReader(str(path)).getSize()
        h = min(w * ih / iw, cfg.get("figure_max_height_in", 2.9) * inch)
        img = Image(str(path), width=h * iw / ih, height=h)
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
