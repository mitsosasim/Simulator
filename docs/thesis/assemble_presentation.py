#!/usr/bin/env python3
"""Assemble the supplied 25 JPEG slides into a PDF without recompressing them.

Requires ReportLab. The original thesis report and slide archive are not modified.
"""

import argparse
from io import BytesIO
from pathlib import Path
from zipfile import ZipFile

from reportlab.lib.utils import ImageReader
from reportlab.pdfgen.canvas import Canvas


def main():
    directory = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=directory / "presentation.pdf")
    args = parser.parse_args()
    archive = directory / "presentation-slides.zip"
    output = args.output.resolve()
    if output in {archive, directory / "report.pdf"}:
        parser.error("The output must not overwrite an original document.")

    with ZipFile(archive) as slides:
        expected = {f"{number}.jpg" for number in range(1, 26)}
        if set(slides.namelist()) != expected or len(slides.infolist()) != 25:
            raise ValueError("Expected exactly the original 25 numbered JPEG slides.")
        images = []
        for number in range(1, 26):
            entry = slides.getinfo(f"{number}.jpg")
            if entry.file_size > 4_000_000:
                raise ValueError(f"Unexpected slide size: {entry.filename}")
            reader = ImageReader(BytesIO(slides.read(entry)))
            if reader.getSize() != (1920, 1080):
                raise ValueError(f"Unexpected dimensions: {entry.filename}")
            images.append(reader)

    canvas = Canvas(str(output), pagesize=(960, 540), pageCompression=1, invariant=1)
    canvas.setTitle("Bachelor thesis presentation - supplied slide images")
    canvas.setAuthor("Dimitrios Asimakopoulos")
    canvas.setSubject("25 original JPEG slides assembled into a PDF")
    canvas.setCreator("assemble_presentation.py using ReportLab")
    for number, reader in enumerate(images, start=1):
        canvas.bookmarkPage(f"slide-{number}")
        canvas.addOutlineEntry(f"Slide {number}", f"slide-{number}", level=0)
        canvas.drawImage(reader, 0, 0, width=960, height=540)
        canvas.showPage()
    canvas.save()
    print(f"Created {output} with {len(images)} slides.")


if __name__ == "__main__":
    main()
