
from pathlib import Path
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

WS = Path(__file__).resolve().parents[3]
DATA = WS / 'paper' / 'data'
FIGURES = WS / 'paper' / 'output' / 'figures'

COLUMN_W = 3.5
TEXT_W = 7.16


BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
RED = '#e34948'
INK, INK_2, MUTED = '#0b0b0b', '#52514e', '#9a9893'
SERIES = [(BLUE, '-'), (ORANGE, '--'), (AQUA, ':')]


FREE, UNKNOWN, OCCUPIED = '#ffffff', '#d9d8d4', '#1b1b1b'


def apply():
    _patch_pdf_indexed_images()
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'Liberation Serif', 'TeX Gyre Termes',
                       'STIX Two Text', 'STIXGeneral'],
        'mathtext.fontset': 'stix',
        'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8,
        'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8,
        'pdf.fonttype': 42, 'ps.fonttype': 42,
        'axes.linewidth': 0.5, 'axes.edgecolor': INK_2, 'axes.labelcolor': INK,
        'xtick.color': INK_2, 'ytick.color': INK_2,
        'xtick.major.width': 0.5, 'ytick.major.width': 0.5,
        'xtick.major.size': 2.5, 'ytick.major.size': 2.5,
        'axes.grid': False, 'grid.linewidth': 0.4, 'grid.color': '#e6e5e1',
        'legend.frameon': False, 'lines.linewidth': 1.0,
        'savefig.dpi': 600, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.01,
        'figure.dpi': 150,
    })


def panel_label(ax, text, x=0.0, y=1.0, **kw):
    """(a), (b), ... drawn inside the image (no subfigure package needed)."""
    ax.text(x, y, text, transform=ax.transAxes, ha='left', va='bottom', fontsize=8,
            color=INK, **kw)


def grid_rgb(grid):
    """Occupancy grid (-1/0/100) -> RGB image (unknown, free, occupied shades)."""
    import numpy as np
    shades = np.array([_rgb(UNKNOWN), _rgb(FREE), _rgb(OCCUPIED)], dtype=np.uint8)
    return shades[np.where(grid < 0, 0, np.where(grid > 50, 2, 1))]


def _rgb(hex_colour):
    h = hex_colour.lstrip('#')
    return [int(h[i:i + 2], 16) for i in (0, 2, 4)]


def _patch_pdf_indexed_images():
    """
    Write indexed PDF images with 8 bits per sample.

    matplotlib 3.6 stores images with few colours as 1/2/4-bit indexed PNG
    data but omits BitsPerComponent from the PNG predictor parameters, so
    readers decode the rows with the wrong length and the image is sheared.
    Forcing 8-bit PNG output keeps the two consistent.
    """
    from io import BytesIO
    import struct

    from matplotlib.backends import backend_pdf

    def write_png(self, img):
        buffer = BytesIO()
        if img.mode == 'P':
            img.save(buffer, format='png', bits=8)
        else:
            img.save(buffer, format='png')
        buffer.seek(8)
        png_data, bit_depth, palette = b'', None, None
        while True:
            length, kind = struct.unpack(b'!L4s', buffer.read(8))
            if kind in (b'IHDR', b'PLTE', b'IDAT'):
                data = buffer.read(length)
                if kind == b'IHDR':
                    bit_depth = int(data[8])
                elif kind == b'PLTE':
                    palette = data
                else:
                    png_data += data
            elif kind == b'IEND':
                break
            else:
                buffer.seek(length, 1)
            buffer.seek(4, 1)
        return png_data, bit_depth, palette

    backend_pdf.PdfFile._writePng = write_png


def full_figure(fig):
    """Bounding box of the whole figure, so the PDF keeps its designed width."""
    from matplotlib.transforms import Bbox
    w, h = fig.get_size_inches()
    return Bbox.from_bounds(0, 0, w, h)
