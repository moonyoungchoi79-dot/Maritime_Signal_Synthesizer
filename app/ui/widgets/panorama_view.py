"""
Panorama View Widget for Camera Detection Visualization (C-1, C-2, C-3)

Displays a -90° to +90° panorama view showing detected ships as bounding boxes.
Uses debouncing to prevent excessive repaints per spec C-3.
"""
import time
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QFontMetrics


# Panorama constants (matching simulation_worker.py)
PANO_W_PX = 1920
PANO_H_PX = 320


class PanoramaView(QWidget):
    """
    Panorama view widget showing camera detections.
    Displays bounding boxes for detected ships within the -90° to +90° FOV.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.detections = []
        self.setMinimumHeight(100)
        self.setMinimumWidth(400)
        self._last_update = 0
        self._debounce_ms = 50  # Per spec C-3: debounce 30-100ms

        # Colors
        self._bg_color = QColor(30, 30, 40)
        self._grid_color = QColor(60, 60, 80)
        self._bbox_color = QColor(0, 255, 0)
        self._bbox_fill = QColor(0, 255, 0, 40)
        self._text_color = QColor(255, 255, 255)
        self._scale_color = QColor(150, 150, 170)

    def set_detections(self, detections):
        """
        Update detections with debounce (C-3).
        Only updates if enough time has passed since last update.
        """
        now = time.time() * 1000
        if now - self._last_update < self._debounce_ms:
            return
        self._last_update = now
        self.detections = detections
        self.update()

    def clear_detections(self):
        """Clear all detections from view."""
        self.detections = []
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # Draw background
        painter.fillRect(0, 0, w, h, self._bg_color)

        # Calculate scale factors
        scale_x = w / PANO_W_PX
        scale_y = h / PANO_H_PX

        # Draw bearing scale at top
        self._draw_scale(painter, w, h)

        # Draw horizon line at cy ratio
        horizon_y = int(0.60 * h)
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(0, horizon_y, w, horizon_y)

        # Draw bounding boxes for each detection
        for det in self.detections:
            self._draw_detection(painter, det, scale_x, scale_y, w, h)

        painter.end()

    def _draw_scale(self, painter, w, h):
        """Draw bearing scale at the top of the view."""
        painter.setPen(QPen(self._scale_color, 1))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)

        # Draw tick marks and labels at key bearings
        bearings = [-90, -60, -30, 0, 30, 60, 90]
        for bearing in bearings:
            # Convert bearing to x position
            x_norm = (bearing + 90.0) / 180.0
            x = int(x_norm * w)

            # Draw tick
            painter.drawLine(x, 0, x, 10)

            # Draw label
            label = f"{bearing}°"
            fm = QFontMetrics(font)
            label_w = fm.horizontalAdvance(label)
            label_x = x - label_w // 2
            # Clamp to visible area
            label_x = max(2, min(w - label_w - 2, label_x))
            painter.drawText(label_x, 22, label)

        # Draw center line (0°)
        center_x = w // 2
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(center_x, 25, center_x, h)

    def _draw_detection(self, painter, det, scale_x, scale_y, view_w, view_h):
        """Draw a single detection bounding box with label."""
        cx = det['cx_px'] * scale_x
        cy = det['cy_px'] * scale_y
        w = det['w_px'] * scale_x
        h = det['h_px'] * scale_y

        x1 = cx - w / 2
        y1 = cy - h / 2

        # Draw filled rectangle
        painter.setPen(QPen(self._bbox_color, 2))
        painter.setBrush(QBrush(self._bbox_fill))
        painter.drawRect(int(x1), int(y1), int(w), int(h))

        # Draw label above box
        painter.setPen(self._text_color)
        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)

        ship_name = det.get('ship_name', f"Ship-{det['ship_idx']}")
        rel_bearing = det['rel_bearing']
        label = f"{ship_name} ({rel_bearing:.1f}°)"

        fm = QFontMetrics(font)
        label_w = fm.horizontalAdvance(label)
        label_x = int(cx - label_w / 2)
        label_y = int(y1 - 5)

        # Clamp label to visible area
        label_x = max(2, min(view_w - label_w - 2, label_x))
        label_y = max(30, label_y)

        painter.drawText(label_x, label_y, label)
