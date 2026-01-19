from PyQt6.QtWidgets import QGraphicsView, QGraphicsItem, QGraphicsPolygonItem
from PyQt6.QtCore import Qt, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QMouseEvent

class SimMapView(QGraphicsView):
    view_changed = pyqtSignal()
    # [New] Signal for ship selection sync
    ship_clicked = pyqtSignal(int) 

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        
        self.scale_factor = 1.15
        self.mode = "VIEW" 
        
        # For ruler
        self.ruler_points = []
        self.temp_ruler_line = None

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.scale(self.scale_factor, self.scale_factor)
        else:
            self.scale(1 / self.scale_factor, 1 / self.scale_factor)
        self.view_changed.emit()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.mode == "RULER":
                sp = self.mapToScene(event.pos())
                self.ruler_points.append(sp)
                self.parent().update_ruler(self.ruler_points)
                return
            elif self.mode == "VIEW":
                # Check if a ship item is clicked
                item = self.itemAt(event.pos())
                if item:
                    # In SimulationPanel, ship markers are QGraphicsPolygonItem
                    # and we set data(0, ship_idx)
                    idx = item.data(0)
                    if idx is not None and isinstance(idx, int):
                        self.ship_clicked.emit(idx)
        
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.mode == "RULER" and len(self.ruler_points) > 0:
            sp = self.mapToScene(event.pos())
            current_points = self.ruler_points + [sp]
            self.parent().update_ruler(current_points)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        super().mouseReleaseEvent(event)