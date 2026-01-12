from PyQt6.QtWidgets import (
    QApplication, QTabBar, QStyleOptionTab
)
from PyQt6.QtCore import (
    Qt
)
from PyQt6.QtGui import (
    QColor, QPainter
)
from app.core.models.project import current_project

class ColoredTabBar(QTabBar):
    def paintEvent(self, event):
        painter = QPainter(self)
        option = QStyleOptionTab()
        
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else: # System
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True

        bg_color = QColor(Qt.GlobalColor.black) if is_dark else QColor(Qt.GlobalColor.white)
        text_color = QColor(Qt.GlobalColor.white) if is_dark else QColor(Qt.GlobalColor.black)

        for i in range(self.count()):
            self.initStyleOption(option, i)
            
            # Draw background
            painter.fillRect(option.rect, bg_color)
            
            # Draw Text
            painter.setPen(text_color)
            font = painter.font()
            painter.setFont(font)
            painter.drawText(option.rect, Qt.AlignmentFlag.AlignCenter, self.tabText(i))
            
            # Draw Icon if exists (Simple centered-left drawing)
            if not self.tabIcon(i).isNull():
                self.tabIcon(i).paint(painter, option.rect.adjusted(5, 0, -5, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
