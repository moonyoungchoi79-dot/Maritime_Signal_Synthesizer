import numpy as np

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QPushButton, QLabel, QTextEdit, 
    QDoubleSpinBox, QTableWidget, QHeaderView, QTableWidgetItem, QAbstractItemView, QComboBox
)
from PyQt6.QtCore import (
    pyqtSignal, Qt
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
from app.core.models.project import current_project
from app.core.geometry import haversine_distance, pixel_to_coords

class SpeedGeneratorPanel(QWidget):
    request_generate = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QHBoxLayout(self)
        
        self.chart_view = QWidget()
        v = QVBoxLayout(self.chart_view)
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self.chart_view)
        v.addWidget(self.toolbar)
        v.addWidget(self.canvas)
        self.canvas.mpl_connect("motion_notify_event", self.on_hover)
        self.ax = self.figure.add_subplot(111)
        
        self.controls = QWidget()
        c_layout = QVBoxLayout(self.controls)
        
        # Filter Combo
        self.combo_filter = QComboBox()
        self.combo_filter.addItems(["All Ships", "Manual Ships", "Random Targets"])
        self.combo_filter.currentIndexChanged.connect(self.refresh_table)
        c_layout.addWidget(self.combo_filter)

        # Ship List Table
        self.ship_table = QTableWidget()
        self.ship_table.setColumnCount(4)
        self.ship_table.setHorizontalHeaderLabels(["chk", "ID", "Name", "Gen"])
        self.ship_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.ship_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.ship_table.verticalHeader().setVisible(False)
        self.ship_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.ship_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.ship_table.itemClicked.connect(self.on_table_clicked)
        c_layout.addWidget(self.ship_table)
        
        self.refresh_table()
        
        self.spin_var = QDoubleSpinBox()
        self.spin_var.setRange(0, 100.0)
        self.spin_var.setValue(1.0)
        self.spin_var.setSingleStep(0.1)
        
        f = QFormLayout()
        f.addRow("Variance:", self.spin_var)
        c_layout.addLayout(f)
        
        btn_gen = QPushButton("Generate Selected")
        btn_gen.clicked.connect(self.request_generate_click)
        c_layout.addWidget(btn_gen)
        
        self.log = QTextEdit()
        c_layout.addWidget(self.log)
        
        self.layout.addWidget(self.chart_view, 3)
        self.layout.addWidget(self.controls, 1)
        
        self.ship_idx = None

    def format_time(self, seconds):
        d = int(seconds // 86400)
        h = int((seconds % 86400) // 3600)
        m = int((seconds % 3600) // 60)
        s = seconds % 60
        if d > 0: return f"{d}d {h}h {m}m {s:.1f}s"
        if h > 0: return f"{h}h {m}m {s:.1f}s"
        if m > 0: return f"{m}m {s:.1f}s"
        return f"{s:.1f}s"
        
    def refresh_table(self):
        self.ship_table.blockSignals(True)
        self.ship_table.setRowCount(0)
        
        filter_mode = self.combo_filter.currentText()
        
        all_ships = sorted(current_project.ships, key=lambda s: s.idx)
        filtered_ships = []
        for s in all_ships:
            if filter_mode == "All Ships":
                filtered_ships.append(s)
            elif filter_mode == "Manual Ships":
                if s.idx < 1000: filtered_ships.append(s)
            elif filter_mode == "Random Targets":
                if s.idx >= 1000: filtered_ships.append(s)
        
        self.ship_table.setRowCount(len(filtered_ships))
        
        for row, ship in enumerate(filtered_ships):
            # Checkbox
            chk_item = QTableWidgetItem()
            chk_item.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            chk_item.setCheckState(Qt.CheckState.Checked) # Default checked
            self.ship_table.setItem(row, 0, chk_item)
            
            # ID
            id_item = QTableWidgetItem(str(ship.idx))
            id_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            self.ship_table.setItem(row, 1, id_item)
            
            # Name
            name_item = QTableWidgetItem(ship.name)
            name_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            self.ship_table.setItem(row, 2, name_item)
            
            # Generated
            gen_str = "Yes" if ship.is_generated else "No"
            gen_item = QTableWidgetItem(gen_str)
            gen_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            if ship.is_generated:
                gen_item.setForeground(Qt.GlobalColor.green)
            else:
                gen_item.setForeground(Qt.GlobalColor.red)
            self.ship_table.setItem(row, 3, gen_item)
            
            # Store ship idx
            id_item.setData(Qt.ItemDataRole.UserRole, ship.idx)
            
        self.ship_table.blockSignals(False)

    def set_ship(self, idx):
        self.ship_idx = idx
        ship = current_project.get_ship_by_idx(idx)
        if ship:
            self.spin_var.setValue(current_project.settings.speed_variance)
            # Select row in table
            for row in range(self.ship_table.rowCount()):
                if self.ship_table.item(row, 1).data(Qt.ItemDataRole.UserRole) == idx:
                    self.ship_table.selectRow(row)
                    break
            self.refresh_graph()
            self.print_log(ship)
            
    def request_generate_click(self):
        current_project.settings.speed_variance = self.spin_var.value()
        self.request_generate.emit()

    def get_selected_ships(self):
        indices = []
        for row in range(self.ship_table.rowCount()):
            item = self.ship_table.item(row, 0)
            if item.checkState() == Qt.CheckState.Checked:
                idx = self.ship_table.item(row, 1).data(Qt.ItemDataRole.UserRole)
                indices.append(idx)
        return indices

    def on_table_clicked(self, item):
        row = item.row()
        idx = self.ship_table.item(row, 1).data(Qt.ItemDataRole.UserRole)
        self.set_ship(idx)

    def refresh_graph(self):
        # Force white background regardless of theme
        bg_color = '#ffffff'
        fg_color = '#000000'
        
        self.figure.patch.set_facecolor(bg_color)
        self.ax.clear()
        self.ax.set_facecolor(bg_color)
        self.ax.grid(True)
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Speed (kn)")
        
        self.ax.xaxis.label.set_color(fg_color)
        self.ax.yaxis.label.set_color(fg_color)
        self.ax.tick_params(axis='x', colors=fg_color)
        self.ax.tick_params(axis='y', colors=fg_color)
        for spine in self.ax.spines.values():
            spine.set_color(fg_color)
            
        self.annot = self.ax.annotate("", xy=(0,0), xytext=(10,10),textcoords="offset points",
                            bbox=dict(boxstyle="round", fc="w", alpha=0.9),
                            arrowprops=dict(arrowstyle="->"))
        self.annot.set_visible(False)
        
        ship = current_project.get_ship_by_idx(self.ship_idx)
        if not ship: return
        
        has_plot = False
        
        if ship.is_generated and ship.cumulative_time:
            ts = np.array(ship.cumulative_time)
            vs = np.array(ship.node_velocities_kn)
            
            if len(ts) > 500:
                idx = np.linspace(0, len(ts)-1, 500, dtype=int)
                t_plot = ts[idx]
                v_plot = vs[idx]
            else:
                t_plot = ts
                v_plot = vs
            
            self.ax.plot(t_plot, v_plot, 'b-', label='Output (v_out)')
            
            if ship.base_velocities_kn:
                v_base = np.array(ship.base_velocities_kn)
                if len(v_base) > 500:
                      idx = np.linspace(0, len(v_base)-1, 500, dtype=int)
                      v_base_plot = v_base[idx]
                else:
                      v_base_plot = v_base
                self.ax.plot(t_plot, v_base_plot, 'g--', label='Base Interpolated')
            
            kf_t = []
            kf_v = []
            if ship.raw_speeds:

                 lats = []
                 lons = []
                 mi = current_project.map_info
                 for px, py in ship.raw_points:
                     _, _, lat, lon = pixel_to_coords(px, py, mi)
                     lats.append(lat); lons.append(lon)
                 
                 dist_cum = [0]
                 for i in range(1, len(lats)):
                     d = haversine_distance(lats[i-1], lons[i-1], lats[i], lons[i])
                     dist_cum.append(dist_cum[-1] + d)
                 
                 total_d = dist_cum[-1] if dist_cum[-1] > 0 else 1
                 total_t = ship.total_duration_sec
                 
                 for i, d in enumerate(dist_cum):
                     ratio = d / total_d
                     kf_t.append(ratio * total_t)
                     kf_v.append(ship.raw_speeds.get(i, 5.0))
                
                 self.ax.plot(kf_t, kf_v, 'ro', label='Keyframes')
            
            # Set dynamic X-axis formatter
            total_sec = ship.total_duration_sec
            def fmt_func(x, pos):
                d = int(x // 86400)
                h = int((x % 86400) // 3600)
                m = int((x % 3600) // 60)
                s = int(x % 60)
                if total_sec < 3600:
                    return f"{m}m {s}s"
                elif total_sec < 86400:
                    return f"{h}h {m}m"
                else:
                    return f"{d}d {h}h"
            self.ax.xaxis.set_major_formatter(ticker.FuncFormatter(fmt_func))
            has_plot = True
        
        if not has_plot:
            self.ax.text(0.5, 0.5, "Not Generated", transform=self.ax.transAxes, ha='center', color=fg_color)
        else:
            self.ax.legend()
        self.canvas.draw()
        
    def on_hover(self, event):
        if event.inaxes == self.ax:
            ship = current_project.get_ship_by_idx(self.ship_idx)
            if not ship or not ship.is_generated or not ship.cumulative_time: return
            
            times = np.array(ship.cumulative_time)
            idx = (np.abs(times - event.xdata)).argmin()
            
            t = times[idx]
            v = ship.node_velocities_kn[idx]
            
            self.annot.xy = (t, v)
            self.annot.set_text(f"T: {self.format_time(t)}\nV: {v:.1f}kn")
            self.annot.set_visible(True)
            self.canvas.draw_idle()
        else:
            if hasattr(self, 'annot') and self.annot.get_visible():
                self.annot.set_visible(False)
                self.canvas.draw_idle()

    def print_log(self, ship):
        self.log.clear()
        if not ship.is_generated: 
            self.log.setText("Click 'Generate Speed' to create time series.")
            return
        self.log.append(f"Duration: {self.format_time(ship.total_duration_sec)}")
        self.log.append(f"Segments: {len(ship.cumulative_time)}")
        self.log.append("Data is interpolated on-the-fly (Lazy Eval).")

    def reset_state(self):
        self.log.clear()
        self.log.setText("Reset due to path change.")
        self.ax.clear()
        self.canvas.draw()