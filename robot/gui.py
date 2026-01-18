# File: robot/gui.py

import sys
import functools
import warnings
import numpy as np
import trimesh
import re
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D, QTextFormat, QColor, QFont, QPixmap
from PyQt5.QtWidgets import QLabel, QGroupBox, QFormLayout, QStyle, QDialog, QProgressBar, QVBoxLayout
import pyqtgraph.opengl as gl
from OpenGL import GL
from scipy.spatial.transform import Rotation as R

# === ICON SUPPORT (QTAwesome or Fallback) ===
try:
    import qtawesome as qta
    HAS_QTA = True
except ImportError:
    HAS_QTA = False
    print("Info: Install 'pip install qtawesome' to get nice icons.")

# === LOCAL IMPORTS ===
import robot.config as config
from robot.kinematics import RobotKinematics
from robot.worker import RobotWorker
try:
    import robot.communication as comm
except ImportError:
    from robot.worker import comm  # Mock

# ------------------------
# Helper: Convert 4x4 Matrix -> QMatrix4x4
# ------------------------
def matrix_to_qtransform(matrix):
    m = QMatrix4x4()
    for i in range(4):
        m.setRow(i, QVector4D(float(matrix[i, 0]), float(matrix[i, 1]), float(matrix[i, 2]), float(matrix[i, 3])))
    return m

# ------------------------
# Helper: Safe Float Parsing
# ------------------------
def parse_float_from_input(line_edit, default_val=0.0):
    text = line_edit.text().strip().replace(',', '.')
    if not text or text == "-":
        return default_val
    try:
        return float(text)
    except ValueError:
        print(f"Invalid value in field: '{text}', using default {default_val}")
        return default_val

# ------------------------
# Class: Homing Dialog (Status Indicators) - Final Polish (Glow + Visible Tiles)
# ------------------------
class HomingDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_ref = parent
        self.setWindowTitle("Robot Homing Procedure")
        
        # 1. Window settings
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedSize(520, 350) # Slightly taller to fit tiles
        self.setModal(True) 

        # 2. Layout with margin for glow
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(20, 20, 20, 20)

        # 3. Main frame
        self.frame = QtWidgets.QFrame()
        self.frame.setObjectName("MainFrame")
        
        # === NEON GLOW EFFECT ===
        # Instead of black shadow, use theme color (Turquoise/Cyan)
        shadow = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(25)
        shadow.setXOffset(0)
        shadow.setYOffset(0)
        # Color: R, G, B, Alpha (opacity) -> Turquoise glow
        shadow.setColor(QColor(0, 229, 255, 60)) 
        self.frame.setGraphicsEffect(shadow)

        # 4. CSS - Modern and readable
        self.frame.setStyleSheet("""
            QFrame#MainFrame {
                background-color: #262626; /* Slightly lighter background than 3D black */
                border: 1px solid #444;    /* Subtle border */
                border-radius: 12px;
            }
            
            QLabel { font-family: 'Segoe UI', sans-serif; }
            
            QLabel#TitleLabel {
                font-size: 18px; 
                font-weight: bold; 
                color: #ffffff;
                letter-spacing: 1px;
                text-transform: uppercase;
            }
            
            QPushButton#CloseBtn {
                background-color: transparent;
                color: #666;
                font-weight: bold;
                font-size: 18px;
                border: none;
            }
            QPushButton#CloseBtn:hover { color: #ff5252; }

            /* ACTION BUTTONS */
            QPushButton#BtnAction {
                background-color: #333;
                color: #eee;
                border: 1px solid #555;
                border-radius: 6px;
                padding: 12px;
                font-weight: bold;
                font-size: 13px;
                text-align: left;
                padding-left: 20px;
            }
            QPushButton#BtnAction:hover {
                background-color: #3d3d3d;
                border: 1px solid #00e5ff;
                color: white;
            }
            QPushButton#BtnAction:pressed { background-color: #222; }

            /* AXIS TILES */
            QLabel[role="axis_indicator"] {
                background-color: #1a1a1a;
                color: #444; /* Default dark text (inactive) */
                border: 1px solid #333;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            
            /* ACTIVE Tiles (during homing) */
            QLabel[role="axis_indicator"][status="idle"] {
                color: #777;
                border: 1px solid #555;
            }
            
            /* HIT - Orange */
            QLabel[role="axis_indicator"][status="hit"] {
                background-color: #3e2723; 
                color: #ffab91;
                border: 1px solid #ff5722;
            }
            /* DONE - Green */
            QLabel[role="axis_indicator"][status="done"] {
                background-color: #1b5e20; 
                color: #a5d6a7;
                border: 1px solid #00e676;
            }
            QPushButton#CloseBtn {
                background-color: transparent;
                color: #aaaaaa; /* Lighter gray to be visible on dark background */
                font-weight: 900; /* Very bold font */
                font-size: 20px;
                border: none;
                margin: 0px;
                padding: 0px;
                min-width: 30px;
                min-height: 30px;
            }
            QPushButton#CloseBtn:hover {
                color: #ffffff; /* White on hover */
                background-color: #c62828; /* Red background on hover */
                border-radius: 15px; /* Round background */
            }

            QProgressBar {
                border: none;
                background-color: #1a1a1a;
                height: 4px;
                border-radius: 2px;
            }
            QProgressBar::chunk { background-color: #00e5ff; }
        """)

        outer_layout.addWidget(self.frame)

        # 5. Building content
        self.layout = QVBoxLayout(self.frame)
        self.layout.setContentsMargins(30, 25, 30, 30)
        self.layout.setSpacing(15)

        # --- TOP ---
        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0) # Optional: remove header margins
        self.lbl_title = QLabel("Homing Procedure")
        self.lbl_title.setObjectName("TitleLabel")
        
        self.btn_close = QtWidgets.QPushButton("✕")
        self.btn_close.setObjectName("CloseBtn")
        self.btn_close.setFixedSize(30, 30)
        self.btn_close.setCursor(Qt.PointingHandCursor)
        self.btn_close.clicked.connect(self.reject)

        header.addWidget(self.lbl_title)
        header.addStretch()
        header.addWidget(self.btn_close)
        self.layout.addLayout(header)

        # Line
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet("background-color: #444; max-height: 1px;")
        self.layout.addWidget(line)

        # Description
        self.lbl_desc = QLabel("System needs calibration. Please ensure workspace is clear.")
        self.lbl_desc.setStyleSheet("color: #999; font-size: 13px; margin-bottom: 5px;")
        self.lbl_desc.setAlignment(Qt.AlignLeft)
        self.layout.addWidget(self.lbl_desc)

        # --- BUTTONS ---
        self.btn_container = QtWidgets.QWidget()
        btn_layout = QVBoxLayout(self.btn_container)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(10)

        self.btn_start = QtWidgets.QPushButton("🏠  Start Auto-Homing Sequence")
        self.btn_start.setObjectName("BtnAction")
        self.btn_start.setCursor(Qt.PointingHandCursor)
        self.btn_start.clicked.connect(self.start_auto_homing)
        
        self.btn_confirm = QtWidgets.QPushButton("✅  Manual Confirm ")
        self.btn_confirm.setObjectName("BtnAction")
        self.btn_confirm.setCursor(Qt.PointingHandCursor)
        self.btn_confirm.clicked.connect(self.manual_confirm)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_confirm)
        self.layout.addWidget(self.btn_container)

        # --- PROGRESS BAR (Hidden by default) ---
        self.progress_container = QtWidgets.QWidget()
        p_layout = QVBoxLayout(self.progress_container)
        p_layout.setContentsMargins(0, 10, 0, 10)
        
        self.lbl_progress = QLabel("Calibrating axes...")
        self.lbl_progress.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.lbl_progress.setAlignment(Qt.AlignCenter)
        
        self.spinner = QProgressBar()
        self.spinner.setRange(0, 0)
        self.spinner.setTextVisible(False)
        
        p_layout.addWidget(self.lbl_progress)
        p_layout.addWidget(self.spinner)
        self.progress_container.hide()
        self.layout.addWidget(self.progress_container)

        # --- AXIS TILES (Now ALWAYS visible at the bottom) ---
        # So the window is not empty
        self.indicators_container = QtWidgets.QWidget()
        ind_layout = QtWidgets.QHBoxLayout(self.indicators_container)
        ind_layout.setSpacing(10)
        ind_layout.setContentsMargins(0, 10, 0, 0)

        self.axis_labels = []
        for i in range(6):
            lbl = QLabel(f"J{i+1}")
            lbl.setFixedSize(40, 40)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setProperty("role", "axis_indicator")
            # Initial state - idle (gray)
            lbl.setProperty("status", "idle_dimmed") 
            self.axis_labels.append(lbl)
            ind_layout.addWidget(lbl)

        self.layout.addStretch()
        self.layout.addWidget(self.indicators_container)

    def set_axis_state(self, axis_idx, state):
        if axis_idx < 0 or axis_idx >= 6: return
        lbl = self.axis_labels[axis_idx]
        
        lbl.setText(f"J{axis_idx+1}")
        lbl.setProperty("status", state.lower())
        lbl.style().unpolish(lbl)
        lbl.style().polish(lbl)

    def start_auto_homing(self):
        self.lbl_desc.setText("Homing in progress. Do not interfere.")
        self.btn_container.hide()
        self.progress_container.show()
        
        # Change tiles status to 'idle' (lighter border)
        for i in range(6): 
            lbl = self.axis_labels[i]
            lbl.setProperty("status", "idle")
            lbl.style().unpolish(lbl); lbl.style().polish(lbl)

        if self.parent_ref:
            self.parent_ref.is_homed = False
            self.parent_ref.request_homing_signal.emit()

    def manual_confirm(self):
        msg = QtWidgets.QMessageBox(self)
        msg.setWindowTitle("Confirm")
        msg.setText("Are you sure robot is at HOME?")
        msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msg.setDefaultButton(QtWidgets.QMessageBox.No)
        msg.setStyleSheet("QMessageBox { background-color: #2b2b2b; color: #ddd; } QPushButton { background-color: #444; color: white; padding: 5px 15px; }")
        
        if msg.exec_() == QtWidgets.QMessageBox.Yes:
            if self.parent_ref:
                self.parent_ref.is_homed = True
                self.parent_ref.update_status_label("Manual Homing Confirmed", "green")
            self.accept()

# ------------------------
# Class: Tool Selection Dialog (Graphical)
# ------------------------
class ToolSelectionDialog(QDialog):
    # Signal returning selected tool name (matches config)
    tool_selected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select End Effector")
        
        # Window settings - frameless, always on top, transparent background for rounded corners
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedSize(600, 350)
        self.setModal(True)

        # Main CSS style
        self.setStyleSheet("""
            QDialog {
                background-color: #2b2b2b;
                border: 2px solid #444;
                border-radius: 15px;
            }
            QLabel {
                color: white;
                font-weight: bold;
                font-size: 18px;
                background: transparent;
            }
            QToolButton {
                background-color: #3c3c3c;
                border: 2px solid #555;
                border-radius: 10px;
                color: #ddd;
                font-size: 14px;
                font-weight: bold;
                padding: 10px;
            }
            QToolButton:hover {
                background-color: #505050;
                border: 2px solid #00e5ff; /* Turquoise highlight */
                color: white;
            }
            QToolButton:pressed {
                background-color: #222;
                border: 2px solid #00b8d4;
            }
            QPushButton#CloseBtn {
                background-color: transparent;
                color: #aaaaaa; /* Lighter gray to be visible on dark background */
                font-weight: 900; /* Very bold font */
                font-size: 20px;
                border: none;
                margin: 0px;
                padding: 0px;
                min-width: 30px;
                min-height: 30px;
            }
            QPushButton#CloseBtn:hover {
                color: #ffffff; /* White on hover */
                background-color: #c62828; /* Red background on hover */
                border-radius: 15px; /* Round background */
            }
        """)

        # Main layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 10, 20, 30)

        # --- Header with close button ---
        header_layout = QtWidgets.QHBoxLayout()
        lbl_title = QLabel("CHANGE ACTIVE TOOL")
        lbl_title.setAlignment(Qt.AlignCenter)
        
        btn_close = QtWidgets.QPushButton("✕")
        btn_close.setObjectName("CloseBtn")
        btn_close.setFixedSize(30, 30)
        btn_close.clicked.connect(self.reject)

        header_layout.addStretch()
        header_layout.addWidget(lbl_title)
        header_layout.addStretch()
        header_layout.addWidget(btn_close)
        layout.addLayout(header_layout)

        layout.addSpacing(10)

        # --- Container for tool buttons ---
        tools_layout = QtWidgets.QHBoxLayout()
        tools_layout.setSpacing(30)

        # Button 1: PNEUMATIC (VGrip)
        self.btn_v_grip = QtWidgets.QToolButton()
        self.btn_v_grip.setText("Vacuum Gripper")
        self.btn_v_grip.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.btn_v_grip.setIconSize(QtCore.QSize(180, 150)) # Image size
        self.btn_v_grip.setFixedSize(220, 220) # Entire button size
        
        # Loading VGrip.png image
        pix_v = QPixmap("assets/VGrip.png")
        if not pix_v.isNull():
            self.btn_v_grip.setIcon(QtGui.QIcon(pix_v))
        else:
            self.btn_v_grip.setText("Vacuum Gripper\n(NO IMAGE)")

        # Click selects tool "CHWYTAK_MALY" (as per old code)
        self.btn_v_grip.clicked.connect(lambda: self.select_tool("CHWYTAK_MALY"))
        
        # Button 2: ELECTRIC (EGrip)
        self.btn_e_grip = QtWidgets.QToolButton()
        self.btn_e_grip.setText("Electric Gripper")
        self.btn_e_grip.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.btn_e_grip.setIconSize(QtCore.QSize(180, 150))
        self.btn_e_grip.setFixedSize(220, 220)

        # Loading EGrip.png image
        pix_e = QPixmap("assets/EGrip.png")
        if not pix_e.isNull():
            self.btn_e_grip.setIcon(QtGui.QIcon(pix_e))
        else:
            self.btn_e_grip.setText("Electric Gripper\n(NO IMAGE)")

        # Click selects tool "CHWYTAK_DUZY"
        self.btn_e_grip.clicked.connect(lambda: self.select_tool("CHWYTAK_DUZY"))

        tools_layout.addStretch()
        tools_layout.addWidget(self.btn_v_grip)
        tools_layout.addWidget(self.btn_e_grip)
        tools_layout.addStretch()

        layout.addLayout(tools_layout)

    def select_tool(self, tool_code):
        self.tool_selected.emit(tool_code)
        self.accept() # Closes window



# ------------------------
# Class: About Dialog (Author Info) - REDESIGN V2
# ------------------------
class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("About")
        
        # 1. Window settings (Frameless, Transparent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedSize(400, 520) # Slightly taller for better spacing
        self.setModal(True)

        # 2. Outer layout (margin for shadow)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(20, 20, 20, 20)

        # 3. Main frame (Card)
        self.frame = QtWidgets.QFrame()
        self.frame.setObjectName("AboutFrame")
        
        # Glow Effect (Turquoise, matching the rest)
        shadow = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(30)
        shadow.setXOffset(0)
        shadow.setYOffset(0)
        shadow.setColor(QColor(0, 229, 255, 80)) # Turquoise glow
        self.frame.setGraphicsEffect(shadow)

        # 4. CSS Style - "Business Card"
        self.frame.setStyleSheet("""
            QFrame#AboutFrame {
                background-color: #1e1e1e; /* Dark background */
                border: 1px solid #444;
                border-radius: 16px;
            }
            
            QLabel {
                font-family: 'Segoe UI', sans-serif;
                background: transparent;
                border: none;
            }

            /* Logo Container - White background for image */
            QLabel#LogoContainer {
                background-color: white; 
                border-radius: 12px; /* Rounded logo corners */
                border: 4px solid #333; /* Frame around logo */
            }

            /* Typography */
            QLabel#Title { 
                font-size: 13px; 
                font-weight: bold; 
                color: #00e5ff; /* Turquoise */
                letter-spacing: 2px;
                text-transform: uppercase;
            }
            QLabel#AuthorName { 
                font-size: 26px; 
                font-weight: 900; 
                color: white; 
                margin: 5px 0;
            }
            QLabel#Subtitle { 
                font-size: 15px; 
                color: #aaaaaa; 
                font-style: italic; 
            }
            
            QLabel#University { 
                font-size: 16px; 
                font-weight: bold; 
                color: #e0e0e0; 
            }
            QLabel#Year { 
                font-size: 13px; 
                color: #666; 
                font-weight: bold;
            }

            /* X button in corner */
            QPushButton#CloseX {
                background-color: transparent;
                color: #888;
                font-size: 20px;
                font-weight: bold;
                border: none;
            }
            QPushButton#CloseX:hover {
                color: white;
                background-color: #c62828;
                border-radius: 15px;
            }
            
            /* Bottom Close button */
            QPushButton#BtnClose {
                background-color: #333;
                color: #ccc;
                border: 1px solid #555;
                border-radius: 6px;
                padding: 8px 20px;
                font-weight: bold;
            }
            QPushButton#BtnClose:hover {
                background-color: #444;
                color: white;
                border: 1px solid #00e5ff;
            }
        """)

        outer_layout.addWidget(self.frame)
        
        # 5. Building content
        layout = QVBoxLayout(self.frame)
        layout.setAlignment(Qt.AlignTop)
        layout.setContentsMargins(20, 15, 20, 30)
        layout.setSpacing(5)

        # --- TOP: X to close ---
        header_layout = QtWidgets.QHBoxLayout()
        header_layout.addStretch()
        
        btn_x = QtWidgets.QPushButton("✕")
        btn_x.setObjectName("CloseX")
        btn_x.setFixedSize(30, 30)
        btn_x.setCursor(Qt.PointingHandCursor)
        btn_x.clicked.connect(self.accept)
        header_layout.addWidget(btn_x)
        
        layout.addLayout(header_layout)

        # --- LOGO (Centered) ---
        # Logo container
        logo_container = QtWidgets.QVBoxLayout()
        logo_container.setSpacing(0)
        logo_container.setAlignment(Qt.AlignCenter)

        self.lbl_logo = QLabel()
        self.lbl_logo.setObjectName("LogoContainer") # Style with white background
        self.lbl_logo.setAlignment(Qt.AlignCenter)
        self.lbl_logo.setFixedSize(140, 140) # Fixed square size
        
        # Loading and scaling
        pix = QPixmap("assets/AT.png")
        if not pix.isNull():
            # Scale image to have margin inside white frame
            self.lbl_logo.setPixmap(pix.scaled(110, 110, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.lbl_logo.setText("AT LOGO")
            self.lbl_logo.setStyleSheet("color: black; font-weight: bold;")
        
        logo_container.addWidget(self.lbl_logo)
        layout.addLayout(logo_container)

        layout.addSpacing(25) # Spacing below logo

        # --- TEXT (Centered) ---
        # Section 1: Thesis Title
        lbl_title = QLabel("ENGINEERING THESIS")
        lbl_title.setObjectName("Title")
        lbl_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_title)

        # Section 2: Author
        lbl_author = QLabel("Jakub Grzebień")
        lbl_author.setObjectName("AuthorName")
        lbl_author.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_author)

    
        layout.addSpacing(30) # Spacing before university

        # Section 3: University
        # Adding graduation cap icon (text) for decoration
        lbl_uni = QLabel("🎓 Tarnow Academy")
        lbl_uni.setObjectName("University")
        lbl_uni.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_uni)

        lbl_year = QLabel("2025 - 2026")
        lbl_year.setObjectName("Year")
        lbl_year.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_year)

        layout.addStretch()

        # --- BOTTOM: Close Button ---
        btn_close = QtWidgets.QPushButton("Close")
        btn_close.setObjectName("BtnClose")
        btn_close.setCursor(Qt.PointingHandCursor)
        btn_close.clicked.connect(self.accept)
        
        # Center button at the bottom
        bottom_layout = QtWidgets.QHBoxLayout()
        bottom_layout.addStretch()
        bottom_layout.addWidget(btn_close)
        bottom_layout.addStretch()
        
        layout.addLayout(bottom_layout)




# ------------------------
# Class: Main Window
# ------------------------
class MainWindow(QtWidgets.QWidget):
    
    # --- SIGNALS ---
    start_move_signal = pyqtSignal(np.ndarray)
    start_linear_move_signal = pyqtSignal(np.ndarray, np.ndarray)
    set_port_signal = pyqtSignal(str)
    request_homing_signal = pyqtSignal()
    request_gripper_signal = pyqtSignal(str)
    start_program_signal = pyqtSignal(str)
    stop_program_signal = pyqtSignal()
    change_tool_signal = pyqtSignal(str)
    
    # SPEED SIGNAL
    set_speed_signal = pyqtSignal(float) 

    def __init__(self):
        super().__init__()
        
        # Initialize Kinematics
        self.kinematics = RobotKinematics(config.URDF_PATH, None)
        self.joint_limits_rad = self.kinematics.get_joint_limits()
        self.visual_origins = self.kinematics.get_visual_origins()
        
        # Start Position
        self.standby_angles = np.array([0] + [np.radians(a) for a in config.STANDBY_ANGLES_DEG])
        self.current_orientation = None 

        # >>> NEW: Gripper states <<<
        self.pneumatic_active = False # True = ON
        self.electric_active = False  # True = CLOSE
        
        self.l6_color_state = 0 
        self.default_link_colors = {} 
        self.joint_value_labels = []
        self.tcp_value_labels = {}
        
        self.homing_dialog = None
        self.update_pending = False
        self.is_homed = False 
        
        self._apply_dark_style()
        self.init_gui()
        self._create_estop_overlay()
        
        self.init_worker()
        self.update_sliders_from_angles(self.standby_angles)
        self.apply_transforms_from_angles(self.standby_angles)
        initial_fk = self.kinematics.forward_kinematics(self.standby_angles)
        if initial_fk is not None:
            self.current_orientation = initial_fk
            self.update_gui_feedback(initial_fk)
        print("Application ready.")
        QtCore.QTimer.singleShot(100, self.go_standby)

    def _apply_dark_style(self):
        """Modern dark CSS style."""
        dark_style = """
        QWidget {
            background-color: #2b2b2b;
            color: #e0e0e0;
            font-family: 'Segoe UI', 'Roboto', sans-serif;
            font-size: 14px;
        }
        QGroupBox {
            border: 1px solid #444;
            border-radius: 6px;
            margin-top: 6px;
            font-weight: bold;
            color: #aaa;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 5px;
            left: 10px;
        }
        QLineEdit {
            background-color: #1e1e1e;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 6px;
            color: #fff;
            selection-background-color: #007acc;
        }
        QLineEdit:focus {
            border: 1px solid #007acc;
        }
        QComboBox {
            background-color: #3c3c3c;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 5px;
            color: #fff;
        }
        QTabWidget::pane {
            border: 1px solid #444;
            border-radius: 4px;
        }
        QTabBar::tab {
            background: #3c3c3c;
            padding: 8px 20px;
            margin-right: 2px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:selected {
            background: #505050;
            color: #fff;
            border-bottom: 2px solid #007acc;
        }
        QLabel {
            color: #ccc;
        }
        QSlider::groove:horizontal {
            border: 1px solid #333;
            height: 6px;
            background: #1a1a1a;
            margin: 2px 0;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #007acc;
            border: 1px solid #007acc;
            width: 16px;
            height: 16px;
            margin: -6px 0;
            border-radius: 8px;
        }
        QSlider::handle:horizontal:hover {
            background: #0099ff;
        }
        QPushButton {
            background-color: #444;
            border: none;
            border-radius: 5px;
            padding: 8px 15px;
            color: white;
            font-weight: bold;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #555;
        }
        QPushButton:pressed {
            background-color: #333;
        }
        QPushButton:disabled {
            background-color: #333;
            color: #777;
        }
        QPlainTextEdit {
            background-color: #1e1e1e;
            color: #a9b7c6;
            border: 1px solid #444;
            font-family: 'Consolas', 'Monospace';
        }
        """
        self.setStyleSheet(dark_style)

    def init_gui(self):
        self.setWindowTitle("PAROL6 3D Control Center by Jakub Grzebień 2025")
        self.resize(1400, 850)
        
        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(15)
        
        self.control_widget = self._create_left_control_panel()
        self.main_layout.addWidget(self.control_widget)

        self.view = self._create_3d_visualization_panel()
        container_3d = QtWidgets.QWidget()
        container_3d.setStyleSheet("background-color: #000; border: 1px solid #555; border-radius: 4px;")
        l3d = QtWidgets.QVBoxLayout(container_3d)
        l3d.setContentsMargins(1, 1, 1, 1)
        l3d.addWidget(self.view)
        
        self.main_layout.addWidget(container_3d)

        right_panel = self._create_right_panel()
        self.main_layout.addWidget(right_panel)

        self.main_layout.setStretch(0, 0) # Left
        self.main_layout.setStretch(1, 1) # 3D (Stretchable)
        self.main_layout.setStretch(2, 0) # Right

        self._load_3d_meshes()
        self._connect_gui_signals()

    def _create_estop_overlay(self):
        self.estop_overlay = QtWidgets.QFrame(self)
        self.estop_overlay.setStyleSheet("background-color: rgba(200, 20, 20, 220);")
        self.estop_overlay.hide()

        overlay_layout = QtWidgets.QVBoxLayout(self.estop_overlay)
        overlay_layout.setAlignment(Qt.AlignCenter)
        overlay_layout.setSpacing(20)

        lbl_image = QLabel()
        lbl_image.setAlignment(Qt.AlignCenter)
        lbl_image.setStyleSheet("background: transparent; border: none;")

        image_path = 'assets/ESTOP.png' 
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            scaled_pixmap = pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            lbl_image.setPixmap(scaled_pixmap)
        else:
            lbl_image.setText("🛑")
            lbl_image.setStyleSheet("background: transparent; color: white; font-size: 100px;")

        lbl_text = QLabel("EMERGENCY STOP")
        lbl_text.setStyleSheet("background: transparent; color: white; font-weight: 900; font-size: 48px; border: none;")
        lbl_text.setAlignment(Qt.AlignCenter)

        lbl_desc = QLabel("E-Stop button pressed.\nRelease E-Stop to resume operation.")
        lbl_desc.setStyleSheet("background: transparent; color: #eee; font-size: 18px; border: none;")
        lbl_desc.setAlignment(Qt.AlignCenter)

        overlay_layout.addStretch()
        overlay_layout.addWidget(lbl_image)
        overlay_layout.addWidget(lbl_text)
        overlay_layout.addWidget(lbl_desc)
        overlay_layout.addStretch()

    def resizeEvent(self, event):
        if hasattr(self, 'estop_overlay'):
            self.estop_overlay.resize(self.width(), self.height())
        super().resizeEvent(event)

    def _create_left_control_panel(self):
        control_widget = QtWidgets.QWidget()
        control_widget.setFixedWidth(280)
        control_layout = QtWidgets.QVBoxLayout(control_widget)
        control_layout.setSpacing(10)
        control_layout.setContentsMargins(0, 0, 0, 0)

        # === PANEL HEADER (Title + ABOUT Button) ===
        header_widget = QtWidgets.QWidget()
        header_layout = QtWidgets.QHBoxLayout(header_widget)
        header_layout.setContentsMargins(5, 5, 5, 0) # Slight top margin

        lbl_panel_name = QLabel("PAROL6 CONTROL")
        lbl_panel_name.setStyleSheet("font-weight: 900; color: #666; font-size: 12px; letter-spacing: 1px;")
        
        # >>> CHANGE: ABOUT button instead of 'i' circle <<<
        self.btn_info = QtWidgets.QPushButton("ABOUT")
        self.btn_info.setFixedSize(85, 26) # Wider, rectangular
        self.btn_info.setCursor(Qt.PointingHandCursor)
        
        # Optional: Add icon if library exists
        if HAS_QTA:
            self.btn_info.setIcon(qta.icon("fa5s.info-circle", color='#00e5ff'))
            self.btn_info.setIconSize(QtCore.QSize(14, 14))

        self.btn_info.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #00e5ff; /* Turquoise text */
                border: 1px solid #00e5ff; /* Turquoise border */
                border-radius: 4px; /* Slightly rounded corners */
                font-weight: bold;
                font-size: 11px;
                font-family: 'Segoe UI', sans-serif;
                padding-bottom: 2px; /* Vertical text adjustment */
            }
            QPushButton:hover {
                background-color: rgba(0, 229, 255, 0.15); /* Subtle background highlight */
                border: 1px solid #00b8d4;
            }
            QPushButton:pressed {
                background-color: #00e5ff; /* Full color on click */
                color: #121212; /* Black text for contrast */
            }
        """)
        # Connecting window open function
        self.btn_info.clicked.connect(self.show_about_dialog)

        header_layout.addWidget(lbl_panel_name)
        header_layout.addStretch()
        header_layout.addWidget(self.btn_info)
        
        control_layout.addWidget(header_widget)
        # ===============================================

        # --- TABS ---
        self.tab_widget = QtWidgets.QTabWidget()
        control_layout.addWidget(self.tab_widget)

        # === TAB 1: Manual Control ===
        manual_tab = QtWidgets.QWidget()
        
        # Define layout primarily to avoid UnboundLocalError
        manual_layout = QtWidgets.QVBoxLayout(manual_tab) 
        manual_layout.setSpacing(15) 
        manual_layout.setContentsMargins(10, 15, 10, 10)
        
        # 1. SPEED SLIDER
        speed_group = QtWidgets.QGroupBox("Speed")
        speed_layout = QtWidgets.QHBoxLayout()
        
        self.speed_label = QtWidgets.QLabel("50%")
        self.speed_label.setFixedWidth(40)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed_label.setStyleSheet("font-weight: bold; color: #00e5ff;")

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(10, 100) # 10% to 100%
        self.speed_slider.setValue(50) 
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        speed_group.setLayout(speed_layout)
        
        manual_layout.addWidget(speed_group)

        # 2. XYZ
        xyz_group = QtWidgets.QGroupBox("Target (mm)")
        xyz_layout = QtWidgets.QGridLayout()
        xyz_layout.setVerticalSpacing(10)
        self.xyz_inputs = {}
        for i, label in enumerate(['X', 'Y', 'Z']):
            lbl = QtWidgets.QLabel(f"{label}")
            lbl.setStyleSheet("font-weight: bold; color: #bbb;")
            inp = QtWidgets.QLineEdit()
            inp.setAlignment(Qt.AlignRight)
            validator = QtGui.QDoubleValidator(config.RANGES_XYZ[i][0], config.RANGES_XYZ[i][1], 3)
            inp.setValidator(validator)
            inp.setText(f"{config.DEFAULT_XYZ_GUI[i]:.1f}")
            xyz_layout.addWidget(lbl, i, 0)
            xyz_layout.addWidget(inp, i, 1)
            self.xyz_inputs[label] = inp
        
        self.set_xyz_btn = QtWidgets.QPushButton("Move to XYZ")
        self.set_xyz_btn.setStyleSheet("background-color: #0277bd;")
        xyz_layout.addWidget(self.set_xyz_btn, 3, 0, 1, 2)
        xyz_group.setLayout(xyz_layout)
        manual_layout.addWidget(xyz_group)

        # 3. RPY
        rpy_group = QtWidgets.QGroupBox("Orientation (°)")
        rpy_layout = QtWidgets.QGridLayout()
        rpy_layout.setVerticalSpacing(10)
        self.rpy_inputs = {}
        for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            lbl = QtWidgets.QLabel(label[0]) 
            lbl.setStyleSheet("font-weight: bold; color: #bbb;")
            inp = QtWidgets.QLineEdit()
            inp.setAlignment(Qt.AlignRight)
            inp.setValidator(QtGui.QDoubleValidator(-180, 180, 2))
            inp.setText("0.0")
            rpy_layout.addWidget(lbl, i, 0)
            rpy_layout.addWidget(inp, i, 1)
            self.rpy_inputs[label] = inp
        
        self.set_rpy_btn = QtWidgets.QPushButton("Set RPY")
        self.set_rpy_btn.setStyleSheet("background-color: #00695c;")
        rpy_layout.addWidget(self.set_rpy_btn, 3, 0, 1, 2)
        rpy_group.setLayout(rpy_layout)
        manual_layout.addWidget(rpy_group)

        # 4. JOINT SLIDERS
        sliders_group = QtWidgets.QGroupBox("Joints")
        sliders_layout = QtWidgets.QVBoxLayout()
        sliders_layout.setSpacing(15) 
        self.joint_sliders = []
        self.joint_labels = []
        
        for i in range(6):
            row = QtWidgets.QHBoxLayout()
            lbl_name = QtWidgets.QLabel(f"J{i+1}")
            lbl_name.setFixedWidth(20)
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX)
            slider.setValue(self.map_rad_to_slider(0.0, i))
            
            lbl_val = QtWidgets.QLabel("0.0°")
            lbl_val.setFixedWidth(45)
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            row.addWidget(lbl_name)
            row.addWidget(slider)
            row.addWidget(lbl_val)
            
            sliders_layout.addLayout(row)
            self.joint_sliders.append(slider)
            self.joint_labels.append(lbl_val)
            
        sliders_group.setLayout(sliders_layout)
        manual_layout.addWidget(sliders_group)

        # Spacer to push banner to the bottom
        manual_layout.addStretch()

        # 5. STATUS BANNER
        self.status_banner = QtWidgets.QLabel("SYSTEM READY")
        self.status_banner.setAlignment(Qt.AlignCenter)
        self.status_banner.setFixedHeight(50) 
        self.status_banner.setWordWrap(True)
        self.status_banner.setStyleSheet("""
            QLabel {
                background-color: #333;
                color: #888;
                border: 2px solid #444;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
                padding: 5px;
            }
        """)
        manual_layout.addWidget(self.status_banner)

        # Add manual tab
        self.tab_widget.addTab(manual_tab, "Manual")

       # === TAB 2: Auto / Program (READABLE VERSION) ===
        program_tab = QtWidgets.QWidget()
        program_layout = QtWidgets.QVBoxLayout(program_tab)
        program_layout.setContentsMargins(5, 10, 5, 5) 
        program_layout.setSpacing(6)
        
        # 1. Program Editor (Larger font)
        lbl_editor = QLabel("PROGRAM EDITOR")
        lbl_editor.setStyleSheet("font-size: 9px; font-weight: bold; color: #888; margin-left: 2px;")
        program_layout.addWidget(lbl_editor)

        self.program_editor = QtWidgets.QPlainTextEdit()
        self.program_editor.setPlaceholderText("Example:\nMOVESTB\nMOVE(200, 0, 150)\nDELAY(500)")
        
        # >>> CHANGE: Font increased to 14px
        self.program_editor.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #e0e0e0;
                border: 1px solid #444;
                border-radius: 4px;
                font-family: 'Consolas', 'Monospace';
                font-size: 14px; 
                padding: 5px;
            }
        """)
        self.program_editor.textChanged.connect(self.validate_program_syntax)
        # Widget with '1' parameter stretches to fill space
        program_layout.addWidget(self.program_editor, 1) 
        
        # 2. Run/Stop Buttons
        prog_btns = QtWidgets.QHBoxLayout()
        prog_btns.setSpacing(5)

        self.run_program_btn = QtWidgets.QPushButton("▶ RUN")
        self.run_program_btn.setMinimumHeight(30)
        self.run_program_btn.setCursor(Qt.PointingHandCursor)
        self.run_program_btn.setStyleSheet("""
            QPushButton { background-color: #2e7d32; border-radius: 3px; color: white; border: 1px solid #1b5e20; font-size: 11px;}
            QPushButton:hover { background-color: #388e3c; }
        """)
        
        self.stop_program_btn = QtWidgets.QPushButton("⏹ STOP")
        self.stop_program_btn.setMinimumHeight(30)
        self.stop_program_btn.setCursor(Qt.PointingHandCursor)
        self.stop_program_btn.setEnabled(False)
        self.stop_program_btn.setStyleSheet("""
            QPushButton { background-color: #c62828; border-radius: 3px; color: white; border: 1px solid #b71c1c; font-size: 11px;}
            QPushButton:disabled { background-color: #333; color: #555; border: 1px solid #444; }
        """)
        
        prog_btns.addWidget(self.run_program_btn)
        prog_btns.addWidget(self.stop_program_btn)
        program_layout.addLayout(prog_btns)

        # Separator
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet("background-color: #333; margin: 2px 0;")
        program_layout.addWidget(line)

        # 3. Command List (No scrollbar, taller)
        lbl_help = QLabel("COMMAND REFERENCE")
        lbl_help.setStyleSheet("font-size: 9px; font-weight: bold; color: #00e5ff; margin-left: 2px;")
        program_layout.addWidget(lbl_help)

        self.cmd_table = QtWidgets.QTableWidget()
        self.cmd_table.setColumnCount(2)
        self.cmd_table.setHorizontalHeaderLabels(["Syntax", "Desc"])
        
        # Hide vertical and horizontal headers (cleaner and more space)
        self.cmd_table.verticalHeader().setVisible(False) 
        
        # Column settings
        self.cmd_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.cmd_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.cmd_table.horizontalHeader().setFixedHeight(22)
        
        # >>> CHANGE: Disable scrollbars completely
        self.cmd_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.cmd_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # >>> CHANGE: Increased table height (fits 8 rows x 24px + header)
        self.cmd_table.setFixedHeight(215) 

        self.cmd_table.setStyleSheet("""
            QTableWidget {
                background-color: #1e1e1e;
                border: 1px solid #444;
                border-radius: 3px;
                gridline-color: #2a2a2a;
                color: #ccc;
                font-size: 11px; /* Slightly larger font in table */
            }
            QHeaderView::section {
                background-color: #2b2b2b;
                color: #888;
                padding: 0px;
                border: none;
                border-bottom: 1px solid #444;
                font-weight: bold;
                font-size: 10px;
            }
            QTableWidget::item { padding-left: 4px; padding-top: 0px; padding-bottom: 0px; }
        """)

        commands_data = [
            ("MOVE(x,y,z)", "PTP move [mm]"),
            ("MOVEL(x,y,z)", "Linear move [mm]"),
            ("DELAY(ms)", "Pause [ms]"),
            ("MOVESTB", "Go Standby"),
            ("VACON", "VGRIP ON"),
            ("VACOFF", "VGRIP OFF"),
            ("EGRIP(OPEN)", "EGRIP Open"),
            ("EGRIP(CLOSE)", "EGRIP Close"),
        ]

        self.cmd_table.setRowCount(len(commands_data))
        # >>> CHANGE: Taller rows for readability (24px)
        self.cmd_table.verticalHeader().setDefaultSectionSize(24) 

        for row, (syntax, desc) in enumerate(commands_data):
            item_syntax = QtWidgets.QTableWidgetItem(syntax)
            item_syntax.setForeground(QColor("#00e5ff"))
            item_syntax.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable) 
            self.cmd_table.setItem(row, 0, item_syntax)

            item_desc = QtWidgets.QTableWidgetItem(desc)
            item_desc.setForeground(QColor("#999"))
            item_desc.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            self.cmd_table.setItem(row, 1, item_desc)

        program_layout.addWidget(self.cmd_table)
        
        self.tab_widget.addTab(program_tab, "Auto")

        control_layout.addStretch()
        return control_widget
    

    def show_about_dialog(self):
        """Displays the info dialog."""
        dlg = AboutDialog(self)
        
        # Centering relative to main window
        geo = self.geometry()
        x = geo.x() + (geo.width() - dlg.width()) // 2
        y = geo.y() + (geo.height() - dlg.height()) // 2
        dlg.move(x, y)
        
        dlg.exec_()



    def on_speed_changed(self, value):
        self.speed_label.setText(f"{value}%")
        multiplier = value / 100.0
        self.set_speed_signal.emit(multiplier)

    def _create_3d_visualization_panel(self):
        view = gl.GLViewWidget()
        view.opts['center'] = QVector3D(0, 0, 0.2)
        view.opts['msaa'] = True
        # g = gl.GLGridItem()
        # g.setSize(1, 1, 1)
        # g.setSpacing(0.1, 0.1, 0.1)
        # view.addItem(g)
        
        ax = gl.GLAxisItem()
        ax.setSize(0.2, 0.2, 0.2)
        view.addItem(ax)
        
        view.setCameraPosition(distance=0.9, elevation=30, azimuth=45)
        view.setMinimumSize(800, 600)
        view.setBackgroundColor((30, 30, 30))
        return view

    # ---------------------------------------------------------
    # RIGHT PANEL - UART, POS, BUTTONS
    # ---------------------------------------------------------
    def _create_right_panel(self):
        container = QtWidgets.QWidget()
        container.setFixedWidth(180) 
        
        layout = QtWidgets.QVBoxLayout(container)
        layout.setSpacing(10)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # === 1. UART ===
        uart_group = QtWidgets.QGroupBox("UART")
        uart_layout = QtWidgets.QVBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        try:
            import serial.tools.list_ports
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
        except ImportError:
            available_ports = []
        self.port_combo.addItems(available_ports if available_ports else ["No ports"])
        
        self.uart_status_label = QtWidgets.QLabel("Disconnected")
        self.uart_status_label.setStyleSheet("color: #777; font-size: 11px;")
        self.uart_status_label.setAlignment(Qt.AlignCenter)
        
        uart_layout.addWidget(self.port_combo)
        uart_layout.addWidget(self.uart_status_label)
        uart_group.setLayout(uart_layout)
        layout.addWidget(uart_group)

        # === 2. POSITION (DEG) ===
        pos_group = QtWidgets.QGroupBox("Position")
        pos_layout = QtWidgets.QVBoxLayout()
        pos_layout.setSpacing(5)
        
        self.joint_value_labels = [] 
        for i in range(6):
            row = QtWidgets.QHBoxLayout()
            lbl_title = QLabel(f"J{i+1}:")
            lbl_title.setStyleSheet("color: #aaa; font-size: 11px;")
            
            lbl_val = QLabel("0.0°")
            lbl_val.setFont(QFont("Monospace", 10, QFont.Bold))
            lbl_val.setStyleSheet("color: #00e5ff;")
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            self.joint_value_labels.append(lbl_val)
            row.addWidget(lbl_title)
            row.addWidget(lbl_val)
            pos_layout.addLayout(row)

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet("color: #555; margin: 5px 0;")
        pos_layout.addWidget(line)

        lbl_header_tcp = QLabel("TCP (XYZABC)")
        lbl_header_tcp.setStyleSheet("color: #fff; font-weight: bold; font-size: 11px;")
        lbl_header_tcp.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(lbl_header_tcp)

        self.tcp_value_labels = {}
        labels_map = [
            ("X", "mm"), ("Y", "mm"), ("Z", "mm"),
            ("A", "deg"), ("B", "deg"), ("C", "deg")
        ]

        for axis, unit in labels_map:
            row = QtWidgets.QHBoxLayout()
            lbl_name = QLabel(f"{axis}:")
            lbl_name.setStyleSheet("color: #aaa; font-size: 11px;")
            
            lbl_val = QLabel("0.00")
            lbl_val.setFont(QFont("Monospace", 10))
            if unit == "mm":
                lbl_val.setStyleSheet("color: #76ff03;") # Green
            else:
                lbl_val.setStyleSheet("color: #ff9100;") # Orange
            
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            self.tcp_value_labels[axis] = lbl_val
            row.addWidget(lbl_name)
            row.addWidget(lbl_val)
            pos_layout.addLayout(row)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)

        # === 3. BUTTONS ===
        btns_layout = QtWidgets.QVBoxLayout()
        btns_layout.setSpacing(10)
        
        def create_btn(text, icon_name, color_hex=None, fallback_char=""):
            btn = QtWidgets.QPushButton(text)
            
            if HAS_QTA and icon_name:
                btn.setIcon(qta.icon(icon_name, color='white'))
                btn.setIconSize(QtCore.QSize(20, 20))
            elif fallback_char:
                btn.setText(f"{fallback_char}  {text}")

            base_style = """
                QPushButton {
                    border-radius: 5px; 
                    color: white; 
                    text-align: center; 
                    padding: 8px;
                    font-weight: bold;
                    border: 1px solid rgba(255,255,255,0.1);
                }
            """
            if color_hex:
                color_style = f"""
                    QPushButton {{ background-color: {color_hex}; }}
                    QPushButton:hover {{ background-color: {color_hex}dd; }}
                    QPushButton:pressed {{ background-color: {color_hex}aa; }}
                """
            else:
                color_style = """
                    QPushButton { background-color: #424242; }
                    QPushButton:hover { background-color: #555; }
                """
            
            btn.setStyleSheet(base_style + color_style)
            btn.setMinimumHeight(45)
            return btn

        self.home_button = create_btn("HOME", "fa5s.home", "#1565c0", "⌂")          
        self.standby_btn = create_btn("STANDBY", "fa5s.pause-circle", "#6a1b9a", "⏸") 
        self.stop_btn    = create_btn("STOP", "fa5s.stop-circle", "#c62828", "🛑")    
        self.safety_btn  = create_btn("SAFETY", "fa5s.exclamation-triangle", "#ef6c00", "⚠️") 
        self.tool_btn    = create_btn("TOOL", "fa5s.tools", "#00838f", "🔧")          
        self.camera_btn  = create_btn("CAMERA", "fa5s.video", "#455a64", "📷")        

       # Pneumatic: Default OFF -> Empty circle
        self.pneumatic_btn = create_btn("VACUUM: OFF", "fa5s.circle", "#009688", "○")
        
        # Electric: Default OPEN -> Open hand
        self.electric_btn = create_btn("E-GRIP: OPEN", "fa5s.hand-paper", "#673ab7", "✋")

        btns_layout.addWidget(self.home_button)
        btns_layout.addWidget(self.standby_btn)
        btns_layout.addWidget(self.stop_btn)
        btns_layout.addWidget(self.safety_btn)
        btns_layout.addWidget(self.tool_btn)
        btns_layout.addWidget(self.camera_btn)
        
        # Adding new buttons to layout
        btns_layout.addWidget(self.pneumatic_btn)
        btns_layout.addWidget(self.electric_btn)
        
        layout.addLayout(btns_layout)
        layout.addStretch() 
        
        return container

    # ------------------------
    # Mapping
    # ------------------------
    def map_slider_to_rad(self, slider_val, joint_index):
        min_rad, max_rad = self.joint_limits_rad[joint_index]
        percentage = (slider_val - config.SLIDER_RANGE_MIN) / (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return min_rad + percentage * (max_rad - min_rad)

    def map_rad_to_slider(self, rad_val, joint_index):
        min_rad, max_rad = self.joint_limits_rad[joint_index]
        rad_range = max_rad - min_rad
        if rad_range == 0: return (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN) // 2
        percentage = (rad_val - min_rad) / rad_range
        val = config.SLIDER_RANGE_MIN + percentage * (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return int(np.clip(val, config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX))

    # ------------------------
    # Logic
    # ------------------------

    def _create_mesh_item_from_file(self, filename):
        """Helper to safely load mesh from file (STL, PLY, OBJ, 3MF) and return GLMeshItem."""
        try:
            print(f"[DEBUG] Loading file: 'assets/{filename}'...")
            mesh_data = trimesh.load(f'assets/{filename}')
            
            if isinstance(mesh_data, trimesh.Scene):
                if len(mesh_data.geometry) == 0:
                     raise ValueError("Empty Scene")
                mesh_data = mesh_data.dump(concatenate=True)

            if not isinstance(mesh_data, trimesh.Trimesh):
                 raise ValueError(f"Invalid 3D file format: {type(mesh_data)}")

            mesh_data.process()
            mesh_data.fix_normals()
            mesh_data.fill_holes()
            
            # Color conversion (Fusion OBJ/3MF support)
            if hasattr(mesh_data.visual, 'to_color'):
                try:
                    mesh_data.visual = mesh_data.visual.to_color()
                except:
                    pass
            
            if hasattr(mesh_data.visual, "vertex_colors") and mesh_data.visual.vertex_colors is not None:
                vc = (mesh_data.visual.vertex_colors.astype(np.float32) / 255.0)[:, :3]
                if len(vc) != len(mesh_data.vertices):
                     vc = np.ones((len(mesh_data.vertices), 3), dtype=np.float32) * 0.8
            else:
                vc = np.ones((len(mesh_data.vertices), 3), dtype=np.float32) * 0.8
            
            mesh_item = gl.GLMeshItem(
                vertexes=np.array(mesh_data.vertices),
                faces=np.array(mesh_data.faces),
                vertexColors=vc,
                shader='shaded',
                smooth=True,
                drawFaces=True
            )
            return mesh_item, vc

        except Exception as e:
            print(f"Error loading mesh '{filename}': {e}. Using placeholder.")
            mesh_item = gl.GLMeshItem(vertexes=np.array([[0,0,0]]))
            vc = np.array([[0.8, 0.8, 0.8]])
            return mesh_item, vc

    def _load_3d_meshes(self):
        self.links_3d = [] 
        
        # 1. Load Standard Kinematic Links
        for i, stl_file in enumerate(config.STL_FILES):
             mesh_item, vc = self._create_mesh_item_from_file(stl_file)
             self.default_link_colors[i] = vc 
             self.view.addItem(mesh_item)
             self.links_3d.append(mesh_item)
        
        # 2. Load Extra Static Base Files (e.g. E-stop, pumps)
        if hasattr(config, 'BASE_EXTRA_FILES') and config.BASE_EXTRA_FILES:
             print(f"[GUI] Loading {len(config.BASE_EXTRA_FILES)} extra static base files...")
             
             # Calculate Scale/Transform for Base (Link 0)
             T_base = np.eye(4)
             try:
                # Attempt to get visual origin of the base link from URDF
                urdf_link_name = self.kinematics.chain.links[0].name
                if urdf_link_name in self.visual_origins:
                    xyz_offset, rpy_offset = self.visual_origins[urdf_link_name]
                    T_origin = np.eye(4)
                    T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                    T_origin[:3, 3] = xyz_offset
                    T_base = T_base @ T_origin
             except Exception as e:
                print(f"[GUI] Warning: could not determine base visual origin: {e}")

             # Apply Global Scene Matrix (e.g. Flip Y)
             T_final = config.S_MATRIX @ T_base
             
             for extra_file in config.BASE_EXTRA_FILES:
                  mesh_item, _ = self._create_mesh_item_from_file(extra_file)
                  
                  # Check for custom offset
                  current_T = T_final.copy()
                  if hasattr(config, 'BASE_EXTRA_OFFSETS') and extra_file in config.BASE_EXTRA_OFFSETS:
                       off_x, off_y, off_z = config.BASE_EXTRA_OFFSETS[extra_file]
                       # Create translation matrix
                       T_off = np.eye(4)
                       T_off[:3, 3] = [off_x, off_y, off_z]
                       # Apply offset to the transform
                       current_T = current_T @ T_off
                       
                  mesh_item.setTransform(matrix_to_qtransform(current_T))
                  self.view.addItem(mesh_item)

        print(f"Loaded {len(self.links_3d)} kinetic meshes.")


    def swap_tool_mesh(self, tool_name):
        filename = config.TOOL_STL_MAP.get(tool_name)
        if not filename:
            print(f"[GUI] No mesh defined for {tool_name}")
            return

        print(f"[GUI] Loading tool mesh: {filename}")
        
        TOOL_INDEX = 6 
        
        if len(self.links_3d) > TOOL_INDEX:
            old_mesh = self.links_3d[TOOL_INDEX]
            self.view.removeItem(old_mesh)
        
        mesh_item, vc = self._create_mesh_item_from_file(filename)
            
        self.view.addItem(mesh_item)
        self.links_3d[TOOL_INDEX] = mesh_item
        self.default_link_colors[TOOL_INDEX] = vc


    def _connect_gui_signals(self):
        self.set_xyz_btn.clicked.connect(self.schedule_update)
        self.set_rpy_btn.clicked.connect(self.schedule_update)
        for inp in self.xyz_inputs.values():
            inp.returnPressed.connect(self.schedule_update)
        for inp in self.rpy_inputs.values():
            inp.returnPressed.connect(self.schedule_update)

        for i in range(6):
            self.joint_sliders[i].valueChanged.connect(
                functools.partial(self.update_slider_label, joint_index=i)
            )
            self.joint_sliders[i].sliderReleased.connect(self.perform_update_from_sliders)

        # Right Panel Buttons
        self.home_button.clicked.connect(self.on_home_click) 
        
        self.standby_btn.clicked.connect(self.go_standby)
        self.stop_btn.clicked.connect(self.on_stop_click) 
        self.camera_btn.clicked.connect(self.reset_camera)
        
        self.safety_btn.clicked.connect(self.on_safety_click)
        self.tool_btn.clicked.connect(self.on_tool_change_click)
        
        # >>> NEW SIGNAL CONNECTIONS <<<
        self.pneumatic_btn.clicked.connect(self.on_pneumatic_click)
        self.electric_btn.clicked.connect(self.on_electric_click)
        
        self.run_program_btn.clicked.connect(self.on_run_program)
        self.stop_program_btn.clicked.connect(self.on_stop_program)

    def init_worker(self):
        self.worker_thread = QThread()
        self.worker = RobotWorker(self.standby_angles, self.kinematics)
        self.worker.moveToThread(self.worker_thread)
        
        self.change_tool_signal.connect(self.worker.change_active_tool)
        self.worker.tool_changed.connect(self.on_tool_changed_feedback)

        self.worker.angles_updated.connect(self.apply_transforms_from_angles)
        self.worker.angles_updated.connect(self.update_joint_value_labels) 
        self.worker.angles_updated.connect(self.update_sliders_from_angles)
        
        self.worker.status_updated.connect(self.update_status_label)
        self.worker.limit_switch_status.connect(self.on_limit_switch_hit)
        
        self.worker.program_finished.connect(self.on_program_finished)
        self.worker.program_line_highlight.connect(self.on_program_line_highlight)
        
        if hasattr(self.worker, 'estop_status_signal'):
            self.worker.estop_status_signal.connect(self.on_estop_signal)

        self.start_move_signal.connect(self.worker.start_move)
        self.start_linear_move_signal.connect(self.worker.start_linear_move)
        self.set_port_signal.connect(self.worker.set_com_port)
        self.request_homing_signal.connect(self.worker.start_homing)
        self.request_gripper_signal.connect(self.worker.set_gripper_state)
        
        # Connect Speed Signal
        if hasattr(self.worker, 'set_speed_multiplier'):
            self.set_speed_signal.connect(self.worker.set_speed_multiplier)
        else:
            print("WARNING: Worker has no set_speed_multiplier method!")
        
        self.start_program_signal.connect(self.worker.start_program)
        self.stop_program_signal.connect(self.worker.stop_program)
        
        self.port_combo.currentTextChanged.connect(self.set_port_signal.emit)
        self.port_combo.currentTextChanged.connect(self.on_port_changed)

        current_port = self.port_combo.currentText()
        self.set_port_signal.emit(current_port)
        self.on_port_changed(current_port) # <--- Call manually to set status at start
        
        self.worker_thread.start()
        self.set_port_signal.emit(self.port_combo.currentText())

        self.active_cyclic_code = None  # Here we store the code (e.g., "E2") if active
        
        self.diag_timer = QTimer(self)
        self.diag_timer.timeout.connect(self.on_diagnostic_tick)
        self.diag_timer.start(500)  # Time in milliseconds (500ms)

    def closeEvent(self, event):
        print("Closing application...")
        self.stop_program_signal.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(2000)
        comm.close_serial_port()
        event.accept()


    def _check_homed_safe(self):
        """Returns True if robot is homed, otherwise shows error."""
        if not self.is_homed:
            self.update_status_label("BLOCKED: Robot not homed! Press HOME first.", "red")
            print("[GUI] Movement blocked - Robot not homed.")
            self.send_diagnostic_code("E1")
            return False
        return True
    
    def on_port_changed(self, port_name):
        """Handles visual status change under COM port."""
        if port_name == "No ports" or not port_name:
            self.uart_status_label.setText("DISCONNECTED")
            # Gray/Red style
            self.uart_status_label.setStyleSheet("color: #777; font-weight: bold; font-size: 11px;")
            self.send_diagnostic_code("E4")
        else:
            self.uart_status_label.setText("CONNECTED")
            # Turquoise/Green style
            self.uart_status_label.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 11px;")

    # ------------------------
    # Slots
    # ------------------------

    @pyqtSlot(bool)
    def on_estop_signal(self, is_active):
        if is_active:
            self.estop_overlay.raise_()
            self.estop_overlay.show()
            self.control_widget.setEnabled(False)
            self.set_cyclic_error("E2")
        else:
            self.estop_overlay.hide()
            self.control_widget.setEnabled(True)
            self.clear_cyclic_error()

    @pyqtSlot(np.ndarray)
    def apply_transforms_from_angles(self, angles):
        if not self.links_3d or len(self.links_3d) != len(config.LINK_NAMES):
            return

        transforms = self.kinematics.forward_kinematics_full(angles)
        MANUAL_INDICES = [0, 1, 2, 3, 4, 5, 6]
        
        for i, config_name in enumerate(config.LINK_NAMES):
            mesh = self.links_3d[i]
            if i < len(MANUAL_INDICES):
                target_idx = MANUAL_INDICES[i]
            else:
                continue

            if target_idx >= len(transforms): continue

            T = transforms[target_idx]
            
            try:
                urdf_link_name = self.kinematics.chain.links[target_idx].name
                if urdf_link_name in self.visual_origins:
                    xyz_offset, rpy_offset = self.visual_origins[urdf_link_name]
                    T_origin = np.eye(4)
                    T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                    T_origin[:3, 3] = xyz_offset
                    T = T @ T_origin
            except Exception:
                pass

            T_scene = config.S_MATRIX @ T
            mesh.setTransform(matrix_to_qtransform(T_scene))
        
        self.view.update()

    def read_target_pose_inputs(self):
        X_user_mm = parse_float_from_input(self.xyz_inputs['X'], default_val=0.0)
        Y_user_mm = parse_float_from_input(self.xyz_inputs['Y'], default_val=0.0)
        Z_user_mm = parse_float_from_input(self.xyz_inputs['Z'], default_val=250.0) 

        X_robot_m =  X_user_mm / 1000.0
        Y_robot_m =  Y_user_mm / 1000.0
        Z_robot_m =  Z_user_mm / 1000.0
            
        return np.array([X_robot_m, Y_robot_m, Z_robot_m])

    @pyqtSlot(str, str)
    def update_status_label(self, text, color):
        """
        Updates ONLY bottom banner.
        Does not touch the label under COM port (it should only show CONNECTED/DISCONNECTED).
        """
        # 1. HomingDialog logic (unchanged)
        if self.homing_dialog and self.homing_dialog.isVisible():
            clean_text = text.strip().upper()
            if len(clean_text) >= 2 and clean_text[1].isdigit():
                code = clean_text[0]
                try: axis_idx = int(clean_text[1]) - 1
                except ValueError: axis_idx = -1
                if 0 <= axis_idx <= 5:
                    if code == 'H': self.homing_dialog.set_axis_state(axis_idx, 'HIT'); return
                    elif code == 'R': self.homing_dialog.set_axis_state(axis_idx, 'DONE'); return

        # 2. Message filter (unchanged)
        ALLOWED_KEYWORDS = ["HOMING", "HOME", "MOVE", "MOVING", "STOP", "ESTOP", 
                            "READY", "STANDBY", "ERROR", "LIMIT", "VACUUM", "GRIP", "COMPLETE", "FINISH"]
        
        is_relevant = any(keyword in text.upper() for keyword in ALLOWED_KEYWORDS)
        if not is_relevant: return

        # 3. REMOVED: self.uart_status_label.setText(text)  <-- THIS IS GONE!

        # 4. Bottom banner update
        bg_color = "#333333"; text_color = "#ffffff"; border_color = "#555555"
        if "red" in color: bg_color = "#c62828"; border_color = "#ff8a80"; text = f"⚠️ {text.upper()} ⚠️"
        elif "orange" in color: bg_color = "#ef6c00"; border_color = "#ffe0b2"
        elif "green" in color: bg_color = "#2e7d32"; border_color = "#a5d6a7"
        elif "blue" in color or "cyan" in color: bg_color = "#0277bd"; border_color = "#81d4fa"

        try:
            self.status_banner.setText(text)
            self.status_banner.setStyleSheet(f"""
                QLabel {{
                    background-color: {bg_color}; color: {text_color};
                    border: 2px solid {border_color}; border-radius: 8px;
                    font-weight: bold; font-size: 13px; padding: 5px;
                }}
            """)
        except AttributeError: pass

        if "HOMING_COMPLETE_OK" in text:
            self.is_homed = True 
            if self.homing_dialog:
                QtCore.QTimer.singleShot(1000, lambda: self.homing_dialog.accept() if self.homing_dialog else None)
    def update_sliders_from_angles(self, angles_rad):
        if len(angles_rad) < 7: offset = 0
        else: offset = 1 
        
        try:
            fk_matrix = self.kinematics.forward_kinematics(angles_rad)
            self.update_gui_feedback(fk_matrix)
        except Exception as e:
            pass

        for i in range(6):
            if self.joint_sliders[i].isSliderDown():
                continue

            if i + offset < len(angles_rad):
                rad_val = angles_rad[i+offset]
                slider_val = self.map_rad_to_slider(rad_val, i)
                
                self.joint_sliders[i].blockSignals(True)
                self.joint_sliders[i].setValue(slider_val)
                self.joint_sliders[i].blockSignals(False)
                self.joint_labels[i].setText(f"{np.degrees(rad_val):.1f}°")

    @pyqtSlot(np.ndarray)
    def update_joint_value_labels(self, angles_rad):
        if not self.joint_value_labels: return
        for i in range(6):
            deg = np.degrees(angles_rad[i])
            self.joint_value_labels[i].setText(f"{deg:.1f}°")

    def update_rpy_inputs_from_matrix(self, matrix):
        try:
            # 1. Calculate RPY from FK matrix (canonical)
            r = R.from_matrix(matrix[:3, :3])
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                roll_calc, pitch_calc, yaw_calc = r.as_euler('xyz', degrees=True) 
                
            # --- FIDELITY STABILIZATION (FLICKER FIX) ---
            if abs(abs(roll_calc) - 180.0) < 0.1: roll_calc = 180.0
            if abs(abs(pitch_calc) - 180.0) < 0.1: pitch_calc = 180.0
            if abs(abs(yaw_calc) - 180.0) < 0.1: yaw_calc = 180.0
            
            if abs(roll_calc) < 0.01: roll_calc = 0.0
            if abs(pitch_calc) < 0.01: pitch_calc = 0.0
            if abs(yaw_calc) < 0.01: yaw_calc = 0.0

            # 2. Check what is currently entered in fields (User Preference)
            try:
                curr_r = parse_float_from_input(self.rpy_inputs['Roll'])
                curr_p = parse_float_from_input(self.rpy_inputs['Pitch'])
                curr_y = parse_float_from_input(self.rpy_inputs['Yaw'])
                
                # Build matrix from what User entered
                r_user = R.from_euler('xyz', [curr_r, curr_p, curr_y], degrees=True)
                matrix_user = r_user.as_matrix()
                
                # Compare User matrix with Robot matrix (FK)
                # Using Frobenius norm of matrix difference
                diff = np.linalg.norm(matrix[:3, :3] - matrix_user)
                
                # If difference is negligible (< 0.1 i.e. very close),
                # it means orientation is the SAME, just representation differs.
                # Then DO NOT OVERWRITE fields to avoid annoying user (maintain 0,-90,180)
                if diff < 0.1:
                    # Update ONLY TCP labels (read-only), leave Inputs
                    if self.tcp_value_labels:
                        self.tcp_value_labels["A"].setText(f"{roll_calc:.1f}")
                        self.tcp_value_labels["B"].setText(f"{pitch_calc:.1f}")
                        self.tcp_value_labels["C"].setText(f"{yaw_calc:.1f}")
                    return 

            except Exception:
                pass

            # 3. If difference is large (robot physically elsewhere), overwrite fields
            for inp in self.rpy_inputs.values(): inp.blockSignals(True)
            self.rpy_inputs['Roll'].setText(f"{roll_calc:.2f}")
            self.rpy_inputs['Pitch'].setText(f"{pitch_calc:.2f}")
            self.rpy_inputs['Yaw'].setText(f"{yaw_calc:.2f}")
            
            if self.tcp_value_labels:
                self.tcp_value_labels["A"].setText(f"{roll_calc:.1f}")
                self.tcp_value_labels["B"].setText(f"{pitch_calc:.1f}")
                self.tcp_value_labels["C"].setText(f"{yaw_calc:.1f}")
                
        except Exception: pass
        finally:
            for inp in self.rpy_inputs.values(): inp.blockSignals(False)

    def update_gui_feedback(self, fk_matrix):
        """Updates text fields and TCP display."""
        self.update_rpy_inputs_from_matrix(fk_matrix)
        pos_robota_m = fk_matrix[:3, 3]
        
        X_user_m =  pos_robota_m[0]
        Y_user_m =  pos_robota_m[1]
        Z_user_m =  pos_robota_m[2]
            
        for inp in self.xyz_inputs.values(): inp.blockSignals(True)
        self.xyz_inputs['X'].setText(f"{X_user_m * 1000.0:.1f}")
        self.xyz_inputs['Y'].setText(f"{Y_user_m * 1000.0:.1f}")
        self.xyz_inputs['Z'].setText(f"{Z_user_m * 1000.0:.1f}")
        for inp in self.xyz_inputs.values(): inp.blockSignals(False)
        
        if self.tcp_value_labels:
            self.tcp_value_labels["X"].setText(f"{X_user_m * 1000.0:.1f}")
            self.tcp_value_labels["Y"].setText(f"{Y_user_m * 1000.0:.1f}")
            self.tcp_value_labels["Z"].setText(f"{Z_user_m * 1000.0:.1f}")

    def perform_update_robot(self):
        if not self._check_homed_safe():
            return
        target_pos_m = self.read_target_pose_inputs()
        roll_deg = parse_float_from_input(self.rpy_inputs['Roll'])
        pitch_deg = parse_float_from_input(self.rpy_inputs['Pitch'])
        yaw_deg = parse_float_from_input(self.rpy_inputs['Yaw'])
        
        try:
            rot = R.from_euler('xyz', [np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)])
            R_mat = rot.as_matrix()
            target_transform = np.eye(4)
            target_transform[:3, :3] = R_mat
            target_transform[:3, 3] = target_pos_m
        except Exception: return
        
        try:
            start_angles = self.worker.get_previous_angles()
            start_transform = self.kinematics.forward_kinematics(start_angles)
        except Exception: return

        self.start_linear_move_signal.emit(start_transform, target_transform)

    def perform_update_from_sliders(self):
        if not self._check_homed_safe():
            return
        active_angles = []
        for i in range(6):
            slider_val = self.joint_sliders[i].value()
            rad_val = self.map_slider_to_rad(slider_val, i)
            active_angles.append(rad_val)
        
        active_angles_np = np.array(active_angles)
        
        try:
            fk_matrix = self.kinematics.forward_kinematics(active_angles_np)
            if fk_matrix is None or fk_matrix.shape != (4, 4):
                return
            self.current_orientation = fk_matrix.copy()
            self.update_gui_feedback(self.current_orientation)
        except Exception as e:
            print(f"!!! ERROR IN PERFORM_UPDATE !!!: {e}")
        
        angles_for_worker = np.concatenate(([0.0], active_angles_np))
        self.start_move_signal.emit(angles_for_worker)

    def update_slider_label(self, value, joint_index):
        rad_val = self.map_slider_to_rad(value, joint_index)
        self.joint_labels[joint_index].setText(f"{np.degrees(rad_val):.1f}°")

    def schedule_update(self):
        if self.update_pending: return
        self.update_pending = True
        QtCore.QTimer.singleShot(config.DEBOUNCE_MS, self.perform_update_and_clear_pending)

    def perform_update_and_clear_pending(self):
        self.perform_update_robot()
        self.update_pending = False

    def go_standby(self):
        if not self._check_homed_safe():
            return
        target_angles = self.standby_angles.copy()
        fk = self.kinematics.forward_kinematics(target_angles)
        self.current_orientation = fk.copy()
        self.update_gui_feedback(self.current_orientation) 
        self.update_sliders_from_angles(target_angles) 
        self.start_move_signal.emit(target_angles)
        self.update_status_label("Status: Standby", "blue")

    # --- BUTTONS ---

    def on_home_click(self):
        self.homing_dialog = HomingDialog(self)
        self.homing_dialog.show()

    def on_safety_click(self):
        if not self._check_homed_safe():
            return
        target_deg = [0, -50, 70, 90, 0, 0]
        target_rad = np.array([0] + [np.radians(a) for a in target_deg])
        
        fk = self.kinematics.forward_kinematics(target_rad)
        self.update_gui_feedback(fk)
        self.update_sliders_from_angles(target_rad)
        
        self.start_move_signal.emit(target_rad)
        self.update_status_label("Status: SAFETY MOVE", "orange")
        self.send_diagnostic_code("W2")

    def on_tool_change_click(self):
        """Displays graphical tool selection dialog."""
        # Create dialog instance
        dlg = ToolSelectionDialog(self)
        
        # Center window relative to main application window
        # (Calculates parent center and subtracts half of dialog size)
        geo = self.geometry()
        x = geo.x() + (geo.width() - dlg.width()) // 2
        y = geo.y() + (geo.height() - dlg.height()) // 2
        dlg.move(x, y)

        # Connect signal from dialog to main tool change signal
        dlg.tool_selected.connect(lambda name: self.change_tool_signal.emit(name))
        
        # Show window
        dlg.exec_()

    def on_stop_click(self):
        self.request_gripper_signal.emit("STOP")
        self.on_stop_program()
        self.update_status_label("Status: STOP SENT", "red")
        self.send_diagnostic_code("W1")

    def reset_camera(self):
        self.view.setCameraPosition(distance=0.9, elevation=30, azimuth=45)

    def on_tool_changed(self, tool_name):
        print(f"GUI: Tool change confirmed: {tool_name}")

    @pyqtSlot(str)
    def on_tool_changed_feedback(self, tool_name):
        print(f"[GUI] Tool changed to: {tool_name}")
        self.tool_btn.setText(f"TOOL: {tool_name}")
        self.update_status_label(f"Changed to: {tool_name}", "cyan")
        self.swap_tool_mesh(tool_name)
        self.perform_update_from_sliders()

    # >>> NEW METHODS FOR GRIPPERS <<<

    @pyqtSlot()
    def on_pneumatic_click(self):
        # Toggle state (Pneumatic: ON/OFF)
        self.pneumatic_active = not self.pneumatic_active
        
        if self.pneumatic_active:
            # STATE: ON (Suction)
            self.pneumatic_btn.setText("VACUUM: ON")
            if HAS_QTA:
                # Full circle / target icon (symbolizes suction)
                self.pneumatic_btn.setIcon(qta.icon("fa5s.dot-circle", color='white'))
            
            # >>> SENDING VGripON COMMAND <<<
            self.request_gripper_signal.emit("VGripON")
        else:
            # STATE: OFF (Idle)
            self.pneumatic_btn.setText("VACUUM: OFF")
            if HAS_QTA:
                # Empty circle icon
                self.pneumatic_btn.setIcon(qta.icon("fa5s.circle", color='white'))
            
            # >>> SENDING VGripOFF COMMAND <<<
            self.request_gripper_signal.emit("VGripOFF")

    @pyqtSlot()
    def on_electric_click(self):
        # Toggle state (Electric: CLOSE/OPEN)
        self.electric_active = not self.electric_active 
        
        if self.electric_active:
            # STATE: CLOSED (Grip)
            self.electric_btn.setText("E-GRIP: CLOSE")
            if HAS_QTA:
                # Clenched fist icon (Rock)
                self.electric_btn.setIcon(qta.icon("fa5s.hand-rock", color='white'))
            
            # >>> SENDING EGRIP_CLOSE COMMAND <<<
            self.request_gripper_signal.emit("EGRIP_CLOSE")
        else:
            # STATE: OPEN (Release)
            self.electric_btn.setText("E-GRIP: OPEN")
            if HAS_QTA:
                # Open hand icon (Paper)
                self.electric_btn.setIcon(qta.icon("fa5s.hand-paper", color='white'))
            
            # >>> SENDING EGRIP_OPEN COMMAND <<<
            self.request_gripper_signal.emit("EGRIP_OPEN")



    def set_link_color(self, link_index_3d, color_rgb):
        try:
            target_mesh = self.links_3d[link_index_3d]
            current_vertexes = target_mesh.vertexes
            current_faces = target_mesh.faces
            if current_vertexes is None: return
            num_verts = current_vertexes.shape[0]
            
            if isinstance(color_rgb, list) or isinstance(color_rgb, np.ndarray):
                new_color_array = np.tile(color_rgb, (num_verts, 1)).astype(np.float32)
                target_mesh.setMeshData(
                    vertexes=current_vertexes,
                    faces=current_faces,
                    vertexColors=new_color_array
                )
        except Exception: return

    @pyqtSlot(int, bool)
    def on_limit_switch_hit(self, joint_index, is_hit):
        if self.homing_dialog and self.homing_dialog.isVisible():
            if is_hit:
                self.homing_dialog.set_axis_state(joint_index, 'HIT')
            else:
                self.homing_dialog.set_axis_state(joint_index, 'DONE')
        else:
            # If not in Homing and limit switch hit -> ERROR
            if is_hit:
                 self.update_status_label(f"LIMIT SWITCH HIT: J{joint_index+1}", "red")
                 self.set_cyclic_error("E5") 
            else:
                 self.clear_cyclic_error()


    def on_diagnostic_tick(self):
        """
        This function is called automatically every 500ms.
        If active error set, send it.
        """
        if self.active_cyclic_code:
            # Using your fixed send_diagnostic_code function
            self.send_diagnostic_code(self.active_cyclic_code)

    def set_cyclic_error(self, code):
        """Enables cyclic sending of given code."""
        self.active_cyclic_code = code
        # Send immediately first time to avoid waiting 500ms
        self.send_diagnostic_code(code)

    def clear_cyclic_error(self):
        """Stops cyclic sending."""
        self.active_cyclic_code = None
    # ---------------------------------------------------------
    # FIXED METHOD: SENDING CODES (Takes port from GUI)
    # ---------------------------------------------------------
    def send_diagnostic_code(self, code):
        """
        Sends error code (e.g. E1, W1) to active COM port.
        Gets port name from selection list in GUI.
        """
        if not code: return
        
        # 1. Get currently selected port from list (e.g. "COM3")
        current_port = self.port_combo.currentText()
        
        # 2. Check if port is valid
        if not current_port or current_port == "No ports":
            print(f"[DIAGNOSTIC SKIPPED] Code '{code}' not sent - no port selected.")
            return

        print(f"[DIAGNOSTIC LOG] Sending '{code}' to port '{current_port}'...")

        try:
            # 3. Function call with two arguments: (PORT, COMMAND)
            # Your communication library requires port as first argument.
            if hasattr(comm, 'send_command'):
                comm.send_command(current_port, code)
            elif hasattr(comm, 'send_message'):
                 comm.send_message(current_port, code)
            else:
                print("[UART ERROR] 'comm' module has no known send function.")
                
        except Exception as e:
            # Catch errors to prevent GUI close on cable issues
            print(f"[UART EXCEPTION] Error sending '{code}': {e}")
    # ---------------------------------------------------------
    # SYNTAX VALIDATOR (NEW FUNCTION)
    # ---------------------------------------------------------
    def validate_program_syntax(self):
        """Checks syntax in editor and highlights error lines in red."""
        text = self.program_editor.toPlainText()
        lines = text.split('\n')
        selections = []
        
        # Valid pattern definitions (Regex)
        # Number (integer or float, optional minus): -?\d+(\.\d+)?
        FLOAT_PTN = r"-?\d+(?:\.\d+)?"
        # MOVE(x,y,z) or MOVEL(x,y,z) - spaces allowed
        MOVE_PTN = rf"^(MOVE|MOVEL)\s*\(\s*{FLOAT_PTN}\s*,\s*{FLOAT_PTN}\s*,\s*{FLOAT_PTN}\s*\)$"
        # DELAY(ms)
        DELAY_PTN = rf"^DELAY\s*\(\s*\d+\s*\)$"
        # EGRIP(OPEN) lub EGRIP(CLOSE)
        EGRIP_PTN = r"^EGRIP\s*\(\s*(OPEN|CLOSE)\s*\)$"
        # Simple commands
        SIMPLE_CMD = ["MOVESTB", "VACON", "VACOFF"]

        for i, line in enumerate(lines):
            line_str = line.strip().upper() # Ignorujemy wielkość liter i spacje na bokach
            
            # 1. Ignore empty lines and comments
            if not line_str or line_str.startswith("#"):
                continue

            is_valid = False
            
            # 2. Checking simple commands
            if line_str in SIMPLE_CMD:
                is_valid = True
            
            # 3. Checking regexes (MOVE, DELAY, etc.)
            elif re.match(MOVE_PTN, line_str):
                is_valid = True
            elif re.match(DELAY_PTN, line_str):
                is_valid = True
            elif re.match(EGRIP_PTN, line_str):
                is_valid = True

            # 4. If line is INVALID -> add red background
            if not is_valid:
                selection = QtWidgets.QTextEdit.ExtraSelection()
                selection.format.setBackground(QColor(80, 0, 0)) # Dark red background
                selection.format.setUnderlineStyle(QtGui.QTextCharFormat.WaveUnderline) # Wavy underline
                selection.format.setUnderlineColor(QColor(255, 0, 0)) # Light red stroke
                selection.format.setProperty(QTextFormat.FullWidthSelection, True)
                
                cursor = self.program_editor.textCursor()
                cursor.movePosition(QtGui.QTextCursor.Start)
                cursor.movePosition(QtGui.QTextCursor.Down, QtGui.QTextCursor.MoveAnchor, i)
                selection.cursor = cursor
                selections.append(selection)

        # Apply selections (doesn't remove text, only changes formatting)
        self.program_editor.setExtraSelections(selections)

    @pyqtSlot()
    def on_run_program(self):
        if not self._check_homed_safe():
            return
        if self.program_editor.extraSelections():
            self.update_status_label("Cannot Run: Syntax Errors Detected!", "red")
            self.send_diagnostic_code("E3")
            return
        program_text = self.program_editor.toPlainText()
        if not program_text.strip(): return
        self.run_program_btn.setEnabled(False)
        self.stop_program_btn.setEnabled(True)
        self.tab_widget.setCurrentIndex(0) 
        self.update_status_label("Status: Running program...", "blue")
        self.start_program_signal.emit(program_text)

    @pyqtSlot()
    def on_stop_program(self):
        self.stop_program_signal.emit()
        self.update_status_label("Status: Stopping...", "orange")
        self.send_diagnostic_code("W1")

    @pyqtSlot(str, bool)
    def on_program_finished(self, status_msg, was_stopped):
        self.run_program_btn.setEnabled(True)
        self.stop_program_btn.setEnabled(False)
        self.tab_widget.setCurrentIndex(1)
        self.update_status_label(f"Status: {status_msg}", "orange" if was_stopped else "green")
        self.on_program_line_highlight(-1)

    @pyqtSlot(int)
    def on_program_line_highlight(self, line_number):
        cursor = self.program_editor.textCursor()
        selection = QtWidgets.QTextEdit.ExtraSelection()
        selection.format.setBackground(QtCore.Qt.transparent)
        selection.cursor = cursor
        selections = [selection]

        if line_number >= 0:
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.movePosition(QtGui.QTextCursor.Down, QtGui.QTextCursor.MoveAnchor, line_number)
            selection = QtWidgets.QTextEdit.ExtraSelection()
            selection.format.setBackground(QColor(60, 60, 40))
            selection.format.setProperty(QTextFormat.FullWidthSelection, True)
            selection.cursor = cursor
            selections.append(selection)

        self.program_editor.setExtraSelections(selections)