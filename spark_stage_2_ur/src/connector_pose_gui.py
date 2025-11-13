#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QLabel, QSpinBox, QPushButton, 
                                 QTextEdit, QGroupBox)
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QFont
except ImportError:
    print("PyQt5 not found, trying PyQt6...")
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QLabel, QSpinBox, QPushButton, 
                                 QTextEdit, QGroupBox)
    from PyQt6.QtCore import QTimer, Qt
    from PyQt6.QtGui import QFont


class ConnectorPoseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = rclpy.create_node('connector_pose_gui')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        # Use source directory instead of install directory
        try:
            # Try to get package path from ament_index (install directory)
            install_dir = get_package_share_directory('spark_stage_2_ur')
            # Navigate to source directory from install
            src_dir = Path(install_dir).parent.parent.parent.parent / 'src' / 'spark_stage_2_ur'
            self.config_file = src_dir / 'config' / 'connector_poses.yaml'
            
            if not self.config_file.exists():
                # Fallback: try install directory
                self.config_file = Path(install_dir) / 'config' / 'connector_poses.yaml'
        except:
            # If ament_index fails, use relative path from workspace
            self.config_file = Path.cwd() / 'src' / 'spark_stage_2_ur' / 'config' / 'connector_poses.yaml'
        
        self.initUI()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_current_pose)
        self.update_timer.start(500)
        
    def initUI(self):
        self.setWindowTitle('Connector Pose Saver')
        self.setGeometry(100, 100, 600, 550)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # File location display
        file_info = QLabel(f"Saving to: {self.config_file}")
        file_info.setStyleSheet("QLabel { color: gray; font-size: 9px; }")
        file_info.setWordWrap(True)
        layout.addWidget(file_info)
        
        # Row and Column inputs
        input_group = QGroupBox("Connector Position")
        input_layout = QHBoxLayout()
        
        row_label = QLabel("Row:")
        self.row_spin = QSpinBox()
        self.row_spin.setMinimum(1)
        self.row_spin.setMaximum(3)
        self.row_spin.setValue(1)
        self.row_spin.setMinimumWidth(80)
        
        col_label = QLabel("Column:")
        self.col_spin = QSpinBox()
        self.col_spin.setMinimum(1)
        self.col_spin.setMaximum(16)
        self.col_spin.setValue(1)
        self.col_spin.setMinimumWidth(80)
        
        input_layout.addWidget(row_label)
        input_layout.addWidget(self.row_spin)
        input_layout.addWidget(col_label)
        input_layout.addWidget(self.col_spin)
        input_layout.addStretch()
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)
        
        # Current pose display
        pose_group = QGroupBox("Current Tool Pose (base_link → tool0)")
        pose_layout = QVBoxLayout()
        self.current_pose_label = QLabel("Position: --\nOrientation: --")
        self.current_pose_label.setFont(QFont("Monospace", 9))
        pose_layout.addWidget(self.current_pose_label)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)
        
        # Save button
        self.save_button = QPushButton("Save Pose")
        self.save_button.setMinimumHeight(50)
        self.save_button.setStyleSheet("QPushButton { font-size: 16px; font-weight: bold; }")
        self.save_button.clicked.connect(self.save_pose)
        layout.addWidget(self.save_button)
        
        # Status/Log area
        log_group = QGroupBox("Status")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        self.log("GUI Started. Move robot and click 'Save Pose'")
        
    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        
    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time())
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            pose_text = f"Position: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]\n"
            pose_text += f"Orientation: [{r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f}]"
            self.current_pose_label.setText(pose_text)
            
        except Exception as e:
            self.current_pose_label.setText("Position: --\nOrientation: --")
    
    def save_pose(self):
        row = self.row_spin.value()
        col = self.col_spin.value()
        
        try:
            # Get current transform
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time())
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            # Load existing config
            if self.config_file.exists():
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
            else:
                config = {'control_nodes': [], 'control_cols': [1, 4, 8, 12, 16]}
            
            # Update or add node
            node_data = {
                'row': row, 
                'col': col,
                'x': round(float(t.x), 3),
                'y': round(float(t.y), 3),
                'z': round(float(t.z), 3),
                'qx': round(float(r.x), 3),
                'qy': round(float(r.y), 3),
                'qz': round(float(r.z), 3),
                'qw': round(float(r.w), 3)
            }
            
            # Check if node exists and update, otherwise append
            found = False
            for i, node in enumerate(config['control_nodes']):
                if node['row'] == row and node['col'] == col:
                    config['control_nodes'][i] = node_data
                    found = True
                    break
            
            if not found:
                config['control_nodes'].append(node_data)
            
            # Sort by row then col
            config['control_nodes'].sort(key=lambda x: (x['row'], x['col']))
            
            # Save to file
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=None, sort_keys=False)
            
            self.log(f"✓ Saved pose ({row},{col}): [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]")
            self.log(f"  File: {self.config_file}")
            self.log(f"  Copy-paste: " + 
                    f"{{row: {row}, col: {col}, x: {t.x:.3f}, y: {t.y:.3f}, z: {t.z:.3f}, " +
                    f"qx: {r.x:.3f}, qy: {r.y:.3f}, qz: {r.z:.3f}, qw: {r.w:.3f}}}")
            
            # Auto-increment to next column in control columns
            control_cols = [1, 4, 8, 12, 16]
            if col in control_cols:
                next_idx = control_cols.index(col) + 1
                if next_idx < len(control_cols):
                    self.col_spin.setValue(control_cols[next_idx])
                else:
                    # Move to next row, first column
                    if row < 3:
                        self.row_spin.setValue(row + 1)
                        self.col_spin.setValue(1)
            
        except Exception as e:
            self.log(f"✗ Error: {str(e)}")
    
    def log(self, message):
        self.log_text.append(message)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = ConnectorPoseGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
