#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rqt PID Tuner Plugin

This plugin provides a Qt-based interface for LLM-based PID auto-tuning.
"""

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QLabel
from python_qt_binding.QtCore import Qt, QTimer

from rm_pid_tuner.msg import TuningStatus, TuningLog, PidParams
from rm_pid_tuner.srv import StartTuning, StopTuning, SetPidParams


class PidTunerPlugin(Plugin):
    """rqt Plugin for PID Auto-Tuning"""

    def __init__(self, context):
        super(PidTunerPlugin, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('PidTunerPlugin')

        # Create widget
        self._widget = QWidget()

        # Load UI file
        ui_file = os.path.join(
            rospkg.RosPack().get_path('rm_pid_tuner'),
            'plugins', 'rqt_pid_tuner', 'resource', 'pid_tuner.ui'
        )

        # If UI file doesn't exist, create a simple layout
        if os.path.exists(ui_file):
            loadUi(ui_file, self._widget)
        else:
            self._create_simple_ui()

        # Add widget to context
        if context.serializable() > 1:
            self._widget.setWindowTitle('PID Tuner')
        context.add_widget(self._widget)

        # Initialize ROS interface
        self._init_ros_interface()

        # Timer for updates
        self._timer = QTimer()
        self._timer.timeout.connect(self._update_status)
        self._timer.start(100)  # 10 Hz

    def _create_simple_ui(self):
        """Create a simple UI if .ui file is not available"""
        from python_qt_binding.QtWidgets import (
            QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
            QSpinBox, QDoubleSpinBox, QCheckBox
        )

        layout = QVBoxLayout()

        # Controller selection
        controller_group = QGroupBox("Controller")
        controller_layout = QFormLayout()

        self.controller_combo = QComboBox()
        self.controller_combo.addItems(['gimbal_controller', 'chassis_controller'])
        controller_layout.addRow("Controller:", self.controller_combo)

        self.joint_combo = QComboBox()
        self.joint_combo.addItems(['yaw', 'pitch'])
        controller_layout.addRow("Joint:", self.joint_combo)

        controller_group.setLayout(controller_layout)
        layout.addWidget(controller_group)

        # PID Parameters
        params_group = QGroupBox("PID Parameters")
        params_layout = QFormLayout()

        self.kp_slider = QSlider(Qt.Horizontal)
        self.kp_slider.setRange(0, 1000)
        self.kp_slider.setValue(100)
        self.kp_label = QLabel("1.0")
        kp_layout = QHBoxLayout()
        kp_layout.addWidget(self.kp_slider)
        kp_layout.addWidget(self.kp_label)
        params_layout.addRow("Kp:", kp_layout)

        self.ki_slider = QSlider(Qt.Horizontal)
        self.ki_slider.setRange(0, 1000)
        self.ki_slider.setValue(10)
        self.ki_label = QLabel("0.1")
        ki_layout = QHBoxLayout()
        ki_layout.addWidget(self.ki_slider)
        ki_layout.addWidget(self.ki_label)
        params_layout.addRow("Ki:", ki_layout)

        self.kd_slider = QSlider(Qt.Horizontal)
        self.kd_slider.setRange(0, 1000)
        self.kd_slider.setValue(5)
        self.kd_label = QLabel("0.05")
        kd_layout = QHBoxLayout()
        kd_layout.addWidget(self.kd_slider)
        kd_layout.addWidget(self.kd_label)
        params_layout.addRow("Kd:", kd_layout)

        params_group.setLayout(params_layout)
        layout.addWidget(params_group)

        # Status
        status_group = QGroupBox("Status")
        status_layout = QFormLayout()

        self.status_label = QLabel("IDLE")
        status_layout.addRow("Status:", self.status_label)

        self.round_label = QLabel("0 / 30")
        status_layout.addRow("Round:", self.round_label)

        self.error_label = QLabel("0.00")
        status_layout.addRow("Avg Error:", self.error_label)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Buttons
        button_layout = QHBoxLayout()

        self.start_button = QPushButton("Start Tuning")
        self.start_button.clicked.connect(self._start_tuning)
        button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Tuning")
        self.stop_button.clicked.connect(self._stop_tuning)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)

        self.apply_button = QPushButton("Apply Params")
        self.apply_button.clicked.connect(self._apply_params)
        button_layout.addWidget(self.apply_button)

        layout.addLayout(button_layout)

        self._widget.setLayout(layout)

        # Connect slider signals
        self.kp_slider.valueChanged.connect(
            lambda v: self.kp_label.setText(f"{v / 100:.2f}")
        )
        self.ki_slider.valueChanged.connect(
            lambda v: self.ki_label.setText(f"{v / 100:.2f}")
        )
        self.kd_slider.valueChanged.connect(
            lambda v: self.kd_label.setText(f"{v / 100:.2f}")
        )

    def _init_ros_interface(self):
        """Initialize ROS subscribers and service clients"""
        # Subscribers
        self._status_sub = rospy.Subscriber(
            '/pid_tuner/tuning_status',
            TuningStatus,
            self._status_callback
        )

        self._log_sub = rospy.Subscriber(
            '/pid_tuner/tuning_log',
            TuningLog,
            self._log_callback
        )

        # Service clients
        rospy.wait_for_service('/pid_tuner/start_tuning', timeout=2.0)
        self._start_client = rospy.ServiceProxy(
            '/pid_tuner/start_tuning',
            StartTuning
        )

        self._stop_client = rospy.ServiceProxy(
            '/pid_tuner/stop_tuning',
            StopTuning
        )

        self._set_params_client = rospy.ServiceProxy(
            '/pid_tuner/set_params',
            SetPidParams
        )

        # State
        self._current_status = None
        self._is_tuning = False

    def _status_callback(self, msg):
        """Handle tuning status messages"""
        self._current_status = msg
        self._is_tuning = (msg.status == 'TUNING')

    def _log_callback(self, msg):
        """Handle tuning log messages"""
        rospy.loginfo(f"[PID Tuner] Round {msg.round}: {msg.analysis}")

    def _update_status(self):
        """Update UI with current status"""
        if self._current_status:
            self.status_label.setText(self._current_status.status)
            self.round_label.setText(
                f"{self._current_status.round} / {self._current_status.max_rounds}"
            )
            self.error_label.setText(f"{self._current_status.avg_error:.2f}")

            # Update button states
            self.start_button.setEnabled(not self._is_tuning)
            self.stop_button.setEnabled(self._is_tuning)

            # Update sliders from current params
            if self._current_status.current_params:
                params = self._current_status.current_params
                self.kp_slider.setValue(int(params.p * 100))
                self.ki_slider.setValue(int(params.i * 100))
                self.kd_slider.setValue(int(params.d * 100))

    def _start_tuning(self):
        """Start the tuning process"""
        try:
            controller = self.controller_combo.currentText()
            joint = self.joint_combo.currentText()

            response = self._start_client(
                controller_name=controller,
                joint_names=[joint],
                target_setpoint=0.0,
                tolerance=0.3,
                max_rounds=30,
                buffer_size=25,
                conservative_mode=True,
                z_n_gain_factor=0.5,
                auto_save=False
            )

            if response.success:
                rospy.loginfo(f"Started tuning: {controller}/{joint}")
            else:
                rospy.logwarn(f"Failed to start tuning: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _stop_tuning(self):
        """Stop the tuning process"""
        try:
            controller = self.controller_combo.currentText()

            response = self._stop_client(
                controller_name=controller,
                save_params=False,
                restore_params=False
            )

            if response.success:
                rospy.loginfo(f"Stopped tuning after {response.total_rounds} rounds")
            else:
                rospy.logwarn(f"Failed to stop tuning: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _apply_params(self):
        """Apply current slider values as PID parameters"""
        try:
            controller = self.controller_combo.currentText()
            joint = self.joint_combo.currentText()

            p = self.kp_slider.value() / 100.0
            i = self.ki_slider.value() / 100.0
            d = self.kd_slider.value() / 100.0

            params = PidParams()
            params.controller_name = controller
            params.joint_name = joint
            params.p = p
            params.i = i
            params.d = d
            params.i_clamp_max = 0.3
            params.i_clamp_min = -0.3
            params.antiwindup = True

            response = self._set_params_client(
                controller_name=controller,
                params=[params],
                apply_immediately=True,
                save_to_file=False
            )

            if response.success:
                rospy.loginfo(f"Applied params: Kp={p}, Ki={i}, Kd={d}")
            else:
                rospy.logwarn(f"Failed to apply params: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def shutdown_plugin(self):
        """Clean up when plugin is closed"""
        self._timer.stop()
        self._status_sub.unregister()
        self._log_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        """Save plugin settings"""
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore plugin settings"""
        pass
