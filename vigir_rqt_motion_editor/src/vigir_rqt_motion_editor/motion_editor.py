import roslib
roslib.load_manifest('vigir_rqt_motion_editor')

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QWidget, QVBoxLayout

from flor_motion.motion_publisher import MotionPublisher

from .position_editor_widget import PositionEditorWidget
from .motion_editor_widget import MotionEditorWidget
from .input_output_selector_widget import InputOutputSelectorWidget


class MotionEditorPlugin(Plugin):
    updateStateSignal = Signal(object)

    def __init__(self, context):
        super(MotionEditorPlugin, self).__init__(context)
        self.setObjectName('MotionEditorPlugin')

        motion_publishers = {'robot': MotionPublisher()}
        motion_publishers['robot'].set_subscriber_prefix('/thor_mang')
        motion_publishers['robot'].set_publisher_prefix('/thor_mang')

        input_output_selector = InputOutputSelectorWidget(motion_publishers)
        self._motion_editor = MotionEditorWidget(input_output_selector)
        position_editor = PositionEditorWidget(input_output_selector)
        position_editor.position_list_updated.connect(self._motion_editor.update_positions_lists)
        position_editor.position_list_updated.emit(position_editor._position_data)

        layout = QVBoxLayout()
        layout.addWidget(input_output_selector)
        layout.addWidget(position_editor)
        layout.addWidget(self._motion_editor)
        self._widget = QWidget()
        self._widget.setLayout(layout)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._motion_editor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._motion_editor.restore_settings(plugin_settings, instance_settings)

    #def shutdown_plugin(self):
        #self._widget.shutdown()
