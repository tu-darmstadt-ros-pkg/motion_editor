import roslib
roslib.load_manifest('vigir_rqt_motion_editor')

import os
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QWidget, QMessageBox

from flor_motion.joint_configuration import appendix_by_name, appendix_names, remove_side_prefix, adapt_to_side
from flor_motion.position_data import PositionData


class PositionEditorWidget(QWidget):
    position_list_updated = Signal(dict)

    def __init__(self, motion_publisher):
        super(PositionEditorWidget, self).__init__()
        self._position_data = PositionData()

        self._motion_publisher = motion_publisher

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'position_editor.ui')
        loadUi(ui_file, self)
        self.save_appendix_combo.addItems(appendix_names)
        self.save_appendix_combo.setCurrentIndex(0)
        self.move_appendix_combo.addItems(appendix_names)
        self.move_appendix_combo.setCurrentIndex(0)
        self.position_list_updated.emit(self._position_data)

    @Slot()
    def on_delete_button_clicked(self):
        appendix_name = str(self.save_appendix_combo.currentText())
        position_name = str(self.save_name_combo.currentText())
        if len(position_name) == 0 or position_name not in self._position_data[remove_side_prefix(appendix_name)]:
            QMessageBox.warning(self, 'Error', 'Position "%s" not in position list.')
            return
        self._position_data[remove_side_prefix(appendix_name)].delete(position_name)
        self.position_list_updated.emit(self._position_data)
        self.on_save_appendix_combo_currentIndexChanged(appendix_name)
        if remove_side_prefix(self.move_appendix_combo.currentText()) == remove_side_prefix(appendix_name):
            self.on_move_appendix_combo_currentIndexChanged(appendix_name)

    @Slot()
    def on_save_button_clicked(self):
        position_name = str(self.save_name_combo.currentText())
        if len(position_name) == 0:
            QMessageBox.warning(self, 'Error', 'No name given to save this position.')
            return
        appendix_name = str(self.save_appendix_combo.currentText())
        appendix = appendix_by_name[appendix_name]
        current_positions = self._motion_publisher.get_current_positions(appendix)
        current_positions = adapt_to_side(appendix_name, current_positions)
        print 'saving %s as "%s": %s' % (appendix_name, position_name, current_positions)
        self._position_data[remove_side_prefix(appendix_name)].save(position_name, current_positions)
        self.position_list_updated.emit(self._position_data)
        self.on_save_appendix_combo_currentIndexChanged(appendix_name)
        if remove_side_prefix(self.move_appendix_combo.currentText()) == remove_side_prefix(appendix_name):
            self.on_move_appendix_combo_currentIndexChanged(appendix_name)

    @Slot(str)
    def on_move_appendix_combo_currentIndexChanged(self, appendix_name):
        # remember selected position name
        position_name = str(self.move_name_combo.currentText())
        # update position names
        self.move_name_combo.clear()
        self.move_name_combo.addItems(sorted(self._position_data[remove_side_prefix(appendix_name)].keys()))
        # restore previously selected position name
        position_index = self.move_name_combo.findText(position_name)
        self.move_name_combo.setCurrentIndex(position_index)

    @Slot(str)
    def on_save_appendix_combo_currentIndexChanged(self, appendix_name):
        # remember selected position name
        position_name = str(self.save_name_combo.currentText())
        # update position names
        self.save_name_combo.clear()
        self.save_name_combo.addItems(sorted(self._position_data[remove_side_prefix(appendix_name)].keys()))
        # restore previously selected position name
        position_index = self.save_name_combo.findText(position_name)
        self.save_name_combo.setCurrentIndex(position_index)

    @Slot()
    def on_move_button_clicked(self):
        position_name = str(self.move_name_combo.currentText())
        if len(position_name) == 0:
            QMessageBox.warning(self, 'Error', 'No position selected to move to.')
            return
        appendix_name = str(self.move_appendix_combo.currentText())
        target_positions = self._position_data[remove_side_prefix(appendix_name)][position_name]
        target_positions = adapt_to_side(appendix_name, target_positions)
        print 'Moving %s to "%s": %s' % (appendix_name, position_name, target_positions)
        self._motion_publisher.move_to_position(appendix_name, target_positions)

    def shutdown(self):
        pass
