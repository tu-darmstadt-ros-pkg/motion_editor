import roslib
roslib.load_manifest('vigir_rqt_motion_editor')
import rospy
import os
import re
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QApplication, QWidget, QListWidgetItem, QMessageBox, QMenu

from flor_motion.joint_configuration import adapt_to_side, appendix_names, appendix_by_name, remove_side_prefix, appendix_types
from flor_motion.motion_data import MotionData
from timeline_widget import TimelineWidget


class MotionEditorWidget(QWidget):

    def __init__(self, motion_publisher):
        super(MotionEditorWidget, self).__init__()
        self._motion_publisher = motion_publisher
        self._motion_data = MotionData()
        self._filter_pattern = ''
        self._playback_marker = None
        self._playback_timer = None

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'motion_editor.ui')
        loadUi(ui_file, self)

        for appendix_type in appendix_types:
            list_widget = getattr(self, '%s_positions_list' % appendix_type)
            list_widget.customContextMenuRequested.connect(getattr(self, '%s_positions_list_context_menu' % appendix_type))

        self._timeline_widget = TimelineWidget()
        for track_name in appendix_names:
            track_type = remove_side_prefix(track_name)
            self._timeline_widget.add_track(track_name, track_type)

        self.timeline_group.layout().addWidget(self._timeline_widget)

        self.update_motion_name_combo()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self.splitter.saveState())
        instance_settings.set_value('filter_pattern', self.filter_pattern_edit.text())
        instance_settings.set_value('filter_motions_checked', self.filter_motions_check.isChecked())
        instance_settings.set_value('filter_positions_checked', self.filter_positions_check.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([300, 100])
        self.filter_pattern_edit.setText(instance_settings.value('filter_pattern', ''))
        self.filter_motions_check.setChecked(instance_settings.value('filter_motions_checked', False) in (1, True, 'true'))
        self.filter_positions_check.setChecked(instance_settings.value('filter_positions_checked', False) in (1, True, 'true'))

    @Slot()
    def on_clear_motion_button_clicked(self):
        self._timeline_widget.scene().clear()

    @Slot()
    def on_delete_motion_button_clicked(self):
        motion_name = str(self.motion_name_combo.currentText())
        if len(motion_name) == 0:
            QMessageBox.warning(self, 'Error', 'No motion selected to delete.')
            return
        if motion_name not in self._motion_data:
            QMessageBox.warning(self, 'Error', 'Motion "%s" not in motion list.' % motion_name)
            return
        self._motion_data.delete(motion_name)
        self.update_motion_name_combo()

    @Slot()
    def on_save_motion_button_clicked(self):
        motion_name = str(self.motion_name_combo.currentText())
        if len(motion_name) == 0:
            QMessageBox.warning(self, 'Error', 'No name given to save this motion.')
            return
        self._motion_data.save(motion_name, self.get_motion_from_timeline())
        self.update_motion_name_combo()

    @Slot()
    def on_load_motion_button_clicked(self):
        motion_name = str(self.motion_name_combo.currentText())
        if len(motion_name) == 0:
            QMessageBox.warning(self, 'Error', 'No motion selected to load.')
            return
        if motion_name not in self._motion_data:
            QMessageBox.warning(self, 'Error', 'Motion "%s" not in motion list.' % motion_name)
            return
        self.on_clear_motion_button_clicked()
        motion = self._motion_data[motion_name]
        for appendix_name, poses in motion.items():
            for pose in poses:
                data = adapt_to_side(appendix_name, pose['positions'])
                self._timeline_widget.scene().add_clip(appendix_name, pose['name'], pose['starttime'], pose['duration'], data)

    @Slot()
    def on_null_motion_button_clicked(self):
        self._clear_playback_marker()
        for appendix_name in appendix_names:
            target_position = [0] * len(appendix_by_name[appendix_name]['joint_names'])
            self._motion_publisher.move_to_position(appendix_name, target_position, self.time_factor_spin.value())

    @Slot()
    def on_run_motion_button_clicked(self):
        motion_name = str(self.motion_name_combo.currentText())
        if len(motion_name) == 0:
            QMessageBox.warning(self, 'Error', 'No motion selected to run.')
            return
        if motion_name not in self._motion_data:
            QMessageBox.warning(self, 'Error', 'Motion "%s" not in motion list.' % motion_name)
            return
        self._clear_playback_marker()
        self._motion_publisher.publish_motion(self._motion_data[motion_name], self.time_factor_spin.value())
        print 'Running motion:', motion_name

    @Slot(str)
    def on_filter_pattern_edit_textChanged(self, pattern):
        self._filter_pattern = pattern
        self._apply_filter_to_position_lists()
        self.update_motion_name_combo()

    def _apply_filter_to_position_lists(self):
        for appendix_type in appendix_types:
            list_widget = getattr(self, '%s_positions_list' % appendix_type)
            for row in range(list_widget.count()):
                item = list_widget.item(row)
                hidden = self.filter_positions_check.isChecked() and re.search(self._filter_pattern, item.text()) is None
                item.setHidden(hidden)

    @Slot(bool)
    def on_filter_positions_check_toggled(self, checked):
        self._apply_filter_to_position_lists()

    @Slot(bool)
    def on_filter_motions_check_toggled(self, checked):
        self.update_motion_name_combo()

    def update_motion_name_combo(self):
        combo = self.motion_name_combo
        # remember selected motion name
        motion_name = str(combo.currentText())
        # update motion names
        combo.clear()
        motion_names = self._motion_data.keys()
        if self.filter_motions_check.isChecked():
            motion_names = [name for name in motion_names if re.search(self._filter_pattern, name) is not None]
        combo.addItems(sorted(motion_names))
        # restore previously selected motion name
        motion_index = combo.findText(motion_name)
        combo.setCurrentIndex(motion_index)

    def update_positions_lists(self, positions):
        for appendix_type in appendix_types:
            list_widget = getattr(self, '%s_positions_list' % appendix_type)
            list_widget.clear()
            for name, position in positions[appendix_type].items():
                item = QListWidgetItem(name)
                item._data = position
                item._text = name
                item._type = appendix_type
                list_widget.addItem(item)
        self._apply_filter_to_position_lists()

    def arm_positions_list_context_menu(self, pos):
        self.positions_list_context_menu('arm', pos)

    def leg_positions_list_context_menu(self, pos):
        self.positions_list_context_menu('leg', pos)

    def torso_positions_list_context_menu(self, pos):
        self.positions_list_context_menu('torso', pos)
        
    def head_positions_list_context_menu(self, pos):
        self.positions_list_context_menu('head', pos)

    def positions_list_context_menu(self, appendix_type, pos):
        list_widget = getattr(self, '%s_positions_list' % appendix_type)
        list_item = list_widget.itemAt(pos)
        if list_item is None:
            return

        menu = QMenu()
        move_to = {}
        for appendix_name in appendix_names:
            if list_item._type in appendix_name:
                move_to[menu.addAction('move "%s"' % appendix_name)] = appendix_name
        result = menu.exec_(list_widget.mapToGlobal(pos))
        if result in move_to:
            appendix_name = move_to[result]
            target_positions = adapt_to_side(appendix_name, list_item._data)
            self._motion_publisher.move_to_position(appendix_name, target_positions, self.time_factor_spin.value())
            print 'Moving %s to: %s' % (appendix_name, target_positions)

    def get_motion_from_timeline(self):
        motion = {}
        for appendix_name, clips in self._timeline_widget.scene().clips().items():
            motion[appendix_name] = []
            for clip in clips:
                pose = {
                    'name': clip.text(),
                    'starttime': clip.starttime(),
                    'duration': clip.duration(),
                    'positions': adapt_to_side(appendix_name, clip.data()),
                }
                motion[appendix_name].append(pose)
        return motion

    @Slot()
    def on_run_timeline_button_clicked(self):
        print 'Running timeline.'
        self._playback_duration = 0.0
        for clips in self._timeline_widget.scene().clips().values():
            if len(clips) > 0 and self._playback_duration < clips[-1].endtime():
                self._playback_duration = clips[-1].endtime()
        self._playback_time_factor = self.time_factor_spin.value()

        self._clear_playback_marker()
        self._playback_marker = self._timeline_widget.scene().add_marker(0.0)

        self._playback_timer = QTimer()
        self._playback_timer.setInterval(30)
        self._playback_timer.timeout.connect(self._playback_update)

        self._playback_start = rospy.get_time()
        self._motion_publisher.publish_motion(self.get_motion_from_timeline(), self.time_factor_spin.value())
        self._playback_timer.start()

    def _clear_playback_marker(self):
        if self._playback_timer is not None:
            self._playback_timer.stop()
        if self._playback_marker is not None:
            self._playback_marker.remove()

    @Slot()
    def _playback_update(self):
        duration = (rospy.get_time() - self._playback_start) / self._playback_time_factor
        if duration > self._playback_duration:
            self._clear_playback_marker()
        else:
            self._playback_marker.set_time(duration)


if __name__ == '__main__':
    import sys
    from flor_motion.motion_publisher import MotionPublisher
    app = QApplication(sys.argv)
    widget = MotionEditorWidget(MotionPublisher())
    positions = {
        'arm': {'test_arm_1': range(6), 'test_arm_2': range(6), },
        'leg': {'test_leg_1': range(6), 'test_leg_2': range(6), },
        'torso': {'test_torso_1': range(3), 'test_torso_2': range(3), },
        'neck': {'test_neck_1': range(1), 'test_neck_2': range(1), },
    }
    widget.update_positions_lists(positions)
    widget.show()
    app.exec_()
