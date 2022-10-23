#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import cv2
import numpy as np
import dearpygui.dearpygui as dpg

from node_editor.util import dpg_get_value, dpg_set_value

from node.node_abc import DpgNodeABC
from node_editor.util import convert_cv_to_dpg
from node.draw_node.draw_util.draw_util import draw_info

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher(Node):

    ros2_node_name = "ros2_image_output"

    def __init__(self, topic_name):
        node_name = self.ros2_node_name + topic_name
        node_name = node_name.replace('/', '_') 
        super().__init__(node_name)
        self.pub = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        self.get_logger().info("ROS2 node initilize...")

    def __del__(self):
        self.get_logger().info("ROS2 node exit...")


class Node(DpgNodeABC):
    _ver = '0.0.1'

    node_label = 'ROS2'
    node_tag = 'ROS2ImageOuput'

    _opencv_setting_dict = None
    _start_label = 'Start'
    _stop_label = 'Stop'

    ros2_node_list = {}

    def __init__(self):
        pass

    def add_node(
        self,
        parent,
        node_id,
        pos=[0, 0],
        opencv_setting_dict=None,
        callback=None,
    ):
        # タグ名
        tag_node_name = str(node_id) + ':' + self.node_tag
        tag_node_input01_name = tag_node_name + ':' + self.TYPE_IMAGE + ':Input01'
        tag_node_input01_value_name = tag_node_name + ':' + self.TYPE_IMAGE + ':Input01Value'
        tag_node_input02_name = tag_node_name + ':' + self.TYPE_TEXT + ':Input02'
        tag_node_input02_value_name = tag_node_name + ':' + self.TYPE_TEXT + ':Input02Value'

        tag_node_button_name = tag_node_name + ':' + self.TYPE_TEXT + ':Button'
        tag_node_button_value_name = tag_node_name + ':' + self.TYPE_TEXT + ':ButtonValue'

        # OpenCV向け設定
        self._opencv_setting_dict = opencv_setting_dict
        small_window_w = self._opencv_setting_dict['result_width']
        small_window_h = self._opencv_setting_dict['result_height']

        # 初期化用黒画像
        black_image = np.zeros((small_window_w, small_window_h, 3))
        black_texture = convert_cv_to_dpg(
            black_image,
            small_window_w,
            small_window_h,
        )

        # テクスチャ登録
        with dpg.texture_registry(show=False):
            dpg.add_raw_texture(
                small_window_w,
                small_window_h,
                black_texture,
                tag=tag_node_input01_value_name,
                format=dpg.mvFormat_Float_rgb,
            )

        # ノード
        with dpg.node(
                tag=tag_node_name,
                parent=parent,
                label=self.node_label,
                pos=pos,
        ):
            # ROS2 Topic入力欄
            with dpg.node_attribute(
                    tag=tag_node_input02_name,
                    attribute_type=dpg.mvNode_Attr_Static,
            ):
                dpg.add_input_text(
                    tag=tag_node_input02_value_name,
                    label='Topic',
                    width=small_window_w - 30,
                )
            # 画像
            with dpg.node_attribute(
                    tag=tag_node_input01_name,
                    attribute_type=dpg.mvNode_Attr_Input,
            ):
                dpg.add_image(tag_node_input01_value_name)

            # 録画/再生追加ボタン
            with dpg.node_attribute(
                    tag=tag_node_button_name,
                    attribute_type=dpg.mvNode_Attr_Static,
            ):
                dpg.add_button(
                    label=self._start_label,
                    tag=tag_node_button_value_name,
                    width=small_window_w,
                    callback=self._button,
                    user_data=tag_node_name,
                )

        return tag_node_name

    def update(
        self,
        node_id,
        connection_list,
        node_image_dict,
        node_result_dict,
    ):
        tag_node_name = str(node_id) + ':' + self.node_tag
        input_value01_tag = tag_node_name + ':' + self.TYPE_IMAGE + ':Input01Value'
        input_value02_tag = tag_node_name + ':' + self.TYPE_TEXT + ':Input02Value'

        small_window_w = self._opencv_setting_dict['result_width']
        small_window_h = self._opencv_setting_dict['result_height']
        draw_info_on_result = self._opencv_setting_dict['draw_info_on_result']

        # ROS2 Topic Name取得
        ros2_topic_name = dpg_get_value(input_value02_tag)

        # ros2_nodeインスタンス取得
        ros2_node = None
        if ros2_topic_name != '':
            if ros2_topic_name in self.ros2_node_list:
                ros2_node =self.ros2_node_list[ros2_topic_name]

        # 画像取得元のノード名(ID付き)を取得する
        node_name = ''
        connection_info_src = ''
        for connection_info in connection_list:
            connection_info_src = connection_info[0]
            connection_info_src = connection_info_src.split(':')[:2]
            node_name = connection_info_src[1]
            connection_info_src = ':'.join(connection_info_src)

        # 画像取得
        frame = node_image_dict.get(connection_info_src, None)

        # 描画
        if frame is not None:
            if draw_info_on_result and connection_info_src != '':
                node_result = node_result_dict[connection_info_src]
                frame = draw_info(node_name, node_result, frame)
            texture = convert_cv_to_dpg(
                frame,
                small_window_w,
                small_window_h,
            )
            dpg_set_value(input_value01_tag, texture)
            if ros2_node != None:
                image_pub = Image()
                image_pub = ros2_node.bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
                ros2_node.pub.publish(image_pub)
            
        return frame, None

    def close(self, node_id):
        pass

    def get_setting_dict(self, node_id):
        tag_node_name = str(node_id) + ':' + self.node_tag
        tag_node_input02_value_name = tag_node_name + ':' + self.TYPE_TEXT + ':Input02Value'

        pos = dpg.get_item_pos(tag_node_name)
        ros2_topic_name = dpg_get_value(tag_node_input02_value_name)

        setting_dict = {}
        setting_dict['ver'] = self._ver
        setting_dict['pos'] = pos
        setting_dict[tag_node_input02_value_name] = ros2_topic_name

        return setting_dict

    def set_setting_dict(self, node_id, setting_dict):
        tag_node_name = str(node_id) + ':' + self.node_tag
        tag_node_input02_value_name = tag_node_name + ':' + self.TYPE_TEXT + ':Input02Value'

        ros2_topic_name = setting_dict[tag_node_input02_value_name]

        dpg_set_value(tag_node_input02_value_name, ros2_topic_name)

    def _button(self, sender, data, user_data):
        tag_node_name = user_data
        input_value02_tag = tag_node_name + ':' + self.TYPE_TEXT + ':Input02Value'
        tag_node_button_value_name = tag_node_name + ':' + self.TYPE_TEXT + ':ButtonValue'

        label = dpg.get_item_label(tag_node_button_value_name)

        # ROS2 Topic取得
        ros2_topic_name = dpg_get_value(input_value02_tag)

        if label == self._start_label:
            if ros2_topic_name != '':
                if not (ros2_topic_name in self.ros2_node_list):
                    print("Create ROS2 Node...")
                    self.ros2_node_list[ros2_topic_name] = ImagePublisher(ros2_topic_name)

            dpg.set_item_label(tag_node_button_value_name, self._stop_label)
        elif label == self._stop_label:
            if ros2_topic_name != '':
                if (ros2_topic_name in self.ros2_node_list):
                    self.ros2_node_list[ros2_topic_name].destroy_publisher(self.ros2_node_list[ros2_topic_name].pub)
                    # self.ros2_node_list[ros2_topic_name].destroy_node()
                    print("Delete ROS2 Node...")
                    del self.ros2_node_list[ros2_topic_name]

            dpg.set_item_label(tag_node_button_value_name, self._start_label)
