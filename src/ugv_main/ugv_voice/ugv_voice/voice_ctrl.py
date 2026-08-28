#!/usr/bin/env python3
import os
import re
import time
import yaml
import threading

import pyttsx3
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor,SetParametersResult

from .kws_sherpa_onnx import kws_sherpa_onnx
from .asr_sherpa_ncnn import asr_sherpa_ncnn
from .tts_sherpa_onnx import tts_sherpa_onnx

import subprocess

class VoiceCtrl(Node):
    def __init__(self):
        super().__init__('voice_ctrl')

        this_path = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(this_path, "../config/voice_config.yaml")
        with open(config_path, "r") as yaml_file:
            self.config = yaml.safe_load(yaml_file)

        pygame.mixer.init()
        pygame.mixer.music.set_volume(self.config['audio_config']['default_volume'])

        self.declare_parameter("language", "en", ParameterDescriptor(description="Language: zh or en"))
        language = self.get_parameter("language").value

        # ---------------- KWS ----------------
        if language == "en":
            kws_model_dir = os.path.join(
                this_path,
                "models",
                "kws",
                "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01"
            )
        elif language == "zh":
            kws_model_dir = os.path.join(
                this_path,
                "models",
                "kws",
                "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
            )
        else:
            self.get_logger().warn(
                f"Unsupported language '{language}', fallback to zh"
            )
            language = "zh"
            kws_model_dir = os.path.join(
                this_path,
                "models",
                "kws",
                "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
            )

        self.kws = kws_sherpa_onnx(kws_model_dir)
        self.kws_enabled = False
        self.kws_thread = threading.Thread(target=self.kws_loop, daemon=True)
        self.kws_thread.start()

        # ---------------- ASR ----------------
        if language == "en":
            asr_model_dir = os.path.join(
                this_path,
                "models",
                "asr",
                "sherpa-ncnn-streaming-zipformer-20M-2023-02-17"
            )
        else:  # zh
            asr_model_dir = os.path.join(
                this_path,
                "models",
                "asr",
                "sherpa-ncnn-streaming-zipformer-bilingual-zh-en-2023-02-13"
            )

        self.asr = asr_sherpa_ncnn(asr_model_dir)
        self.asr_enabled = False
        self.asr_thread = threading.Thread(target=self.asr_loop, daemon=True)
        self.asr_thread.start()

        tts_model_dir = os.path.join(this_path, "models", "sherpa-onnx-vits-zh-ll")
        self.tts = tts_sherpa_onnx(tts_model_dir)
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', self.config['audio_config']['speed_rate'])

        self.usb_connected = False
        try:
            self.usb_connected = True
            self.get_logger().info("Audio USB connected")
        except Exception as e:
            self.usb_connected = False
            self.get_logger().warn(f"Audio USB not connected: {e}")

        self.play_audio_event = threading.Event()
        self.min_time_between_play = self.config['audio_config']['min_time_bewteen_play']

        self.kws_subscription = self.create_subscription(Bool,'/kws',self.kws_callback,10)
        self.asr_subscription = self.create_subscription(Bool,'/asr',self.asr_callback,10)
        self.tts_subscription = self.create_subscription(String,'/tts',self.tts_callback,10)

    def kws_loop(self):
        while True:
            if self.kws_enabled:
                result = self.kws.get_result()
                if result:
                    print("Keyword Spotting", result)
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def asr_loop(self):
        while True:
            if self.asr_enabled:
                result = self.asr.get_result()
                if result:
                    print("voice reconize", result)
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def contains_chinese(self, text):
        return bool(re.search('[\u4e00-\u9fff]', text))

    def play_audio(self, input_audio_file):
        if not self.usb_connected:
            return
        try:
            pygame.mixer.music.load(input_audio_file)
            pygame.mixer.music.play()
        except:
            play_audio_event.clear()
            return
        while pygame.mixer.music.get_busy():
            pass
        time.sleep(self.min_time_between_play)
        self.play_audio_event.clear()

    def play_speech(self, input_text):
        filename = 'audio-say.wav'
        if not self.usb_connected:
            return
        try:
            if self.contains_chinese(input_text):
                filename = self.tts.synthesize(input_text, output_wav=os.path.join("/home/ws/ugv_ws/tts_cn.wav"))
                self.play_audio(filename)                  
            else:
                filename = os.path.join("/home/ws/ugv_ws/tts_en.wav")
                self.engine.save_to_file(input_text, filename)
                self.engine.runAndWait()
                if os.path.exists(filename) and os.path.getsize(filename) > 0:
                    self.play_audio(filename)
                else:
                    self.get_logger().warn("TTS file not generated correctly")
        except Exception as e:
            self.get_logger().error(f"[play failure] {e}")
        finally:
            if filename and os.path.exists(filename):
                try:
                    os.remove(filename)
                except Exception as e:
                    self.get_logger().warn(f"[delete file failure] {e}")
            self.play_audio_event.clear()

    def kws_callback(self, msg):
        if msg.data:
            if not self.kws_enabled:
                self.get_logger().info("KWS start")
                self.kws.start()
                self.kws_enabled = True
        else:
            if self.kws_enabled:
                self.get_logger().info("KWS stop")
                self.kws.stop()
                self.kws_enabled = False

    def asr_callback(self, msg):
        if msg.data:
            if not self.asr_enabled:
                self.get_logger().info("ASR start")
                self.asr.start()
                self.asr_enabled = True
        else:
            if self.asr_enabled:
                self.get_logger().info("ASR stop")
                self.asr.stop()
                self.asr_enabled = False

    def tts_callback(self, msg):
        self.get_logger().info(f"Speech request: {msg.data}")

        if not self.usb_connected:
            return
        if self.play_audio_event.is_set():
            return
        self.play_audio_event.set()

        self.play_speech(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

