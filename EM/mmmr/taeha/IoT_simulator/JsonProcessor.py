# This Python file uses the following encoding: utf-8
from PySide6.QtCore import QObject, Signal, Slot
from PySide6 import QtQuick
import json


class JsonProcessor(QObject):
    def __init__(self):
        super().__init__()

    @Slot(str, result=dict)
    def processJson(self, jsonString):
        """QML에서 JSON 문자열을 받아 Python에서 파싱 후 반환"""
        try:
            parsed_data = json.loads(jsonString)
            print(parsed_data["data"])


            return parsed_data  # QML에서 접근 가능하도록 반환
        except json.JSONDecodeError:
            print("JSON parsing failed")
            return {"error": "Invalid JSON"}
