# This Python file uses the following encoding: utf-8
import sys
from pathlib import Path

import json
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtCore import QObject, Signal, Slot

import rc_resources

sys.stdout.reconfigure(encoding='utf-8')

class JsonProcessor(QObject):
    def __init__(self):
        super().__init__()

    @Slot(str, result=dict)
    def processJson(self, jsonString):
        """QML에서 JSON 문자열을 받아 Python에서 파싱 후 반환"""
        try:
            parsed_data = json.loads(jsonString)
            return parsed_data  # QML에서 접근 가능하도록 반환
        except json.JSONDecodeError:
            print("JSON parsing failed")
            return {"error": "Invalid JSON"}

if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    json_processor = JsonProcessor()
    engine.rootContext().setContextProperty("jsonProcessor", json_processor)

    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)
    if not engine.rootObjects():
        sys.exit(-1)
    sys.exit(app.exec())
