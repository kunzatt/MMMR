# This Python file uses the following encoding: utf-8
import sys
from pathlib import Path

import json
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtCore import QObject, Signal, Slot, QResource
from JsonProcessor import JsonProcessor

import rc_resources
QResource.registerResource("rc_resources.py")

sys.stdout.reconfigure(encoding='utf-8')


if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()
    engine.addImportPath("qrc:/devices/")

    import_paths = engine.importPathList()
    print("Registered import paths:")
    for path in import_paths:
        print(path)

    json_processor = JsonProcessor()
    engine.rootContext().setContextProperty("jsonProcessor", json_processor)

    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)
    if not engine.rootObjects():
        sys.exit(-1)
    sys.exit(app.exec())
