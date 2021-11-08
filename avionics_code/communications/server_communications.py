from auvsi_suas.client import client
from auvsi_suas.proto import interop_api_pb2

class ServerComs:
    def __init__(self):
        pass

    def connect(self):
        client = client.Client(url='http://127.0.0.1:8000',
                               username='testuser',
                               password='testpass')

    def get_mission(self):
        pass

    def upload_telemetry(self):
        pass
