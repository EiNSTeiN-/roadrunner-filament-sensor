#!/usr/bin/env python3

import asyncio
from websockets.client import connect
import json
import time
from server import Sensor, Subscriber

class ReplaySubscriber(Subscriber):
    def __init__(self, filename):
        self.filename = filename
        super().__init__()

    def historical_data(self):
        start_time = time.time()
        with open(self.filename, 'r') as f:
            for line in f:
                data = json.loads(line)
                eventtime = data.get('eventtime') + start_time
                for name, status in data.get('status').items():
                    yield Sensor(name, status, eventtime)

    async def task(self):
        while True:
            await asyncio.sleep(0.1)
