#!/usr/bin/env python3

import asyncio
from websockets.client import connect
import json
import traceback
import time
from server import Events, Subscriber, Sensor

class MoonrakerResponse:
    def __init__(self, data : dict):
        self.data = data
        self.id = int(data.get('id'))
        self.result = data.get('result')
        self.error = data.get('error')

    def is_successful(self):
        return self.error is None

class MoonrakerClient:
    NOTIFY_STATUS_UPDATE = 'notify_status_update'
    NOTIFY_KLIPPY_READY = 'notify_klippy_ready'
    NOTIFY_PROC_STAT_UPDATE = 'notify_proc_stat_update'

    def __init__(self, host):
        self.host = host
        self.seq = 0
        self.responses = {}
        self.ws = None
        self.events = Events()

    def is_connected(self):
        return self.ws is not None

    async def connect(self):
        try:
            self.ws = await connect(f"ws://{self.host}/websocket")
        except:
            print(f'Error connecting to {self.host}')
        return self.ws

    async def receive_task(self):
        while True:
            await self.connect()

            while True:
                try:
                    message = await self.ws.recv()
                    data = json.loads(message)
                    if 'id' in data:
                        r = MoonrakerResponse(data)
                        self.responses[r.id] = r
                    else:
                        method = data.get('method')
                        params = data.get('params')
                        if params is None:
                            params = {}
                            print(f'none params {data}')
                        await self.events.trigger(method, *params)
                except BaseException as e:
                    self.ws = None
                    print(f'Exception while receiving message: {e}')
                    print(traceback.format_exc())
                    break

    async def jsonrpc(self, method, params=None):
        msgid = self.seq
        self.seq += 1
        data = {
            "jsonrpc": "2.0",
            "method": method,
            "id": msgid,
        }
        if params:
            data['params'] = params
        await self.ws.send(json.dumps(data))
        while True:
            if msgid not in self.responses:
                await asyncio.sleep(0.1)
                continue
            r = self.responses[msgid]
            if not r.is_successful():
                print(f'Error calling {method}: {r.error}')
            return r.result

    async def get_server_info(self):
        return await self.jsonrpc("server.config")

    async def get_objects_list(self):
        return (await self.jsonrpc("printer.objects.list")).get('objects')

    async def get_objects_query(self, objects):
        params = {
            "objects": objects,
        }
        return await self.jsonrpc("printer.objects.query", params)

    async def get_printer_info(self):
        return await self.jsonrpc("printer.info")

    async def get_server_files_metadata(self, filename):
        params = {
            "filename": filename,
        }
        return await self.jsonrpc("server.files.metadata", params)

    async def wait_ready(self):
        while True:
            info = await self.get_printer_info()
            if info.get('state') == 'ready':
                break
            await asyncio.sleep(0.1)

    async def objects_subscribe(self, objects):
        params = {
            "objects": objects,
        }
        return await self.jsonrpc("printer.objects.subscribe", params)

class LiveSubscriber(Subscriber):

    def __init__(self, host):
        self.client = MoonrakerClient(host)
        self.client.events.on(MoonrakerClient.NOTIFY_STATUS_UPDATE, self.notify_status_update)
        self.client.events.on(MoonrakerClient.NOTIFY_KLIPPY_READY, self.klippy_ready)
        self.client.events.on(MoonrakerClient.NOTIFY_PROC_STAT_UPDATE, self.stat_update)
        super().__init__()

        self.sensors = {}
        self.klippy_reconnect = False
        self.klippy_time_offset = None
        self.eventtime_offset = None

    def tasks(self):
        return [
            self.client.receive_task(),
            self.task(),
        ]

    async def notify_status_update(self, statuses : dict, eventtime):
        # keep up-to-date merged sensor info so we can send them to new clients.
        for name, status in statuses.items():
            # if a new print starts, get file information immediately
            if name == 'print_stats' and 'filename' in status:
                if not status['filename']:
                    status['file'] = None
                else:
                    status['file'] = await self.client.get_server_files_metadata(status['filename'])
                    print(f'new print {statuses}')

            if name in self.sensors:
                self.sensors[name].update(status, eventtime + self.eventtime_offset)
            else:
                self.sensors[name] = Sensor(name, status, eventtime + self.eventtime_offset)

        # suppress temperature-only updates when target is 0
        for name in list(statuses.keys()):
            if list(statuses[name].keys()) == ['temperature'] and self.sensors[name].status['target'] == 0.0:
                del statuses[name]

        # notify of all sensors that were updated as we received it from the printer
        if len(statuses) > 0:
            await self.events.trigger(self.SENSOR_UPDATED, statuses, eventtime + self.eventtime_offset)

    async def klippy_ready(self):
        print('Klippy restarted')
        self.klippy_reconnect = True

    async def stat_update(self, stats):
        server_time = stats.get('moonraker_stats').get('time')
        if self.klippy_time_offset is None:
            self.klippy_time_offset = server_time - time.time()
            print(f'Klippy time offset: {self.klippy_time_offset}s')

    def time_offset(self):
        return self.klippy_time_offset

    async def subscribe_to_objects(self):
        objects = await self.client.get_objects_list()

        sensors = []
        for object_name in objects:
            for wanted in ['filament_switch_sensor', 'filament_motion_sensor', 'high_resolution_filament_sensor']:
                if object_name.split()[0] == wanted:
                    sensors.append(object_name)
        print(f"Found filament sensors to monitor: {sensors}")

        status = (await self.client.get_objects_query({'configfile': None})).get('status')
        extruders = {}
        for sensor in sensors:
            cfg = status['configfile']['config'][sensor]
            extruder = cfg.get('extruder')
            if extruder:
                extruders[sensor] = extruder
        print(f"Found extruders to monitor: {extruders}")

        subscriptions = {k: None for k in sensors}
        subscriptions['toolhead'] = ['position', 'status']
        subscriptions['print_stats'] = ['filename', 'state', 'message', 'info']
        for extruder in extruders.values():
            subscriptions[extruder] = None
        resp = await self.client.objects_subscribe(subscriptions)

        # the eventtime in this response corresponds to the "most recent" eventtime
        # on the printer, we keep track of it here so we can always send updates
        # with the correct local time.
        self.eventtime_offset = -resp.get('eventtime') + time.time()
        await self.notify_status_update(resp.get('status'), resp.get('eventtime'))

    def historical_data(self):
        now = time.time() + self.klippy_time_offset
        for sensor in self.sensors.values():
            yield Sensor(sensor.name, sensor.status, now)

    async def task(self):
        try:
            while True:
                while not self.client.is_connected():
                    await asyncio.sleep(0.1)
                print('Connected to printer')

                self.klippy_reconnect = False

                await self.client.wait_ready()
                print('Printer ready')

                await self.subscribe_to_objects()

                while self.client.is_connected() and not self.klippy_reconnect:
                    await asyncio.sleep(0.1)
                print('Disconnected from printer')
        except BaseException as e:
            print(f'Exception in {self} task: {e}')
            print(traceback.format_exc())
