import json
import traceback
import mergedeep
import asyncio
import aiohttp
import time
from aiohttp import web

class Events:
    def __init__(self):
        self.callbacks = {}
        self.unhandled = []

    def on(self, method, cb):
        if method in self.callbacks:
            self.callbacks[method].append(cb)
        else:
            self.callbacks[method] = [cb]

    def unregister(self, method, cb):
        if method in self.callbacks:
            self.callbacks[method].remove(cb)

    async def trigger(self, method, *params):
        if method in self.callbacks:
            for cb in self.callbacks[method]:
                try:
                    await cb(*params)
                except BaseException as e:
                    print(f'Exception when calling {cb}: {e}')
                    print(traceback.format_exc())
        elif method not in self.unhandled:
            self.unhandled.append(method)
            print(f'Unhandled event: {method}')


class Subscriber:
    SENSOR_UPDATED = 'sensor-updated'

    def __init__(self):
        self.events = Events()

    def tasks(self):
        return [self.task()]

    async def task(self):
        return

    def historical_data(self):
        return []

    def time_offset(self):
        return 0.

class Sensor:
    def __init__(self, name: str, status: dict, eventtime: float):
        self.name = name
        self.status = {}
        mergedeep.merge(self.status, status)
        self.eventtime = eventtime

    def update(self, status, eventtime):
        mergedeep.merge(self.status, status)
        self.eventtime = eventtime

class WebsocketHandler:
    def __init__(self, subscriber: Subscriber, ws):
        self.subscriber = subscriber
        self.ws = ws

    async def send(self, data : dict):
        await self.ws.send_str(json.dumps(data))

    def batch_historical_data(self, batch_size):
        batch = {}
        for sensor in self.subscriber.historical_data():
            if sensor.eventtime not in batch:
                if len(batch) >= batch_size:
                    yield batch
                    batch = {}
                batch[sensor.eventtime] = {}
            batch[sensor.eventtime][sensor.name] = sensor.status
        if len(batch) > 0:
            yield batch
        return

    async def callback(self, statuses : dict, eventtime : float):
        print(f'Sending updated sensor data: {statuses.keys()}')
        await self.send({'type': 'live', 'events': {eventtime: statuses}})

    async def handle(self):
        try:
            now = time.time() + self.subscriber.time_offset()
            await self.send({'now': now})

            print(f'Sending initial data to client...')
            for events in self.batch_historical_data(10_000):
                await self.send({'type': 'historical', 'events': events})

            print(f'Sending new data to client...')
            if self.subscriber:
                self.subscriber.events.on(Subscriber.SENSOR_UPDATED, self.callback)

            async for msg in self.ws:
                print(f'websocket message {msg}')

                if msg.type == aiohttp.WSMsgType.TEXT:
                    if msg.data == 'close':
                        await self.ws.close()
                    else:
                        print(f'Received: {msg.data}')
                        # await ws.send_str(msg.data + '/answer')
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    print('ws connection closed with exception %s' % self.ws.exception())

                await asyncio.sleep(0.1)
        finally:
            if self.subscriber:
                self.subscriber.events.unregister(Subscriber.SENSOR_UPDATED, self.callback)

class Server:
    def __init__(self, subscriber: Subscriber, port):
        self.subscriber = subscriber
        self.loop = asyncio.get_event_loop()
        self.port = port
        self.app = web.Application()
        self.app.router.add_route('GET', '/', self.index)
        self.app.router.add_route('GET', r'/assets/{name:.*}', self.assets)
        self.app.router.add_route('GET', '/api/ws', self.websocket)
        self.handler = self.app.make_handler()

    async def index(self, request):
        return web.FileResponse(path='html/index.html',headers={'Content-Type': 'text/html'})

    async def assets(self, request):
        asset = request.match_info['name']
        ext = asset.split('.')[-1]

        headers = {}
        if ext == 'js':
            headers['Content-Type'] = 'application/javascript'
        else:
            headers['Content-Type'] = 'text/%s' % (ext, )
        return web.FileResponse(path=f'assets/{asset}',headers=headers)

    async def websocket(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        print('websocket client connected')
        r = WebsocketHandler(self.subscriber, ws)
        await r.handle()
        print('websocket connection closed')

        return ws

    def task(self):
        print(f'Listening on http://127.0.0.1:{self.port}')
        return self.loop.create_server(self.handler, '0.0.0.0', self.port)

    # def stop_server(self, loop):
    #     self.srv.close()
    #     loop.run_until_complete(self.srv.wait_closed())
    #     loop.run_until_complete(self.handler.finish_connections(1.0))
    #     loop.run_until_complete(self.app.finish())
