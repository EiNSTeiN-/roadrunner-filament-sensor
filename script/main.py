#!/usr/bin/env python3

import asyncio
from websockets.client import connect
import argparse
import json
import sys

from live import LiveSubscriber
from replay import ReplaySubscriber
from server import Server, Subscriber

parser = argparse.ArgumentParser(
                    prog='stream',
                    description='Stream motion sensor data')

parser.add_argument('-p', '--port', type=int, default=8812, help="Host a http server on the specified port for viewing data")
parser.add_argument('--http', action='store_true', help="Host a http server on the specified port for viewing data")
parser.add_argument('--live', help="Fetch live sensor data from printer")
parser.add_argument('-f', '--file', help="Store raw sensor data to file")
parser.add_argument('--replay', help="Replay saved sensor data from file")
# parser.add_argument('-s', '--sensor', help="Name of sensor (defaults to all filament motion sensors)")

args = parser.parse_args()
tasks = []
writer = None

class FileWriter:
    def __init__(self, filename):
        self.filename = filename
        self.file = open(filename, "w")
        self.first_eventtime = None
    
    async def __call__(self, statuses, eventtime):
        # Write each line as we received it from the printer: a dict of all statuses
        # that were updated at this eventtime.
        if self.first_eventtime is None:
            self.first_eventtime = eventtime
        self.file.write(json.dumps({'eventtime': eventtime - self.first_eventtime, 'status': statuses})+"\n")

    def close(self):
        self.file.close()

if args.live and args.replay:
    print('--live and --replay are mutually exclusive')
    sys.exit(1)

# if not args.http and not args.file:
#     print('Nothing to do; specify either --serve or --file to do something with sensor data')
#     sys.exit(1)

if args.live:
    subscriber = LiveSubscriber(args.live)

    if args.file:
        print('Saving status updates to', args.file)
        writer = FileWriter(args.file)
        subscriber.events.on(Subscriber.SENSOR_UPDATED, writer)

    async def sensor_changed(statuses, eventtime):
        print('update:', statuses)
    subscriber.events.on(Subscriber.SENSOR_UPDATED, sensor_changed)
elif args.replay:
    subscriber = ReplaySubscriber(args.replay)

if subscriber:
    tasks += [asyncio.ensure_future(t) for t in subscriber.tasks()]

if args.http:
    server = Server(subscriber, args.port)
    tasks += [asyncio.ensure_future(server.task())]

if len(tasks) == 0:
    print('Nothing to do!')
    sys.exit(1)

try:
    asyncio.get_event_loop().run_until_complete(asyncio.wait(tasks))
finally:
    if writer:
        writer.close()
