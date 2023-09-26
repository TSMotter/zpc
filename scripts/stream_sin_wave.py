import math
import threading
import time
import json
import logging
import argparse
import serial
from enum import Enum, auto
import yahdlc
import asyncio
from websockets.server import serve

import sin_wave_pb2


class OP(Enum):
    JSON_TO_STDOUT = 0
    PROTOBUF_TO_FILE = auto()
    PROTOBUF_TO_SERIAL = auto()
    HDLC_TO_FILE = auto()
    HDLC_TO_SERIAL = auto()
    JSON_TO_WEBSOCKET = auto()


OPERATIONS = [
    "JSON_TO_STDOUT",
    "PROTOBUF_TO_FILE",
    "PROTOBUF_TO_SERIAL",
    "HDLC_TO_FILE",
    "HDLC_TO_SERIAL",
    "JSON_TO_WEBSOCKET"]
SERIAL = None


class Actions:
    def __init__(self, f="batch.bin"):
        self.filep = f

    def dumb(self, batch):
        ...

    def write_to_stdout(self, batch):
        print(batch)

    def write_to_file(self, batch):
        with open(self.filep, "ab") as f:
            f.write(batch)

    def write_to_serial(self, batch):
        global SERIAL
        SERIAL.write(batch)

    async def websocket_main(self, handler):
        async with serve(handler, "localhost", 8765):
            await asyncio.Future()


class SinWaveGenerator:
    def __init__(self, ch, tbd, dry):
        # Class members
        self.samples_in_each_sin_cycle = None
        self.angular_frequency = None
        self.batch_cadency = None
        self.num_samples_per_batch = None
        self.num_sample_per_second = None
        self.time_step_between_samples = None
        self.time_before_die = None
        self.sin_amplitude = None
        self.channel = None
        self.overall_samples_cntr = 0

        # Constants
        self.sin_amplitude = 1.0
        self.time_before_die = tbd

        # Initializations
        self.channel = str(ch)
        self.dryrun = dry

        self._gatter_parameters()
        self._log()

    def _gatter_parameters(self):
        params = self.channel.split('_')
        self.samples_in_each_sin_cycle = int(params[1])
        batch_cadency_ms = int(params[2])
        sin_frequency = float(params[3])

        self.angular_frequency = 2 * math.pi * sin_frequency
        self.batch_cadency = batch_cadency_ms / 1000
        num_cycles_per_batch = self.batch_cadency * sin_frequency
        self.num_samples_per_batch = math.ceil(
            num_cycles_per_batch * self.samples_in_each_sin_cycle)
        self.num_sample_per_second = self.samples_in_each_sin_cycle * sin_frequency
        self.time_step_between_samples = sin_frequency * \
            (1 / self.num_samples_per_batch)

    def _log(self):
        logging.debug(
            f"self.samples_in_each_sin_cycle: {self.samples_in_each_sin_cycle}")
        logging.debug(f"self.angular_frequency: {self.angular_frequency}")
        logging.debug(f"self.batch_cadency: {self.batch_cadency}")
        logging.debug(
            f"self.num_samples_per_batch: {self.num_samples_per_batch}")
        logging.debug(
            f"self.num_sample_per_second: {self.num_sample_per_second}")
        logging.debug(
            f"self.time_step_between_samples: {self.time_step_between_samples}")
        logging.debug(f"self.time_before_die: {self.time_before_die}")
        logging.debug(f"self.sin_amplitude: {self.sin_amplitude}")
        logging.debug(f"self.channel: {self.channel}")

    def _generate_sample(self):
        T = (self.overall_samples_cntr / self.num_sample_per_second)
        sample_value = self.sin_amplitude * \
            math.sin(self.angular_frequency * T)

        channel = self.channel
        freq = self.samples_in_each_sin_cycle
        val = float(format(sample_value, '.10f'))
        time = float(format(T, '.10f'))
        self.overall_samples_cntr = self.overall_samples_cntr + 1
        return channel, freq, val, time

    def _generate_batch_as_json(self):
        batch_samples_as_json = []

        for _ in range(self.num_samples_per_batch):
            c, f, v, t = self._generate_sample()

            sample_as_json = {"channel": c,
                              "frequency": f,
                              "value": v,
                              "time": t}
            batch_samples_as_json.append(sample_as_json)

        return json.dumps(batch_samples_as_json)

    def _generate_batch_as_protobuf(self):
        batch_serializer = sin_wave_pb2.Batch()

        for _ in range(self.num_samples_per_batch):
            c, f, v, t = self._generate_sample()

            sample_as_protobuf = batch_serializer.items.add()
            sample_as_protobuf.channel = c
            sample_as_protobuf.frequency = f
            sample_as_protobuf.value = v
            sample_as_protobuf.time = t

        return batch_serializer.SerializeToString()

    # coroutine to stream json
    async def stream_json_coro(self, websocket):
        start_time = time.time()
        try:
            while time.time() - start_time <= self.time_before_die:
                batch = self._generate_batch_as_json()
                await asyncio.sleep(self.batch_cadency)
                await websocket.send(batch)
        except Exception as e:
            logging.info(f"Caught websocket exception {e}")
            websocket.close()
            await asyncio.sleep(2)
            self.overall_samples_cntr = 0

    def stream_json(self, action: callable):
        start_time = time.time()
        while time.time() - start_time <= self.time_before_die:

            batch = self._generate_batch_as_json()

            time.sleep(self.batch_cadency)
            if (self.dryrun == False):
                action(batch)

    def stream_protobuf(self, action: callable):
        start_time = time.time()
        while time.time() - start_time <= self.time_before_die:

            batch = self._generate_batch_as_protobuf()

            time.sleep(self.batch_cadency)
            if (self.dryrun == False):
                action(batch)

    def stream_hdlc_frame(self, action: callable):
        start_time = time.time()
        while time.time() - start_time <= self.time_before_die:

            batch = self._generate_batch_as_protobuf()
            hdlc_frame = yahdlc.frame_data(batch)

            time.sleep(self.batch_cadency)
            if (self.dryrun == False):
                action(hdlc_frame)


class Streamer():
    def __init__(self, args):
        global SERIAL

        self.args = args

        if (self.args.device):
            SERIAL = serial.Serial(
                port=self.args.device,
                baudrate=115200,
                bytesize=8,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5)

            def serial_listener_thread():
                global SERIAL
                while True:
                    incoming_bytes = SERIAL.read_all()
                    if incoming_bytes:
                        logging.debug(f"Read: {incoming_bytes.hex()}")

            serial_thread = threading.Thread(target=serial_listener_thread)
            serial_thread.daemon = True
            serial_thread.start()

        self.act = Actions()
        self.swg = SinWaveGenerator(self.args.channel, 30, self.args.dryrun)

    def stream(self):
        global OPERATIONS

        if (self.args.operation == OPERATIONS[OP.JSON_TO_STDOUT.value]):
            self.swg.stream_json(self.act.write_to_stdout)
        elif (self.args.operation == OPERATIONS[OP.PROTOBUF_TO_FILE.value]):
            self.swg.stream_protobuf(self.act.write_to_file)
        elif (self.args.operation == OPERATIONS[OP.PROTOBUF_TO_SERIAL.value]):
            self.swg.stream_protobuf(self.act.write_to_serial)
        elif (self.args.operation == OPERATIONS[OP.HDLC_TO_FILE.value]):
            self.swg.stream_hdlc_frame(self.act.write_to_file)
        elif (self.args.operation == OPERATIONS[OP.HDLC_TO_SERIAL.value]):
            self.swg.stream_hdlc_frame(self.act.write_to_serial)
        elif (self.args.operation == OPERATIONS[OP.JSON_TO_WEBSOCKET.value]):
            asyncio.run(self.act.websocket_main(self.swg.stream_json_coro))


"""**********
    MAIN
**********"""
if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            description='Streams samples of a sin wave based on the channel parameter',
            epilog='No epilog')
        parser.add_argument(
            '-op',
            '--operation',
            default=OPERATIONS[0],
            type=str,
            required=False,
            help="Choose the operation mode",
            choices=OPERATIONS),
        parser.add_argument(
            '-c',
            '--channel',
            type=str,
            required=True,
            help="""Channel name follows the pattern 'channel_X_Y_Z'.
            X is the number of samples present in each sinusoidal cycle
            Y represents the time step (in milliseconds) between 2 subsequent batches of data.
            Z is the frequency of the sinusoidal wave itself"""),
        parser.add_argument(
            '-d',
            '--device',
            type=str,
            required=False,
            help="Device name. Ex: /dev/ttyUSB0"),
        parser.add_argument(
            '-l',
            '--loglevel',
            default="INFO",
            type=str,
            required=False,
            help="Log level",
            choices=["ERROR", "WARNING", "INFO", "DEBUG"]),
        parser.add_argument('--dryrun', default=False, action='store_true')

        args = parser.parse_args()
        logging.basicConfig(level=args.loglevel)

        s = Streamer(args)
        s.stream()

    except KeyboardInterrupt:
        logging.info('KeyboardInterrupt caught.')
        exit(0)
    finally:
        logging.info('Finally block reached.')
        if SERIAL is not None:
            SERIAL.close()
