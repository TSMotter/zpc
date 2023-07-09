import math
import time
import json
import logging
import argparse
import serial

import sin_wave_pb2


class SinWave:
    def __init__(self, args, serial):
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
        self.serial = None

        # Constants
        self.sin_amplitude = 1.0
        self.time_before_die = 30

        # Initialization 
        self.channel = str(args.channel)
        self.dryrun = args.dryrun
        self.serial = serial

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
        self.num_samples_per_batch = math.ceil(num_cycles_per_batch * self.samples_in_each_sin_cycle)
        self.num_sample_per_second = self.samples_in_each_sin_cycle * sin_frequency
        self.time_step_between_samples = sin_frequency*(1/self.num_samples_per_batch)

    def _log(self):
        logging.debug(f"self.samples_in_each_sin_cycle: {self.samples_in_each_sin_cycle}")
        logging.debug(f"self.angular_frequency: {self.angular_frequency}")
        logging.debug(f"self.batch_cadency: {self.batch_cadency}")
        logging.debug(f"self.num_samples_per_batch: {self.num_samples_per_batch}")
        logging.debug(f"self.num_sample_per_second: {self.num_sample_per_second}")
        logging.debug(f"self.time_step_between_samples: {self.time_step_between_samples}")
        logging.debug(f"self.time_before_die: {self.time_before_die}")
        logging.debug(f"self.sin_amplitude: {self.sin_amplitude}")
        logging.debug(f"self.channel: {self.channel}")

    def generate_samples_json(self, action):
        samples_cntr = 0
        start_time = time.time()
        while time.time() - start_time <= self.time_before_die:
            batch_samples_as_json = []

            for sample_idx in range(self.num_samples_per_batch):
                T = ((sample_idx+samples_cntr)/self.num_sample_per_second)
                sample_value = self.sin_amplitude * math.sin(self.angular_frequency * T)

                sample_as_json = {"channel": self.channel,
                                "frequency": self.samples_in_each_sin_cycle,
                                "value": float(format(sample_value, '.10f')),
                                "time": float(format(T, '.10f'))}
                batch_samples_as_json.append(sample_as_json)

            samples_cntr = samples_cntr + self.num_samples_per_batch
            batch_as_json_object = json.dumps(batch_samples_as_json)

            time.sleep(self.batch_cadency)
            if (self.dryrun == False):
                action(batch_as_json_object)

    def generate_samples_protobuf(self, action):
        samples_cntr = 0
        start_time = time.time()
        while time.time() - start_time <= self.time_before_die:
            batch_serializer = sin_wave_pb2.Batch()

            for sample_idx in range(self.num_samples_per_batch):
                T = ((sample_idx+samples_cntr)/self.num_sample_per_second)
                sample_value = self.sin_amplitude * math.sin(self.angular_frequency * T)

                sample_as_protobuf = batch_serializer.items.add()
                sample_as_protobuf.channel = self.channel
                sample_as_protobuf.frequency = self.samples_in_each_sin_cycle
                sample_as_protobuf.value = float(format(sample_value, '.10f'))
                sample_as_protobuf.time = float(format(T, '.10f'))

            samples_cntr = samples_cntr + self.num_samples_per_batch

            time.sleep(self.batch_cadency)
            if (self.dryrun == False):
                action(batch_serializer.SerializeToString())
    
    def action_dumb(self, batch):
        ...

    def action_print(self, batch):
        print(batch)

    def action_write_serialized_data_to_file(self, batch):
        with open("batch.bin", "ab") as f:
            f.write(batch)

    def action_write_serialized_data_on_serial(self, batch):
        self.serial.write(batch)



"""**********
    MAIN
**********"""
if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            description='Streams a sin wave',
            epilog='No epilog')
        parser.add_argument(
            '-c',
            '--channel',
            type=str,
            required=True,
            help="Channel name in the pattern 'channel_20_2000_2'"),
        parser.add_argument(
            '-D',
            '--device',
            type=str,
            required=False,
            help="Device name"),
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

        serial_error = 1
        ser = serial.Serial(port=args.device, baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout=0.5)
        serial_error = 0

        S = SinWave(args, ser)
        S.generate_samples_protobuf(S.action_write_serialized_data_on_serial)

    except KeyboardInterrupt:
        logging.info('KeyboardInterrupt caught. Performing any cleanup needed')
        exit(0)
    finally:
        logging.info('Finally block reached.')
        if not serial_error:
            ser.close()