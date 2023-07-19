import json, sin_wave_pb2


FILEP = "batch.bin"

deserializer = sin_wave_pb2.Batch()

with open(FILEP, "rb") as f:
  deserializer.ParseFromString(f.read())

batch_samples_as_json = []
for sample in deserializer.items:
    sample_as_json = {
    "channel": sample.channel,
    "frequency": sample.frequency,
    "value": sample.value,
    "time": sample.time
    }
    batch_samples_as_json.append(sample_as_json)

batch_as_json_object = json.dumps(batch_samples_as_json)
print(batch_as_json_object)