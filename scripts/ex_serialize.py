import sin_wave_pb2

FILEP = "batch.bin"

serializer = sin_wave_pb2.Batch()

foo = serializer.items.add()
foo.channel = "channel_20_2000_2"
foo.frequency = 20
foo.value = 0.3090169944
foo.time = 0.025

bar = serializer.items.add()
bar.channel = "channel_20_2000_2"
bar.frequency = 20
bar.value = 0.5877852523
bar.time = 0.050


with open(FILEP, "wb") as f:
    f.write(serializer.SerializeToString())
