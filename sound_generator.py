import os
import dynamic_sound as ds
from data_converter import data_converter

# data conversion from airsim data structure to csv files
data_dir = './data/run1/'
data_converter(data_dir)


src_folder = r"data/run1/converted"
drone_path = ds.Path(file=os.path.join(src_folder, "drone_path.csv"))
microphone_path = ds.Path(file=os.path.join(src_folder, "camera_path.csv"))
drone_sound = r"dynamic_sound/tests/resources/sounds/flying_drone.wav"

# path interpolation
#drone_path.interpolate_path(100)

# microphone
microphone = ds.microphones.Hedraphone_v2(
    "_tmp/simulation_sound_drone.wav",  # output file
    sample_rate=48_000  # Hz
)

# source
source = ds.sources.AudioFile(filename=drone_sound, sample_rate=48_000, gain_db=10.0, loop=True)
#source = ds.sources.SineWave(frequency=2_000, amplitude=1.0)
#source = ds.sources.WhiteNoise(duration=10.0, sample_rate=48_000, amplitude=1.0)

# simulation environment
sim = ds.Simulation(
    temperature=20,
    pressure=1,
    relative_humidity=50
)
sim.add_microphone(path=microphone_path, microphone=microphone)
sim.add_source(path=drone_path, source=source)

# reflection
# drone_path.positions[:,3] = -drone_path.positions[:,3]
# sim.add_source(
#     path=drone_path,
#     source=source
# )

sim.run()
