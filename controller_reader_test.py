from controller_reader import ControllerReader
import itertools


with ControllerReader() as reader:
    for point in itertools.islice(reader.iter_readings(), 100):
        print(point)

    for point in reader.read_for(0.1):
        print(point)

    for point in reader.read_for(5):
        print(point)
        
