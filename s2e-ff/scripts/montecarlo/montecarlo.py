import subprocess
import numpy as np
import pathlib
from make_ini import ff_simulation_base,position,velocity

for i in range(100):
    print(pathlib.Path(__file__))
    ff_simulation_base.echo()
    position.echo()
    velocity.echo()
    print("cd {} && ./S2E_FF".format(pathlib.Path(__file__).parent.parent.parent/"build/build"))
    subprocess.run("cd {} && ./S2E_FF".format(pathlib.Path(__file__).parent.parent.parent/"build/build"),shell=True)