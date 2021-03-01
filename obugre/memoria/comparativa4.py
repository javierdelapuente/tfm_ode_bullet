import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


COLUMN_TYPE = 0
COLUMN_GLOBAL_TIME = 1
COLUMN_ENGINE_NAME = 2
COLUMN_ENGINE_ELLAPSED_TIME = 3
COLUMN_BODY_NAME = 3
COLUMN_BODY_KINETIC_ENERGY = 10

data = pd.read_csv("./comparativa4.csv", header=None, sep=";")
print(data.head())

# convert to seconds
data[COLUMN_GLOBAL_TIME] = data[COLUMN_GLOBAL_TIME] / 1000000.0


dataode = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'ode')]
dataode[COLUMN_ENGINE_ELLAPSED_TIME] = dataode[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0
print(dataode)

databullet = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'bullet')]
databullet[COLUMN_ENGINE_ELLAPSED_TIME] = databullet[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0

print(dataode)
print(databullet)

### ahora rendimiento
fig, axs = plt.subplots(1)
fig.suptitle('Rendimiento')
axs.plot(dataode[COLUMN_GLOBAL_TIME], dataode[COLUMN_ENGINE_ELLAPSED_TIME], 'b--', label='ode')
axs.plot(databullet[COLUMN_GLOBAL_TIME], databullet[COLUMN_ENGINE_ELLAPSED_TIME], 'r', label='bullet')
axs.set_ylabel("Tiempo utilizado (s)")
axs.set_aspect('equal')
axs.legend()


#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()
