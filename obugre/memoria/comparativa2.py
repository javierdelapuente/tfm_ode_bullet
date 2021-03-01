import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


COLUMN_TYPE = 0
COLUMN_GLOBAL_TIME = 1
COLUMN_ENGINE_NAME = 2
COLUMN_ENGINE_ELLAPSED_TIME = 3
COLUMN_BODY_NAME = 3
COLUMN_BODY_KINETIC_ENERGY = 10

data = pd.read_csv("./comparativa2.csv", header=None, sep=";")
print(data.head())

# convert to seconds
data[COLUMN_GLOBAL_TIME] = data[COLUMN_GLOBAL_TIME] / 1000000.0


# sphere1. - maciza
body1data = data[(data[COLUMN_BODY_NAME] == 'sphere1') ]
body1dataode = body1data[(body1data[COLUMN_ENGINE_NAME] == 'ode')]
body1databullet = body1data[(body1data[COLUMN_ENGINE_NAME] == 'bullet')]

# sphere2. - hueca
body2data = data[(data[COLUMN_BODY_NAME] == 'sphere2') ]
body2dataode = body2data[(body2data[COLUMN_ENGINE_NAME] == 'ode')]
body2databullet = body2data[(body2data[COLUMN_ENGINE_NAME] == 'bullet')]

fig, axs = plt.subplots(2, sharex=True) # sharex = True
fig.suptitle('Energía cinética')

axs[0].set_title("Esfera Maciza")
axs[0].plot(body1dataode[COLUMN_GLOBAL_TIME], body1dataode[COLUMN_BODY_KINETIC_ENERGY], 'b--', label='ode')
axs[0].plot(body1databullet[COLUMN_GLOBAL_TIME], body1databullet[COLUMN_BODY_KINETIC_ENERGY], 'r', label='bullet')
axs[0].set_ylabel("Energía Cinética")
axs[0].set_ylim(ymin=0.0, ymax=300)
axs[0].legend()

axs[1].set_title("Esfera Hueca")
axs[1].plot(body2dataode[COLUMN_GLOBAL_TIME], body2dataode[COLUMN_BODY_KINETIC_ENERGY], 'b--' ,label='ode')
axs[1].plot(body2databullet[COLUMN_GLOBAL_TIME], body2databullet[COLUMN_BODY_KINETIC_ENERGY], 'r' , label='bullet')
axs[1].set_ylabel("Energía Cinética")
axs[1].set_ylim(ymin=0.0, ymax=300)
axs[1].legend()

plt.xlabel("Tiempo Global (s)")
fig.tight_layout()
plt.show()

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
axs.legend()


#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()
