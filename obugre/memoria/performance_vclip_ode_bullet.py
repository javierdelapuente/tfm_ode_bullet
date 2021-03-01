import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import fileinput
import sys


COLUMN_TYPE = 0


COLUMN_GLOBAL_TIME = 1
COLUMN_ENGINE_NAME = 2
COLUMN_ENGINE_ELLAPSED_TIME = 3
COLUMN_BODY_NAME = 3
COLUMN_CONVEX_TOTAL_TIME = 4
COLUMN_CONVEX_TOTAL_TESTS = 5
COLUMN_BODY_KINETIC_ENERGY = 10

# fi = fileinput.input(openhook=fileinput.hook_encoded('utf-8'))
filename = sys.argv[1]

data = pd.read_csv(filename, header=None, sep=";")
#print(data.head())

# convert to seconds
data[COLUMN_GLOBAL_TIME] = data[COLUMN_GLOBAL_TIME] / 1000000.0


ode = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'ode')]
odevclip = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'odevclip')]
bullet = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'bullet')]
bulletvclip = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'bulletvclip')]


ode[COLUMN_ENGINE_ELLAPSED_TIME] = ode[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0
odevclip[COLUMN_ENGINE_ELLAPSED_TIME] = odevclip[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0
bullet[COLUMN_ENGINE_ELLAPSED_TIME] = bullet[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0
bulletvclip[COLUMN_ENGINE_ELLAPSED_TIME] = bulletvclip[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0

ode['time_step'] = ode[COLUMN_ENGINE_ELLAPSED_TIME].diff()
odevclip['time_step'] = odevclip[COLUMN_ENGINE_ELLAPSED_TIME].diff()
bullet['time_step'] = bullet[COLUMN_ENGINE_ELLAPSED_TIME].diff()
bulletvclip['time_step'] = bulletvclip[COLUMN_ENGINE_ELLAPSED_TIME].diff()


# ODE RENDIMIENTO MPR VS VCLIP
fig, axs = plt.subplots(2, sharex=True) # sharex = True
#fig.suptitle('Rendimiento ODE.')
### ahora rendimiento
axs[0].set_title("Tiempo total utilizado por ODE.")
axs[0].plot(ode[COLUMN_GLOBAL_TIME], ode[COLUMN_ENGINE_ELLAPSED_TIME], 'b--', label='ODE con MPR (libCCD)')
axs[0].plot(odevclip[COLUMN_GLOBAL_TIME], odevclip[COLUMN_ENGINE_ELLAPSED_TIME], 'r', label='ODE con V-Clip')
axs[0].set_ylabel("Tiempo total (s)")
axs[0].legend()
#axs.set_title("XXXX")
### ahora rendimiento
axs[1].set_title("Tiempo por paso de integración.")
axs[1].plot(ode[COLUMN_GLOBAL_TIME], ode['time_step'], 'b--', label='ODE con MPR (libCCD)')
axs[1].plot(odevclip[COLUMN_GLOBAL_TIME], odevclip['time_step'], 'r', label='ODE con V-Clip')
axs[1].set_ylabel("Tiempo total (s)")
axs[1].legend()
#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()




# BULLET RENDIMIENTO GJK/EPA VS VCLIP
fig, axs = plt.subplots(2, sharex=True) # sharex = True
#fig.suptitle('Rendimiento Bullet.')
### ahora rendimiento
axs[0].set_title("Tiempo total utilizado por Bullet.")
axs[0].plot(bullet[COLUMN_GLOBAL_TIME], bullet[COLUMN_ENGINE_ELLAPSED_TIME], 'b--', label='Bullet con GJK/EPA')
axs[0].plot(bulletvclip[COLUMN_GLOBAL_TIME], bulletvclip[COLUMN_ENGINE_ELLAPSED_TIME], 'r', label='Bullet con V-Clip')
axs[0].set_ylabel("Tiempo total (s)")
axs[0].legend()
#axs.set_title("XXXX")
### ahora rendimiento
axs[1].set_title("Tiempo por paso de integración.")
axs[1].plot(bullet[COLUMN_GLOBAL_TIME], bullet['time_step'], 'b--', label='Bullet con GJK/EPA')
axs[1].plot(bulletvclip[COLUMN_GLOBAL_TIME], bulletvclip['time_step'], 'r', label='Bullet con V-Clip')
axs[1].set_ylabel("Tiempo total (s)")
axs[1].legend()
#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()
