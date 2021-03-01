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


print("-- with cache")
datavclip = data.loc[(data[COLUMN_TYPE] == 2) & (data[COLUMN_ENGINE_NAME] == 'odevclip')]
print(datavclip)
datavclip = datavclip.copy()
print(datavclip)
datavclip['time_convex_frame'] = datavclip[COLUMN_CONVEX_TOTAL_TIME].diff()
datavclip['tests_convex'] = datavclip[COLUMN_CONVEX_TOTAL_TESTS].diff()
datavclip['time_per_test'] = datavclip['time_convex_frame']/datavclip['tests_convex']
datavclip['time_ode'] = datavclip[COLUMN_ENGINE_ELLAPSED_TIME].diff()

print(datavclip)

print("-- nocache")
datavclipnocache = data.loc[(data[COLUMN_TYPE] == 2) & (data[COLUMN_ENGINE_NAME] == 'odevclipnocache')]
print(datavclipnocache)
datavclipnocache = datavclipnocache.copy()
print(datavclipnocache)
datavclipnocache['time_convex_frame'] = datavclipnocache[COLUMN_CONVEX_TOTAL_TIME].diff()
datavclipnocache['tests_convex'] = datavclipnocache[COLUMN_CONVEX_TOTAL_TESTS].diff()
datavclipnocache['time_per_test'] = datavclipnocache['time_convex_frame']/datavclipnocache['tests_convex']
datavclipnocache['time_ode'] = datavclipnocache[COLUMN_ENGINE_ELLAPSED_TIME].diff()

print(datavclipnocache)

fig, axs = plt.subplots(1)
fig.suptitle('Tiempo Medio V-Clip por paso integración')
axs.plot(datavclip[COLUMN_GLOBAL_TIME], datavclip['time_per_test'], 'b--', label='V-Clip con caché')
axs.plot(datavclipnocache[COLUMN_GLOBAL_TIME], datavclipnocache['time_per_test'], 'r', label='V-Clip sin caché')
#axs.set_ylabel("Tiempo medio test V-Clip por integración (s)")
axs.set_ylabel(r"Tiempo medio test V-Clip por paso integración ($\mu s$)")

axs.legend()
#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()



fig, axs = plt.subplots(1)
fig.suptitle('Tiempo ODE por paso integración')
axs.plot(datavclip[COLUMN_GLOBAL_TIME], datavclip['time_ode'], 'b--', label='V-Clip con caché')
axs.plot(datavclipnocache[COLUMN_GLOBAL_TIME], datavclipnocache['time_ode'], 'r', label='V-Clip sin caché')
#axs.set_ylabel("Tiempo medio test V-Clip por integración (s)")
axs.set_ylabel(r"Tiempo medio test V-Clip por paso integración ($\mu s$)")

axs.legend()
#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()



fig, axs = plt.subplots(1)
fig.suptitle('Pruebas V-Clip por paso integración')
axs.plot(datavclip[COLUMN_GLOBAL_TIME], datavclip['tests_convex'], 'b--', label='V-Clip con caché')
axs.plot(datavclipnocache[COLUMN_GLOBAL_TIME], datavclipnocache['tests_convex'], 'r', label='V-Clip sin caché')
axs.set_ylabel(r"Número pruebas V-Clip por paso integración")

axs.legend()
#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()

datavclipnocache = data[(data[COLUMN_TYPE] == 2) & (data[COLUMN_ENGINE_NAME] == 'odevclipnocache')]
dataodevclip = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'odevclip')]
dataodevclip[COLUMN_ENGINE_ELLAPSED_TIME] = dataodevclip[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0
print(dataodevclip)

dataodenocache = data[(data[COLUMN_TYPE] == 0) & (data[COLUMN_ENGINE_NAME] == 'odevclipnocache')]
dataodenocache[COLUMN_ENGINE_ELLAPSED_TIME] = dataodenocache[COLUMN_ENGINE_ELLAPSED_TIME].astype(float) / 1000000.0

print(dataodevclip)
print(dataodenocache)

### ahora rendimiento
fig, axs = plt.subplots(1)
fig.suptitle('Rendimiento')
axs.plot(dataodevclip[COLUMN_GLOBAL_TIME], dataodevclip[COLUMN_ENGINE_ELLAPSED_TIME], 'b--', label='V-Clip con caché')
axs.plot(dataodenocache[COLUMN_GLOBAL_TIME], dataodenocache[COLUMN_ENGINE_ELLAPSED_TIME], 'r', label='V-Clip sin caché')
axs.set_ylabel("Tiempo total (s)")
axs.legend()


#axs.set_title("XXXX")
plt.xlabel("Tiempo Global (s)")
plt.show()
