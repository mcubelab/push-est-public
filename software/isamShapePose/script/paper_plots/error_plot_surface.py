"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

n_groups = 1

abs_RMSE = [0.000799204, 0.000740111, 0.0278592]
delrin_RMSE = [0.000705293, 0.000784489, 0.02685548]
plywood_RMSE = [0.000805525, 0.000739883, 0.0326061]
pu_RMSE = [0.000854511, 0.000725197, 0.0287349]

plt.rcdefaults()
fig, ax = plt.subplots(nrows=2, ncols=1)

# Example data
#~ types = ('input', 'inc_vhgp', 'batch_vhgp', 'inc_eals', 'batch_eals')
#~ value1 = np.array([norm(input_vhgp_RMSE[0:2]), norm(inc_vhgp_RMSE[0:2]), norm(batch_vhgp_RMSE[0:2]),
         #~ norm(inc_eals_RMSE[0:2]), norm(batch_eals_RMSE[0:2])]) * 1000.0
#~ value2 = np.array([norm(input_vhgp_RMSE[2]), norm(inc_vhgp_RMSE[2]), norm(batch_vhgp_RMSE[2]),
         #~ norm(inc_eals_RMSE[2]), norm(batch_eals_RMSE[2])])
types = ('abs', 'delrin', 'plywood', 'pu')
value1 = np.array([norm(abs_RMSE[0:2]), norm(delrin_RMSE[0:2]), norm(plywood_RMSE[0:2]), 
         norm(pu_RMSE[0:2])]) * 1000.0
value2 = np.array([norm(abs_RMSE[2]), norm(delrin_RMSE[2]), norm(plywood_RMSE[2]),
         norm(pu_RMSE[2])])
y_pos = np.arange(len(types))
#error = [0,0,0,0]

ax[0].barh(y_pos, value1, xerr=None, align='center',
        color='grey', ecolor='black')
ax[0].set_yticks(y_pos)
ax[0].set_yticklabels(types)
ax[0].invert_yaxis()  # labels read top-to-bottom
ax[0].set_xlabel('RMSE in position (mm)')
#ax[0].set_title('RMSE in position ')


ax[1].barh(y_pos, value2, xerr=None, align='center',
        color='grey', ecolor='black')
ax[1].set_yticks(y_pos)
ax[1].set_yticklabels(types)
ax[1].invert_yaxis()  # labels read top-to-bottom
ax[1].set_xlabel('RMSE in rotation (rad)')
#ax[1].set_title('RMSE in rotation ')
#fig.suptitle('RMSE of estimation')

plt.show()

