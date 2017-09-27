"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

n_groups = 1

inc_vhgp_RMSE = [ 0.000825611, 0.000762018, 0.0294435]
batch_vhgp_RMSE = [ 0.000733764, 0.00079533, 0.0292154]
input_vhgp_RMSE = [ 0.000900306, 0.000881479, 0.0377015]
inc_eals_RMSE = [ 0.000866729, 0.00076766, 0.0297085]
batch_eals_RMSE = [ 0.00073574, 0.000812739, 0.0293306]
input_eals_RMSE = [ 0.000900306, 0.000881479, 0.0377015]

plt.rcdefaults()
fig, ax = plt.subplots(nrows=2, ncols=1)

# Example data
#~ types = ('input', 'inc_vhgp', 'batch_vhgp', 'inc_eals', 'batch_eals')
#~ value1 = np.array([norm(input_vhgp_RMSE[0:2]), norm(inc_vhgp_RMSE[0:2]), norm(batch_vhgp_RMSE[0:2]),
         #~ norm(inc_eals_RMSE[0:2]), norm(batch_eals_RMSE[0:2])]) * 1000.0
#~ value2 = np.array([norm(input_vhgp_RMSE[2]), norm(inc_vhgp_RMSE[2]), norm(batch_vhgp_RMSE[2]),
         #~ norm(inc_eals_RMSE[2]), norm(batch_eals_RMSE[2])])
types = ('input', 'inc_eals', 'inc_vhgp', 'batch_vhgp', 'batch_eals')
value1 = np.array([norm(input_vhgp_RMSE[0:2]), norm(inc_eals_RMSE[0:2]), norm(inc_vhgp_RMSE[0:2]), 
         norm(batch_eals_RMSE[0:2]), norm(batch_vhgp_RMSE[0:2])]) * 1000.0
value2 = np.array([norm(input_vhgp_RMSE[2]), norm(inc_eals_RMSE[2]), norm(inc_vhgp_RMSE[2]),
         norm(batch_eals_RMSE[2]), norm(batch_vhgp_RMSE[2])])
y_pos = np.arange(len(types))
error = [0,0,0,0,0]

ax[0].barh(y_pos, value1, xerr=error, align='center',
        color='grey', ecolor='black')
ax[0].set_yticks(y_pos)
ax[0].set_yticklabels(types)
ax[0].invert_yaxis()  # labels read top-to-bottom
ax[0].set_xlabel('RMSE in position (mm)')
#ax[0].set_title('RMSE in position ')


ax[1].barh(y_pos, value2, xerr=error, align='center',
        color='grey', ecolor='black')
ax[1].set_yticks(y_pos)
ax[1].set_yticklabels(types)
ax[1].invert_yaxis()  # labels read top-to-bottom
ax[1].set_xlabel('RMSE in rotation (rad)')
#ax[1].set_title('RMSE in rotation ')
#fig.suptitle('RMSE of estimation')

plt.show()

