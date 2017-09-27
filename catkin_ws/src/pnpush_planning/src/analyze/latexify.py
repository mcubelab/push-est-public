from math import sqrt
import matplotlib
def latexify(fig_width=None, fig_height=None, columns=1, fontsize=7, scale = 1):
    """Set up matplotlib's RC params for LaTeX plotting.
    Call this before plotting a figure.

    Parameters
    ----------
    fig_width : float, optional, inches
    fig_height : float,  optional, inches
    columns : {1, 2}
    """
    
    fontsize *= scale
    
    # code adapted from http://www.scipy.org/Cookbook/Matplotlib/LaTeX_Examples

    # Width and max height in inches for IEEE journals taken from
    # computer.org/cms/Computer.org/Journal%20templates/transactions_art_guide.pdf

    
    assert(columns in [1,2])

    if fig_width is None:
        fig_width = 3.39 if columns==1 else 6.9 # width in inches

    fig_width *= scale
    if fig_height is None:
        golden_mean = (sqrt(5)-1.0)/2.0    # Aesthetic ratio
        fig_height = fig_width*golden_mean # height in inches

    fig_height *= scale
    
    #MAX_HEIGHT_INCHES = 8.0
    # if fig_height > MAX_HEIGHT_INCHES:
        # print("WARNING: fig_height too large:" + fig_height + 
              # "so will reduce to" + MAX_HEIGHT_INCHES + "inches.")
        # fig_height = MAX_HEIGHT_INCHES
        
    params = {'backend': 'ps',
              'text.latex.preamble': ['\usepackage{gensymb}'],
              'axes.labelsize': fontsize, # fontsize for x and y labels (was 10)
              'axes.titlesize': fontsize,
              'text.fontsize': fontsize, # was 10
              'legend.fontsize': fontsize, # was 10
              'xtick.labelsize': fontsize,
              'ytick.labelsize': fontsize,
              'text.usetex': True,
              'figure.figsize': [fig_width,fig_height],
              'font.family': 'serif',
              'axes.linewidth': 0.25,
              'legend.linewidth': 0.25,
              'grid.major.linewidth': 0.25,
              'xtick.major.width': 0.25,
              'ytick.major.width': 0.25
              
    }

    matplotlib.rcParams.update(params)
    
