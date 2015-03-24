from pylab import *
from matplotlib import rc, rcParams

rc('text',usetex=True)
rc('font',**{'family':'serif','serif':['Computer Modern']})


#  Import the data from a text file and save as a 2D matrix.
#dataMatrix1 = genfromtxt('winavrg.dat')
dataMatrix1 = genfromtxt('data.txt')

# Slice out required abscissae and ordinate vectors.

xl = dataMatrix1[:,1]
yl = dataMatrix1[:,2]
xr = dataMatrix1[:,4]
yr = dataMatrix1[:,5]
#y2 = dataMatrix1[:,2]
#print x
plot(xl,yl,label=r'left leg')
plot(xr,yr,label=r'right leg')

legend(loc='upper right')

xlabel(r'$x$',fontsize=16)
plt.ylabel(r'$z$',fontsize=16)

show()
