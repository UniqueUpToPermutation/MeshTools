import numpy as np
import sys
from scipy.sparse import csc_matrix
import scipy.sparse.linalg
import math

if len(sys.argv) < 2:
    print("No .pyz file specified!")
    exit()

# load data
filename = sys.argv[1]
fileContents = np.load(filename)
dim = fileContents.f.args[0]
loc = fileContents.f.args[1]
val = fileContents.f.args[2]

# build matrix
row_count = dim[0]
col_count = dim[1]
row = loc[:, 0]
col = loc[:, 1]

mat = csc_matrix((val, (row, col)), shape=(row_count, col_count))
mat = -mat;

if len(sys.argv) < 4:
    mode_count = 20
else:
    mode_count = int(sys.argv[3])

mode_count = min(mat.shape[0] - 2, mode_count)
    
# solve this bitch
print("Computing Eigenvectors...")
if ("-sym" in sys.argv):
    (val, modes) = scipy.sparse.linalg.eigsh(mat, k=mode_count, sigma=-0.0001)
else:
    (val, modes) = scipy.sparse.linalg.eigs(mat, k=mode_count, sigma=-0.0001)
val = np.real(val)
modes = np.real(modes)

if len(sys.argv) < 3:
    output_loc = "result"
else:
    output_loc = sys.argv[2]

# First row is the eigenvalues!
merge = np.concatenate((val.reshape(1, mode_count), modes), axis=0)

print("Eigenvalues:")
print(val)

# save result
np.save(output_loc, merge)