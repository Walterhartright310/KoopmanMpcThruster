import numpy as np

def lift_function(X, C, rbf_type, eps=1.0, k=1.0):
    """
    Calculates the Radial Basis Function (RBF) kernel for a given data set.

    Args:
    X: A 2D array of data points (number of points x number of features).
    C: A 2D array of centers (number of centers x number of features).
    rbf_type: String specifying the type of RBF ('thinplate', 'gauss', 'invquad', 'invmultquad', 'polyharmonic').
    eps: Float parameter for scaling distances (default: 1.0).
    k: Float parameter for polyharmonic RBF (default: 1.0).

    Returns:
    Y: A 2D array of RBF kernel values (number of centers x number of data points).

     
    Lifts the input data by appending the RBF kernel values.

    Args:
    x: A 1D or 2D array of data points.
    cent: A 2D array of centers.
    rbf_type: String specifying the type of RBF.

    Returns:
    lifted_x: A 2D array with the original data and RBF kernel values appended as a new row.
    """  
    X = np.atleast_2d(X)  # Ensure x is a 2D array
    rbf_type = rbf_type.lower()
    # Handle optional parameters
    if not np.all(np.isfinite(eps)):
        raise ValueError("eps must be finite")
    if not np.all(np.isfinite(k)):
        raise ValueError("k must be finite")

    C_big = C.copy()  # Avoid modifying the original C array创建副本

    Y = np.zeros((C.shape[1], X.shape[1])) #中心C 的 列向量 ，X 的列向量

    for i in range(C_big.shape[1]):
        center = C_big[:, i].reshape(-1, 1)
        center = np.repeat(center, X.shape[1], axis=1)  # Equivalent to repmat
        squared_distances = np.sum((X - center) ** 2, axis=0)  # Efficient distance calculation

        if rbf_type == 'thinplate':
            y = squared_distances * np.log(np.sqrt(squared_distances))
            y[np.isnan(y)] = 0  # Handle potential NaNs
        elif rbf_type == 'gauss':
            y = np.exp(-eps**2 * squared_distances)
        elif rbf_type == 'invquad':
            y = 1.0 / (1.0 + eps**2 * squared_distances)
        elif rbf_type == 'invmultquad':
            y = 1.0 / np.sqrt(1.0 + eps**2 * squared_distances)
        elif rbf_type == 'polyharmonic':
            y = squared_distances**(k / 2.0) * np.log(np.sqrt(squared_distances))
            y[np.isnan(y)] = 0  # Handle potential NaNs
        else:
            raise ValueError("Unsupported RBF type: {}".format(rbf_type))

        Y[i, :] = y
    lifted_Y= np.vstack((X, Y))
    return lifted_Y
# Example usage

cent = np.random.rand(2, 3)  # Example centers
#print('origina Center',cent)
X = np.array([[0.5,0.5], [1.5,0.2]])

rbf_type = 'gauss'  # Example RBF type
Y=lift_function(X,cent,rbf_type)
print("Lifted data point:", Y)
#data_point = np.array([[0.5], [1.5]])
#lifted_point = lift_function(data_point)

#print("Lifted data point:", lifted_point)

