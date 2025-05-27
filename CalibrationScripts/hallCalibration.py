import numpy as np

# Measurement values
PosX_xa = np.array([501.75, 504.9, 508.8, 512.55, 513.75, 512.7, 510.3, 508.05, 507.6, 504.45, 502.35, 505.95, 505.5, 504.45])
PosX_ya = np.array([16.5, 49.2, 43.05, 13.8, 6, 3.3, -7.65, -10.5, -7.95, -14.7, -9.15, -13.2, -13.95, -12.3])
PosX_za = np.array([-77.7, -68.1, -65.7, -64.65, -63.15, -69.6, -71.55, -71.55, -72.45, -78.3, -77.4, -75.3, -76.05, -76.5])

NegX_xa = np.array([-536.55, -542.55, -543.75, -545.25, -546.3, -546.3, -547.2, -547.65, -547.65, -546.15, -545.55, -545.1, -544.65, -546.15, -540.9])
NegX_ya = np.array([-71.85, -61.5, -56.1, -54.3, -51, -50.7, -51.9, -50.85, -50.7, -51.9, -52.95, -51.45, -52.65, -51.75, -42.15])
NegX_za = np.array([-52.5, -53.7, -55.35, -56.55, -56.25, -59.1, -61.5, -63, -62.55, -65.1, -65.85, -65.4, -62.55, -62.7, -61.2])

PosY_xa = np.array([7.05, -3.9, -5.1, -9.3, -11.25, -13.5, -22.2, -25.95, -28.65, -29.25, -31.8, -31.5, -28.65, -28.2])
PosY_ya = np.array([461.1, 465.15, 469.8, 475.5, 478.5, 477.75, 477.15, 475.35, 474.45, 468.45, 455.25, 451.05, 451.5, 451.65])
PosY_za = np.array([-27.15, -30.15, -29.85, -28.8, -41.85, -76.2, -82.05, -85.5, -89.4, -86.7, -93, -88.65, -88.5, -86.85])

NegY_xa = np.array([-36, -33.6, -33.45, -33.15, -33.6, -33.75, -34.2, -34.5, -34.05, -34.35, -35.4, -33.45, -29.4, -30.3])
NegY_ya = np.array([-517.35, -515.7, -517.2, -517.35, -516.75, -517.35, -516.45, -515.7, -516.75, -515.7, -514.5, -514.5, -515.4, -515.7])
NegY_za = np.array([-114.3, -116.7, -118.05, -116.55, -117.15, -117.6, -117.9, -119.4, -119.1, -122.1, -122.25, -123.15, -123.45, -124.35])

PosZ_xa = np.array([3.9, -35.4, -42.75, -50.4, -54.3, -59.25, -60.45, -61.5, -61.35, -65.1, -68.4, -71.4])
PosZ_ya = np.array([29.4, 59.85, 70.8, 76.2, 81, 84.3, 81.75, 75.3, 72, 71.25, 71.55, 70.5])
PosZ_za = np.array([441.75, 453.6, 455.25, 453.75, 453.75, 452.25, 451.5, 452.25, 453.6, 454.05, 453, 452.4])

NegZ_xa = np.array([-36.75, -51.6, -56.55, -60.15, -63.75, -65.1, -66.45, -65.4, -66.15, -65.25, -60.15, -56.7, -58.5, -57.75])
NegZ_ya = np.array([30.9, 63.15, 70.8, 74.7, 77.25, 77.7, 79.35, 81.9, 81.15, 83.25, 87.15, 88.35, 87.9, 89.85])
NegZ_za = np.array([-550.2, -561, -567.45, -572.25, -574.05, -574.05, -575.85, -571.95, -570, -568.35, -569.85, -568.35, -569.25, -571.05])

# Means
meanPosX = [np.mean(PosX_xa), np.mean(PosX_ya), np.mean(PosX_za)]
meanNegX = [np.mean(NegX_xa), np.mean(NegX_ya), np.mean(NegX_za)]
meanPosY = [np.mean(PosY_xa), np.mean(PosY_ya), np.mean(PosY_za)]
meanNegY = [np.mean(NegY_xa), np.mean(NegY_ya), np.mean(NegY_za)]
meanPosZ = [np.mean(PosZ_xa), np.mean(PosZ_ya), np.mean(PosZ_za)]
meanNegZ = [np.mean(NegZ_xa), np.mean(NegZ_ya), np.mean(NegZ_za)]

# Reference matrix
measured = np.array([meanPosX, meanNegX, meanPosY, meanNegY, meanPosZ, meanNegZ])
reference = np.array([
    [507, 0, 0],
    [-507, 0, 0],
    [0, 507, 0],
    [0, -507, 0],
    [0, 0, 507],
    [0, 0, -507]
])

# Add bias term
measured_bias = np.hstack((measured, np.ones((6, 1))))

# Calculating the A matrix using least squares
A_T = np.linalg.lstsq(measured_bias, reference, rcond=None)[0]
A = A_T.T
print("A matrix:\n", A)

# Calculating the RMSE
ref_fitted = (A @ measured_bias.T).T
residuals = reference - ref_fitted
squared_errors = residuals ** 2
mse_per_axis = np.mean(squared_errors, axis=0)
rmse = np.sqrt(np.mean(squared_errors))
print("RMSE:", rmse)