%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 


Camera.fx: 817.601449
Camera.fy: 817.601449
Camera.cx: 638.707832
Camera.cy: 471.546653

#Camera.k1: 0.15269
#Camera.k2: -0.15797
#Camera.p1: 0.002668
#Camera.p2: -3.24e-04

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

Camera.width: 1280
Camera.height: 960

# Camera frames per second 
Camera.fps: 30

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# stereo baseline times fx
Camera.bf: 4915.698
#Camera.bf: 49.15698
#Camera.bf: 45834


# Close/Far threshold. Baseline times.
#ThDepth: 35
ThDepth: 10

#--------------------------------------------------------------------------------------------
# Stereo Rectification. Only if you need to pre-rectify the images.
# Camera.fx, .fy, etc must be the same as in LEFT.P
#--------------------------------------------------------------------------------------------
# 相机拍摄的图像尺寸
LEFT.height: 960
LEFT.width: 1280
# 畸变系数
LEFT.D: !!opencv-matrix
   rows: 1
   cols: 14
   dt: d
   data: [1.7632203437036997e-01, -2.8421287189530248e-01, 0., 0., 0.,
       0., 0., -1.6661584336541882e-01, 0., 0., 0., 0., 0., 0. ]
# 相机内参
LEFT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 7.6593829041646700e+02, 0., 6.4520686017202013e+02, 0.,
       7.6297446022710346e+02, 4.6897279384400093e+02, 0., 0., 1. ]
# 在立体校正过程中，相机为实现共面过程中所需要进行的旋转
LEFT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [9.9998719530327329e-01, -1.2193737173703777e-03,
       4.9114516418886584e-03, 1.2212163617061162e-03,
       9.9999918505169516e-01, -3.7219154140515239e-04,
       -4.9109937987260547e-03, 3.7818472071015252e-04,
       9.9998786948453822e-01]
# 在立体校正过程后，相机在新坐标系下的投影矩阵
LEFT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 8.1760144944695116e+02, 0., 6.3870783233642578e+02, 0., 0.,
       8.1760144944695116e+02, 4.7154665374755859e+02, 0., 0., 0., 1.,
       0. ]

RIGHT.height: 960
RIGHT.width: 1280
RIGHT.D: !!opencv-matrix
   rows: 1
   cols: 14
   dt: d
   data: [ 1.6319713796675256e-01, -2.3834704175019850e-01, 0., 0., 0.,
       0., 0., -1.1499955926412903e-01, 0., 0., 0., 0., 0., 0. ]
RIGHT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 7.6593829041646700e+02, 0., 6.4951092679334840e+02, 0.,
       7.6297446022710346e+02, 4.7218896426390540e+02, 0., 0., 1.]
RIGHT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 9.9996590346168390e-01, 3.8767445054260488e-03,
       7.2912801412478292e-03, -3.8794798664078737e-03,
       9.9999240961094904e-01, 3.6104910193603590e-04,
       -7.2898251024727610e-03, -3.8932316591979885e-04,
       9.9997335308369484e-01 ]

RIGHT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [  8.1760144944695116e+02, 0., 6.3870783233642578e+02,
       -4.9156982182679254e+04, 0., 8.1760144944695116e+02,
       4.7154665374755859e+02, 0., 0., 0., 1., 0. ]
#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1200

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize:2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500

Custom.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 9.9998416180313698e-01, -5.1015657462820966e-03,
       -2.3770085851704797e-03, 5.0998139207723480e-03,
       9.9998672023239810e-01, -7.4246671639397041e-04,
       2.3807647618169963e-03, 7.3033265558748750e-04,
       9.9999689928187330e-01 ]

Custom.T:  !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ -6.0121353910273491e+01, -2.3308307575657894e-01,
       -4.3837658095538801e-01 ]

Custom.Q:  !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1., 0., 0., -6.3870783233642578e+02, 0., 1., 0.,
       -4.7154665374755859e+02, 0., 0., 0., 8.1760144944695116e+02, 0.,
       0., 1.6632458160440896e-02, 0. ]