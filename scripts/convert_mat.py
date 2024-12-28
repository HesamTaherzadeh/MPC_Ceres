import scipy.io
import pandas as pd
import numpy as np 

# Load the .mat file
mat_file = scipy.io.loadmat('Data.mat')
detector_size = mat_file['Detector_Size'][0][0]
ground_point = mat_file['Ground_Point']
image_point_left = mat_file['Image_Point_Left']
image_point_right = mat_file['Image_Point_Right']
num_columns = mat_file['Number_of_Columns'][0][0]
num_rows = mat_file['Number_of_Rows'][0][0]
principal_distance = mat_file['Principal_Distance'][0][0]

data_dict = {
    'Detector_Size': [detector_size],
    'Number_of_Columns': [num_columns],
    'Number_of_Rows': [num_rows],
    'Principal_Distance': [principal_distance]
}
detector_size = mat_file['Detector_Size'][0][0]
ground_point = mat_file['Ground_Point']
image_point_left = mat_file['Image_Point_Left']
image_point_right = mat_file['Image_Point_Right']
num_columns = mat_file['Number_of_Columns'][0][0]
num_rows = mat_file['Number_of_Rows'][0][0]
principal_distance = mat_file['Principal_Distance'][0][0]

num_rows_data = ground_point.shape[0]

scalar_data = {
    'Detector_Size': np.full(num_rows_data, detector_size),
    'Number_of_Columns': np.full(num_rows_data, num_columns),
    'Number_of_Rows': np.full(num_rows_data, num_rows),
    'Principal_Distance': np.full(num_rows_data, principal_distance)
}
df_scalars = pd.DataFrame(scalar_data)

df_ground_point = pd.DataFrame(ground_point, columns=['Ground_Point_X', 'Ground_Point_Y', 'Ground_Point_Z'])
df_image_point_left = pd.DataFrame(image_point_left, columns=['Image_Point_Left_X', 'Image_Point_Left_Y'])
df_image_point_right = pd.DataFrame(image_point_right, columns=['Image_Point_Right_X', 'Image_Point_Right_Y'])

df_combined = pd.concat([df_ground_point, df_image_point_left, df_image_point_right], axis=1)

df_combined.to_csv('combined_data.csv', index=False)