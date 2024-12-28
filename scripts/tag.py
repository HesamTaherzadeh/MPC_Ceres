import pandas as pd
import matplotlib.pyplot as plt
import argparse

# Set up argument parsing
parser = argparse.ArgumentParser(description="Tag points in a CSV file based on user input.")
parser.add_argument("input_file", help="Path to the input CSV file")
parser.add_argument("--output_file", help="Path to save the tagged CSV file (optional)", default=None)
args = parser.parse_args()

# Read the CSV file into a DataFrame
try:
    df = pd.read_csv(args.input_file)
except FileNotFoundError:
    print(f"Error: The file '{args.input_file}' was not found.")
    exit()
except Exception as e:
    print(f"An error occurred while reading the file: {e}")
    exit()

# Plot the points
plt.figure(figsize=(10, 6))
plt.scatter(df['Image_Point_Left_X'], df['Image_Point_Left_Y'], c='blue', label='Left Image Points')
plt.title('Left Image Points')
plt.xlabel('Image_Point_Left_X')
plt.ylabel('Image_Point_Left_Y')
plt.legend()
plt.grid(True)

# Use ginput to pick points
print("Please click on the points you want to tag. Press 'Enter' when done.")
picked_points = plt.ginput(n=-1, timeout=0)  # n=-1 allows unlimited points, timeout=0 waits indefinitely

# Close the plot
plt.close()

# Tag the points
df['Tag'] = 0  # Initialize the tag column with 0

# Check which points were picked
for (x_picked, y_picked) in picked_points:
    # Find the closest point in the DataFrame
    distances = ((df['Image_Point_Left_X'] - x_picked) ** 2 + (df['Image_Point_Left_Y'] - y_picked) ** 2) ** 0.5
    closest_index = distances.idxmin()
    df.at[closest_index, 'Tag'] = 1

# Print the updated DataFrame
print(df)

# Save the DataFrame to a new CSV file if an output file is provided
if args.output_file:
    try:
        df.to_csv(args.output_file, index=False)
        print(f"Tagged data saved to '{args.output_file}'.")
    except Exception as e:
        print(f"An error occurred while saving the file: {e}")