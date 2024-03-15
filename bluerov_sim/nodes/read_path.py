import yaml
# print('/home/kanzhoong/mallard_ws/setpoints.yaml')

# Read the YAML file into a dictionary
with open('/home/kanzhoong/mallard_ws/setpoints.yaml', 'r') as infile:
  setpoints = yaml.safe_load(infile)

# Extract the setpoints from the dictionary

x=[setpoints[i]['x'] for i in range(len(setpoints))]
y=[setpoints[i]['y'] for i in range(len(setpoints))]
yaw=[setpoints[i]['yaw'] for i in range(len(setpoints))]

print(yaw[0])