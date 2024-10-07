import xml.etree.ElementTree as ET
import math

#extracting svg paths from a svg file
#idea: https://docs.python.org/3/library/xml.etree.elementtree.html and also a little help from copilot and chatgpt

def extractSvgPaths(filePath):
    tree = ET.parse(filePath)
    root = tree.getroot()
    namespace = {'svg': 'http://www.w3.org/2000/svg'}
    pathElements = root.findall('.//svg:path', namespace)
    allPathsData = [path.get('d') for path in pathElements if path.get('d')]
    return allPathsData





def svg_path_to_turtlesim(svg_path):
    commands = []
    x, y = 0, 0  # Starting position
    angle = 0    # Starting angle

    # Split the SVG path into commands
    path_commands = svg_path.replace(',', ' ').split()
    
    i = 0
    while i < len(path_commands):
        command = path_commands[i]
        
        if command == 'M' or command == 'm':
            # Move to absolute or relative coordinates
            dx = float(path_commands[i + 1])
            dy = float(path_commands[i + 2])
            if command == 'm':  # Relative move
                dx += x
                dy += y
            x, y = dx, dy
            commands.append(f"forward {math.hypot(dx, dy)}")  # Move forward
            angle = math.degrees(math.atan2(dy - y, dx - x))  # Calculate angle
            commands.append(f"setheading {angle}")  # Set heading
            i += 3
            
        elif command == 'L' or command == 'l':
            # Line to absolute or relative coordinates
            dx = float(path_commands[i + 1])
            dy = float(path_commands[i + 2])
            if command == 'l':  # Relative line
                dx += x
                dy += y
            distance = math.hypot(dx - x, dy - y)
            commands.append(f"forward {distance}")  # Move forward
            angle = math.degrees(math.atan2(dy - y, dx - x))  # Calculate angle
            commands.append(f"setheading {angle}")  # Set heading
            x, y = dx, dy  # Update current position
            i += 3
            
        # Handle additional SVG commands (like C, Q, etc.) here
        
        else:
            i += 1  # Skip unknown commands

    return commands





#main code
svgFilePath = 'img/sze1px10x10px.svg'

pathDataList = extractSvgPaths(svgFilePath)

allPaths = []
#printing the paths to check if the extraction was successful
for pathData in pathDataList:
    #print("Path: ", pathData)
    allPaths.append(svg_path_to_turtlesim(pathData))

print(allPaths)
# # i have the above code, now i'm going to list the functions i'll need:
# # - most important thing is that the paths must stay in a list, one path is one element, so if you do any modifications, please keep that in mind
# # - the paths are already extracted from the svg file, so you don't need to worry about that
# # - i need you to go through the paths and do the following things:
# # - convert each path to a format that turtlesim can understand (it's important that svg uses uppercase letters for absolute coordinates and lowercase letters for relative coordinates)
# # - turtlesim uses a canvas that is 11x11, so you need to scale the paths to fit that canvas
# # - you need to handle bezier curves too, so you need to convert them to lines, but in a way that the path still looks like the original
