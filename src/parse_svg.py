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

#
#extracting paths one-by-one from the allPathsData list
def extractCoordinates(pathData):
    points = []
    i = 0
    while i < len(pathData):
        if pathData[i] in 'ML':  
            i += 1
            x, y = '', ''
            while pathData[i] != ',':
                x += pathData[i]
                i += 1
            i += 1  
            while i < len(pathData) and pathData[i] not in [' ', 'M', 'L', 'C']:
                y += pathData[i]
                i += 1
            points.append((float(x), float(y)))
        elif pathData[i] == 'C':  
            i += 1
            controlPoints = []
            for _ in range(3):
                x, y = '', ''
                while pathData[i] != ',':
                    x += pathData[i]
                    i += 1
                i += 1  # Skip comma
                while i < len(pathData) and pathData[i] not in [' ', 'M', 'L', 'C']:
                    y += pathData[i]
                    i += 1
                controlPoints.append((float(x), float(y)))
            p0 = points[-1]  
            points.extend(cubicBezierCurve(p0, controlPoints))
        i += 1
    return points
#turtlesim cannot handle bezier curves, so we need to convert them to line segments
def cubicBezierCurve(p0, controlPoints, numSteps=100):
    p1, p2, p3 = controlPoints
    points = []
    for i in range(numSteps + 1):
        t = i / numSteps
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points.append((x, y))
    return points



#turtlesim canvas is 11x11, so we need to scale the coordinates to fit the canvas
def convertCoordinates(coordinates, canvasSize=11):
    minX = min(coord[0] for coord in coordinates)
    minY = min(coord[1] for coord in coordinates)
    maxX = max(coord[0] for coord in coordinates)
    maxY = max(coord[1] for coord in coordinates)
    scale = min(canvasSize / (maxX - minX), canvasSize / (maxY - minY))
    return [( (x - minX) * scale, (y - minY) * scale) for x, y in coordinates]


#convert the coordinates to the format that turtlesim understands
def convertCrdToTurtlesimCrd(coordinates):
    movements = []
    for i in range(len(coordinates) - 1):
        p1, p2 = coordinates[i], coordinates[i + 1]
        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        movements.append((distance, angle))
    return movements



#main code
svgFilePath = 'img/sze1px10x10px.svg'

pathDataList = extractSvgPaths(svgFilePath)

allCoordinates = []
#printing the paths to check if the extraction was successful
for pathData in pathDataList:
    #print("Path: ", pathData)
    coordinates = extractCoordinates(pathData)
    allCoordinates.extend(coordinates)

convertedCoordinates = convertCoordinates(allCoordinates)
#print("Coordinates: ", convertedCoordinates)

turtlesimCoordinates = convertCrdToTurtlesimCrd(convertedCoordinates)
print("Turtlesim Coordinates: ", turtlesimCoordinates)