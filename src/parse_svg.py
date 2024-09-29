import xml.etree.ElementTree as ET


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



svgFilePath = 'img/sze1px10x10px.svg'

pathDataList = extractSvgPaths(svgFilePath)

allCoordinates = []
#printing the paths to check if the extraction was successful
for pathData in pathDataList:
    print("Path: ", pathData)
    coordinates = extractCoordinates(pathData)
    allCoordinates.extend(coordinates)