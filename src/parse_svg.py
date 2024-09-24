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



svgFilePath = '/home/ajr/ros2_ws/src/nov_fij_draw_logo/img/sze1px10x10px.svg'


pathDataList = extractSvgPaths(svgFilePath)

print(pathDataList)