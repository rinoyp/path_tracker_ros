import math

#Calculate the bearing angle between two GPS coordinates 
def calcBearing(pointA, pointB):
	#convert latitude and longitude to radians
	latA = math.radians(pointA['lat'])
	latB = math.radians(pointB['lat'])
	deltaLon = math.radians(pointB['lon']) - math.radians(pointA['lon'])
	
	x = math.cos(latB)*math.sin(deltaLon)
	y = math.cos(latA)*math.sin(latB) - math.sin(latA)*math.cos(latB)*math.cos(deltaLon)
		
	bearing = math.degrees(math.atan2(x,y))
	return bearing
		
#Use haversine formula to calculate the great circle distance between points in metres
def getDistance(src, dst):
	earthRadius = 6371000.0
	phi1 = math.radians(src['lat'])
	phi2 = math.radians(dst['lat'])
	lam1 = math.radians(src['lon'])
	lam2 = math.radians(dst['lon'])
	deltaPhi = phi2 - phi1
	deltaLam = lam2 - lam1
	a = math.sin(deltaPhi/2.0)**2.0 + math.cos(phi1)*math.cos(phi2)*math.sin(deltaLam/2.0)**2.0
	c = 2.0* math.atan2(math.sqrt(a),math.sqrt(1.0-a))
		
	return earthRadius*c

#Returns a new GPS coordinate calculated from a distance and bearing from 
#a source point.
def getCoord(src,dist,bearing):
	earthRadius = 6371000.0

	theta = math.radians(bearing)
	lat1 = math.radians(src['lat'])
	lon1 = math.radians(src['lon'])
		
	sigma = dist/earthRadius
	a = math.sin(lat1)*math.cos(sigma) + math.cos(lat1)*math.sin(sigma)*math.cos(theta)
	lat2 = math.asin(a)
		
	y = math.sin(theta)*math.sin(sigma)*math.cos(lat1)
	x = math.cos(sigma) - math.sin(lat1)*math.sin(lat2)
	lon2 = lon1 + math.atan2(y,x)

	coord = {'lat':math.degrees(lat2), 'lon':math.degrees(lon2)}

	return coord

#Parses the GPS boundaries from a file, beginning at the start location and
#Going clockwise to the rest of corners that define the boundaries	
def parseGPSBounds(f):
	lats = []
	lons = []
	for line in f:
		coords = line.split(",")
		if len(coords) >= 2:
			lats.append(float(coords[0]))
			lons.append(float(coords[1]))

	nCoords = len(lats)
	bounds = [{'lat':lats[i], 'lon':lons[i]} for i in range(nCoords)]
	return bounds

#Returns the signed angular distance in degrees between two angles
#between -180 and 180 degrees(angle2 - angle1)
def angularDistance(angle1, angle2):
	theta1, theta2 = math.radians(angle1), math.radians(angle2)
	uv1 = {'x':math.cos(theta1), 'y':math.sin(theta1)}
	uv2 = {'x':math.cos(theta2), 'y':math.sin(theta2)}

	dot = uv1['x']*uv2['x'] + uv1['y']*uv2['y']
	det = uv1['x']*uv2['y'] - uv2['x']*uv1['y']

	return math.degrees(math.atan2(det,dot))	

#Convert a GPS coordinate into a coordinate in metres
def getDistCoord(gpsCoord,baseCoord, baseHeading):
	heading = calcBearing(baseCoord,gpsCoord)
	theta = math.radians(angularDistance(baseHeading,heading))
	dist = getDistance(baseCoord, gpsCoord)
	
	coord = dict()
	coord['x'], coord['y'] = dist*math.sin(theta), dist*math.cos(theta)
	return coord

#Convert a coordinate in metres back into a GPS coordinate
def getGPSCoord(distCoord, baseCoord, baseHeading):
	dist = math.sqrt(distCoord['x']**2 + distCoord['y']**2)
	heading = math.degrees(math.atan2(distCoord['x'], distCoord['y'])) + baseHeading
	coord = getCoord(baseCoord, dist, heading)
	return coord

def vectorDist(coord1, coord2):
	x, y = coord2['x'] - coord1['x'], coord2['y'] - coord1['y']
	return math.sqrt(x*x + y*y) 

def constrainAngle(angle):
	theta = math.radians(angle)
	newAngle = math.degrees(math.atan2(math.sin(theta),math.cos(theta)))
	return newAngle
	
def drawSquare(c,path_size,start_heading):
	lats, lons = [],[]
	heading = start_heading
	lats.append(c['lat'])
	lons.append(c['lon'])
	for i in range(4):
		for j in range(path_size):
			c = getCoord(c,1.0,heading)
			lats.append(c['lat'])
			lons.append(c['lon'])
					
		for j in range(4):
			c = getCoord(c,0.5,heading)
			heading = constrainAngle(heading+90.0/4.0)
			lats.append(c['lat'])
			lons.append(c['lon'])
			
	goal = [{'lat':lats[i],'lon':lons[i]} for i in range(len(lats))]
	return goal
	
def drawCircle(c, radius, start_heading):
	lats, lons = [],[]
	heading = start_heading
	dist = 2.0*math.pi*radius
	curvature = 360.0/dist
	lats.append(c['lat'])
	lons.append(c['lon'])
	for i in range(int(dist)):
		c = getCoord(c,1.0,heading)
		heading = constrainAngle(heading+curvature)
		lats.append(c['lat'])
		lons.append(c['lon'])
	
	goal = [{'lat':lats[i],'lon':lons[i]} for i in range(len(lats))]
	return goal

def drawLine(c, length, start_heading):
	goal = [getCoord(c,float(i),start_heading) for i in range(int(length))]
	return goal