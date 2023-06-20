from movementLogic import calculateCommand, getDistance, distanceToBall, calculateCommandToGoal 
from BallAnalysis import findRobot																

# Define constants
safetydistance = 50


side, safe = getRobotSide(x_limits)
print(side)
print(safe)


print("getsafe says:")	
print(getSafe(side))

print("avoidObstacle Says:")


def getRobotSide(x_limits): # x_limits is list [lo_x, hi_x, lo_y, hi_y]. Returns side as char (w,e,n,s) and safe as bool (0,1)
	robotXY, robotHeading = findRobot(frame, debugFrame) 
	
	side = 'x'
	safe = 2

	# West of X
	if robotXY[0] < x_limits[0]:						
		side = 'w'										# QDPI west of X
		if robotXY[0] < (x_limits[0]-safetydistance):	
			safe = 1									# QDPI is clear to go around
		else: 											# QDPI is too close to X, get clear
			safe = 0

	# East of X
	elif robotXY[0] > x_limits[1]:						
		side = 'e'										
		if robotXY[0] > (x_limits[1]+safetydistance): 	# QDPI is clear
			safe = 1
		else: 											# QDPI is too close to X, get clear
			safe = 0
	
	# North of X
	elif robotXY[1] < x_limits[2]:
		side = 'n'
		if robotXY[1] < (x_limits[2]-safetydistance): 	# QDPI is clear
			safe = 1
		else:
			safe = 0

	# South of X
	elif robotXY[1] > x_limits[3]:
		side = 's'
		if robotXY[1] > (x_limits[3]+safetydistance):
			safe = 1
		else:
			safe = 0

	#Something went wrong
	else: 
		print("Side and safe not set! \n")

	return side, safe

def getSafe(side):
	robotXY, robotHeading = findRobot(frame, debugFrame) 
	if side == 'w':
		if robotHeading < 90 or robotHeading > 270: #facing X
			return -1 	# reverse 100
		else:
			return 3, 0	# forward 100

	elif side == 'e':
		if robotHeading > 90 and robotHeading < 270: #facing X
			return -1 	# reverse 100
		else:
			return 3, 0	# forward 100
	
	elif side == 'n': 		#facing X						
		if robotHeading > 180:
			return -1
		else:
			return 3, 0

	elif side == 's':
		if robotHeading < 180: #facing X
			return -1
		else:
			return 3, 0
	
	else: 
		print("error in getSafe!\n")

def avoidObstacle(x_limits, courselimits):
	robotXY, robotHeading = findRobot(frame, debugFrame)
	side, safe = getRobotSide(x_limits)
	if safe == 2:
		print("Error with safe in avoidObstacle")
	else:
		if safe == 0:
			return getSafe(side)
	
	if side == 'e':
		if robotHeading < 88 or robotHeading > 92:	# Heading is not north
			desired_heading = 90
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = robotXY[1]-(course_limits[2]+safetydistance) #drive north
			return 3, manualDistance
	
	elif side == 'n':
		if robotHeading < 178 or robotheading > 182: # Heading is not west
			desired_heading = 180
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = robotXY[0]-(course_limits[0]+safetydistance) #drive west
			return 3, manualDistance
	
	elif side == 'w':
		if robotHeading < 268 or robotHeading > 272: # heading is not south
			desired_heading = 270
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = (course_limits[3]-safetydistance)-robotXY[1] #drive south
			return 3, manualDistance
	
	elif side == 's':
		if robotHeading > 2 and robotHeading < 358: # heading is not east
			desired_heading = 0
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = (course_limits[1]-safetydistance)-robotXY[0] #drive east
			return 3, manualDistance
	
	else:
		print("error with side in avoidObstacle!\n")




	


