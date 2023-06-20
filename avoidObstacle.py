#from movementLogic import calculateCommand, getDistance, distanceToBall, calculateCommandToGoal 	#TODO import, outcmtd for test
#from BallAnalysis import findRobot																	#TODO import, outcmtd for test

# Define constants
safetydistance = 50

x_limits = [53,54,55,56] #TODO delete, is a test


side, safe = getRobotSide(x_limits)
print(side)
print(safe)


print("getsafe says:")	
print(getSafe(side))



def getRobotSide(x_limits): # x_limits is list [lo_x, hi_x, lo_y, hi_y]. Returns side as char (w,e,n,s) and safe as bool (0,1)
	#robotXY, robotHeading = findRobot(frame, debugFrame) #TODO outcmtd for test
	robotXY = [53,200] #TODO delete, is a test
	
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
	#robotXY, robotHeading = findRobot(frame, debugFrame) #TODO outcmtd for test
	robotHeading = 100 #TODO delete, is a test
	if side == 'w':
		if robotHeading < 90 or robotHeading > 270: #facing X
			return -1 	# reverse 100
		else:
			return 3	# forward 100

	elif side == 'e':
		if robotHeading > 90 and robotHeading < 270: #facing X
			return -1 	# reverse 100
		else:
			return 3	# forward 100
	
	elif side == 'n': 		#facing X						
		if robotHeading > 180:
			return -1
		else:
			return 3

	elif side == 's':
		if robotHeading < 180: #facing X
			return -1
		else:
			return 3
	
	else: 
		print("error in getSafe!\n")

def avoidObstacle(x_limits):
	robotXY, robotHeading = findRobot(frame, debugFrame)
	side, safe = getRobotSide(x_limits)
	if safe == 2:
		print("Error with safe in avoidObstacle")
	else:
		if safe == 0:
			return getSafe(side)
	
	if side == 'e'
		if robotHeading != 
	

	


