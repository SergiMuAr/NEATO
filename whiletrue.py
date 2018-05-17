while true:
var = get_5sensor(get_laser())

		centralLeftSensorValue = ((distDetection-var[3])/calibration if var[3] < distDetection else 0)
		outerLeftSensorValue = ((distDetection-var[4])/calibration  if var[4] < distDetection else 0)
		centralRightSensorValue = ((distDetection-var[1])/calibration if var[1] < distDetection else 0)
		outerRightSensorValue = ((distDetection-var[0])/calibration if var[0] < distDetection else 0)
		centralSensorValue = ((distDetection-var[2])/calibration if var[2] < distDetection else 0)

		left = (centralLeftSensorValue + outerLeftSensorValue)/2
		right = (centralRightSensorValue + outerRightSensorValue)/2

		minLeft = min(centralLeftSensorValue,outerLeftSensorValue)
		maxLeft = max(centralLeftSensorValue,outerLeftSensorValue)
		minRight = min(centralRightSensorValue,outerRightSensorValue)
		maxRight = max(centralRightSensorValue,outerRightSensorValue)

		if maxLeft < maxRight or (maxLeft == maxRight and maxLeft > maxRight):
			print 'GIRAR IZQUIERDA -------------------------------------'	
			right = right + centralSensorValue
		else:
			print 'GIRAR DERECHA ---------------------------------------'	
			left = left + centralSensorValue

		if abs(left-right) < 5:
			right = right + centralSensorValue

		if left == 0 and right == 0:
			leftMotor = 150
			rightMotor = 150
			velocity = initialVelocity
		elif (abs(left)+abs(right)) < 50:
			leftMotor = max(1000 - right, 0)
			rightMotor = max(1000 - left, 0)
			velocity = 150
		else:
			leftMotor = max(1000 - right, 0)
			rightMotor = max(1000 - left, 0)
			velocity = 100

		print 'Left/Right: ',left,' / ',right
		print 'LeftMotor/RightMotor: ',leftMotor,' / ',rightMotor

		envia(ser, 'SetMotor LWheelDist '+ str(rightMotor) +' RwheelDist ' + str(leftMotor) + ' Speed ' + str(initialVelocity))