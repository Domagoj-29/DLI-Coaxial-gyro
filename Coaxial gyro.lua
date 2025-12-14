local function createCapacitor()
	local oldCharge=false
	local chargeCounter=0
	local dischargeCounter=nil
	return function(charge,chargeTicks,dischargeTicks)
		if dischargeCounter==nil then
			dischargeCounter=dischargeTicks
		end

		if charge then
			chargeCounter=math.min(chargeCounter+1,chargeTicks)
		else
			chargeCounter=0
		end

		if oldCharge and not charge then
			dischargeCounter=0
		end

		if not charge and dischargeCounter<dischargeTicks then
			dischargeCounter=dischargeCounter+1
		end

		oldCharge=charge

		local storedCharge=(dischargeCounter>0 and dischargeCounter<dischargeTicks) or (chargeCounter==chargeTicks and charge)
		return storedCharge
	end
end
local function createPulse()
	local counter=0
	return function(toggleSignal)
		if not toggleSignal then
			counter=0
		else
			counter=counter+1
		end
		return counter==1
	end
end
local function createSRlatch()
	local output=false
	return function(set,reset)
		if set and reset then
			output=false
		elseif set then
			output=true
		elseif reset then
			output=false
		end
		return output
	end
end
local function createDelta()
	local oldValue=0
	return function(inputValue)
		local deltaValue=inputValue-oldValue
		oldValue=inputValue
		return deltaValue
	end
end
local function createPID()
	local oldError=0
	local integral=0
	return function(setpoint,processVariable,P,I,D,active)
		if not active then
			oldError=0
			integral=0
			return 0
		end

		local error=setpoint-processVariable
		local derivative=error-oldError
		oldError=error
		integral=integral+error*I

		return error*P+integral+derivative*D
	end
end
local function originalPID()
	local integral=0
	local oldVariable=0
	local output=0
	return function(setpoint,processVariable,P,I,D)
		local error=setpoint-processVariable
		local difference=processVariable-oldVariable

		if output>0 and output<1 then
			integral=integral+I*error
			integral=math.max(math.min(integral,1),0)
		end

		oldVariable=processVariable
		output=math.max(math.min((P*error)+integral+(-D*difference),1),0)
		return output
	end
end
local function thresholdGate(inputNumber,lowThreshold,highThreshold)
	return (inputNumber>=lowThreshold and inputNumber<=highThreshold)
end
local function clamp(inputNumber,minValue,maxValue)
	return math.max(minValue,math.min(inputNumber,maxValue))
end
local function len(x,y)
	return math.sqrt(x*x+y*y)
end

autopilotPitchPID=createPID()
altDifferencePID=createPID()
altHoldPID=createPID()

posHoldPitchPulse1=createPulse()
posHoldPitchPulse2=createPulse()
posHoldRollPulse1=createPulse()
posHoldRollPulse2=createPulse()

pitchSRlatch=createSRlatch()
rollSRlatch=createSRlatch()

capacitor=createCapacitor()

posHoldPitchPID=createPID()
posHoldRollPID=createPID()


pitchPID=createPID()
rollPID=createPID()
yawPID=createPID()
collectivePID=originalPID()

deltaAltitude=createDelta()


local liftMaxSpeed=0.12+property.getNumber("Lift max speed")*0.02
local pitchMaxTilt=0.1+property.getNumber("Pitch max tilt")*0.015
local pitchTrim=property.getNumber("Pitch trim")*0.02
local rollMaxTilt=0.08+property.getNumber("Roll max tilt")*0.015

local yawMaxSpeed=0.14+property.getNumber("Yaw max speed")*0.12
local yawTurnByRoll=property.getNumber("Yaw turn by roll")
local posHoldMaxTilt=property.getNumber("Position hold max tilt")
local autopilotDeceleration=property.getNumber("Autopilot deceleration")

local liftPIDSensitivity=property.getNumber("Lift PID sensitivity")
local pitchPIDSensitivity=property.getNumber("Pitch PID sensitivity")
local rollPIDSensitivity=property.getNumber("Roll PID sensitivity")
local yawPIDSensitivity=property.getNumber("Yaw PID sensitivity")

local autopilotPitch=0
local stabilizedAltHold=0
function onTick()
	local gpsX=input.getNumber(1)
	local altitude=input.getNumber(2)
	local gpsY=input.getNumber(3)

	local directionalSpeedRight=input.getNumber(7)
	local directionalSpeedFront=input.getNumber(9)
	local angularSpeedUp=input.getNumber(11)

	local tiltFront=-input.getNumber(15)
	local tiltRight=-input.getNumber(16)
	local compass=input.getNumber(17)


	local rollAD=input.getNumber(18)
	local pitchWS=input.getNumber(19)
	local yawLR=input.getNumber(20)
	local collectiveUD=input.getNumber(21)

	local altHoldAltitude=math.max(input.getNumber(22),0)
	local waypointX=input.getNumber(23)
	local waypointY=input.getNumber(24)

	local altHoldEnabled=input.getBool(1)
	local autopilotEnabled=input.getBool(2)

	-- Autopilot
	if waypointX==0 and waypointY==0 then
		autopilotEnabled=false
	end

	local autopilotYaw=3*((compass+math.atan((gpsX-waypointX),(gpsY-waypointY))/(math.pi*2)+1)%1-0.5)
	local distance=len((gpsX-waypointX),(gpsY-waypointY))
	local autopilotPitchPIDSetpoint=math.min((distance-100)/(500+autopilotDeceleration*100),1)

	if not (thresholdGate(autopilotYaw,-0.2,0.2) and distance>100) then
		autopilotPitchPIDSetpoint=0
		if distance<10 then
			autopilotYaw=0
		end
	end

	autopilotPitch=autopilotPitchPID(autopilotPitchPIDSetpoint,autopilotPitch,0,0.01,0,distance>100)

	-- Altitude hold
	local altitudeDifference=altDifferencePID(altHoldAltitude,altitude,0.005,0,0,altHoldEnabled)

	stabilizedAltHold=altHoldPID(altitudeDifference,stabilizedAltHold,0,0.02,0,altHoldEnabled)
	local outputAltHold=clamp(stabilizedAltHold,-liftMaxSpeed,liftMaxSpeed)

	-- Position hold
	local controlsNearZero=thresholdGate(yawLR,-0.1,0.1) and thresholdGate(pitchWS,-0.1,0.1) and thresholdGate(rollAD,-0.1,0.1) and collectiveUD<0.1
	local chargeCapacitor=(controlsNearZero and not autopilotEnabled) or (not (distance<100) and autopilotEnabled)
	local capacitor=capacitor(chargeCapacitor,120,1)

	local posHoldPitchPIDActive=pitchSRlatch(not thresholdGate(directionalSpeedFront,-3,3),posHoldPitchPulse1(thresholdGate(directionalSpeedFront,-1,1)))
	posHoldPitchPIDActive=not posHoldPitchPulse2(not posHoldPitchPIDActive) and capacitor  -- The secondary pulses activate when going from ON to OFF

	local posHoldRollPIDActive=rollSRlatch(not thresholdGate(directionalSpeedRight,-3,3),posHoldRollPulse1(thresholdGate(directionalSpeedRight,-1,1)))
	posHoldRollPIDActive=not posHoldRollPulse2(not posHoldRollPIDActive) and capacitor

	local posHoldPitch=posHoldPitchPID(0,directionalSpeedFront,0.005,0.0001,0,posHoldPitchPIDActive)
	local posHoldRoll=posHoldRollPID(0,directionalSpeedRight,0.005,0.0001,0,posHoldRollPIDActive)

	posHoldPitch=clamp(posHoldPitch,-posHoldMaxTilt*0.01,posHoldMaxTilt*0.01)
	posHoldRoll=clamp(posHoldRoll,-posHoldMaxTilt*0.01,posHoldMaxTilt*0.01)

	-- Pitch control
	local pitchControl=(autopilotEnabled) and autopilotPitch or pitchWS

	local pitchPIDSetpoint=pitchControl*pitchMaxTilt*math.sqrt(1-math.abs(rollAD)/2)
	local stabilizedPitch=pitchPID(pitchPIDSetpoint,tiltFront,pitchPIDSensitivity+3,0,80,true)

	local outputPitch=posHoldPitch+stabilizedPitch+(pitchTrim*0.02)
	-- Roll control
	local rollControl=(autopilotEnabled) and 0 or rollAD

	local rollPIDSetpoint=rollControl*rollMaxTilt*math.sqrt(1-math.abs(pitchWS)/2)
	local stabilizedRoll=rollPID(rollPIDSetpoint,tiltRight,rollPIDSensitivity*0.5+2,0,40,true)

	local outputClockwiseRoll=posHoldRoll+stabilizedRoll
	local outputCounterClockwiseRoll=-(posHoldRoll+stabilizedRoll)
	-- Yaw control
	local yawControl=(autopilotEnabled) and autopilotYaw or yawLR

	local yawPIDSetpoint=yawMaxSpeed*clamp((yawControl+pitchWS*rollAD*yawTurnByRoll),-1,1)
	local stabilizedYaw=yawPID(yawPIDSetpoint,angularSpeedUp,yawPIDSensitivity+5,0,0,true)
	-- Collective control
	local collectivePIDSetpoint=(altHoldEnabled) and outputAltHold or collectiveUD*liftMaxSpeed

	local stabilizedCollective=collectivePID(collectivePIDSetpoint,deltaAltitude(altitude),6+liftPIDSensitivity,0.2+liftPIDSensitivity*0.04,10+2*liftPIDSensitivity)

	local outputClockwiseCollective=stabilizedCollective-stabilizedYaw
	local outputCounterClockwiseCollective=stabilizedCollective+stabilizedYaw

	output.setNumber(1,outputPitch)
	output.setNumber(2,outputClockwiseRoll)
	output.setNumber(3,outputCounterClockwiseRoll)
	output.setNumber(4,outputClockwiseCollective)
	output.setNumber(5,outputCounterClockwiseCollective)
end