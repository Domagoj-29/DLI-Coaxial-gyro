-- Author: Domagoj29
-- GitHub: https://github.com/Domagoj-29
-- Workshop: https://steamcommunity.com/profiles/76561198935577915/myworkshopfiles/

-- onTick functions

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

local autopilotPitchPID=createPID()
local altDifferencePID=createPID()
local altHoldPID=createPID()

local posHoldPitchPulse1=createPulse()
local posHoldPitchPulse2=createPulse()
local posHoldRollPulse1=createPulse()
local posHoldRollPulse2=createPulse()

local pitchSRlatch=createSRlatch()
local rollSRlatch=createSRlatch()

local capacitor=createCapacitor()

local posHoldPitchPID=createPID()
local posHoldRollPID=createPID()


local pitchPID=createPID()
local rollPID=createPID()
local yawPID=createPID()
local collectivePID=originalPID()

local deltaAltitude=createDelta()

pgN=property.getNumber

LiftMaxSpeed=0.12+pgN("Lift max speed")*0.02
PitchMaxTilt=0.1+pgN("Pitch max tilt")*0.015
PitchTrim=pgN("Pitch trim")*0.02
RollMaxTilt=0.08+pgN("Roll max tilt")*0.015

YawMaxSpeed=0.14+pgN("Yaw max speed")*0.12
YawTurnByRoll=pgN("Yaw turn by roll")
PosHoldMaxTilt=pgN("Position hold max tilt")
AutopilotDeceleration=pgN("Autopilot deceleration")

LiftPIDSensitivity=pgN("Lift PID sensitivity")
PitchPIDSensitivity=pgN("Pitch PID sensitivity")
RollPIDSensitivity=pgN("Roll PID sensitivity")
YawPIDSensitivity=pgN("Yaw PID sensitivity")

AutopilotPitch=0
StabilizedAltHold=0
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
	local autopilotPitchPIDSetpoint=math.min((distance-100)/(500+AutopilotDeceleration*100),1)

	if not (thresholdGate(autopilotYaw,-0.2,0.2) and distance>100) then
		autopilotPitchPIDSetpoint=0
		if distance<10 then
			autopilotYaw=0
		end
	end

	AutopilotPitch=autopilotPitchPID(autopilotPitchPIDSetpoint,AutopilotPitch,0,0.01,0,distance>100)

	-- Altitude hold
	local altitudeDifference=altDifferencePID(altHoldAltitude,altitude,0.005,0,0,altHoldEnabled)

	StabilizedAltHold=altHoldPID(altitudeDifference,StabilizedAltHold,0,0.02,0,altHoldEnabled)
	local outputAltHold=clamp(StabilizedAltHold,-LiftMaxSpeed,LiftMaxSpeed)

	-- Position hold
	local controlsNearZero=thresholdGate(yawLR,-0.1,0.1) and thresholdGate(pitchWS,-0.1,0.1) and thresholdGate(rollAD,-0.1,0.1) and collectiveUD<0.1
	local chargeCapacitor=(controlsNearZero and not autopilotEnabled) or (distance<=100 and autopilotEnabled)
	local capacitor=capacitor(chargeCapacitor,120,1)

	local posHoldPitchPIDActive=pitchSRlatch(not thresholdGate(directionalSpeedFront,-3,3),posHoldPitchPulse1(thresholdGate(directionalSpeedFront,-1,1)))
	posHoldPitchPIDActive=not posHoldPitchPulse2(not posHoldPitchPIDActive) and capacitor  -- The secondary pulses activate when going from ON to OFF

	local posHoldRollPIDActive=rollSRlatch(not thresholdGate(directionalSpeedRight,-3,3),posHoldRollPulse1(thresholdGate(directionalSpeedRight,-1,1)))
	posHoldRollPIDActive=not posHoldRollPulse2(not posHoldRollPIDActive) and capacitor

	local posHoldPitch=posHoldPitchPID(0,directionalSpeedFront,0.005,0.0001,0,posHoldPitchPIDActive)
	local posHoldRoll=posHoldRollPID(0,directionalSpeedRight,0.005,0.0001,0,posHoldRollPIDActive)

	posHoldPitch=clamp(posHoldPitch,-PosHoldMaxTilt*0.01,PosHoldMaxTilt*0.01)
	posHoldRoll=clamp(posHoldRoll,-PosHoldMaxTilt*0.01,PosHoldMaxTilt*0.01)

	-- Pitch control
	local pitchControl=(autopilotEnabled) and AutopilotPitch or pitchWS

	local pitchPIDSetpoint=pitchControl*PitchMaxTilt*math.sqrt(1-math.abs(rollAD)/2)
	local stabilizedPitch=pitchPID(pitchPIDSetpoint,tiltFront,PitchPIDSensitivity+3,0,80,true)

	local outputPitch=posHoldPitch+stabilizedPitch+(PitchTrim*0.02)
	-- Roll control
	local rollControl=(autopilotEnabled) and 0 or rollAD

	local rollPIDSetpoint=rollControl*RollMaxTilt*math.sqrt(1-math.abs(pitchWS)/2)
	local stabilizedRoll=rollPID(rollPIDSetpoint,tiltRight,RollPIDSensitivity*0.5+2,0,40,true)

	local outputClockwiseRoll=posHoldRoll+stabilizedRoll
	local outputCounterClockwiseRoll=-(posHoldRoll+stabilizedRoll)
	-- Yaw control
	local yawControl=(autopilotEnabled) and autopilotYaw or yawLR

	local yawPIDSetpoint=YawMaxSpeed*clamp((yawControl+pitchWS*rollAD*YawTurnByRoll),-1,1)
	local stabilizedYaw=yawPID(yawPIDSetpoint,angularSpeedUp,YawPIDSensitivity+5,0,0,true)
	-- Collective control
	local collectivePIDSetpoint=(altHoldEnabled) and outputAltHold or collectiveUD*LiftMaxSpeed

	local stabilizedCollective=collectivePID(collectivePIDSetpoint,deltaAltitude(altitude),6+LiftPIDSensitivity,0.2+LiftPIDSensitivity*0.04,10+2*LiftPIDSensitivity)

	local outputClockwiseCollective=stabilizedCollective-stabilizedYaw
	local outputCounterClockwiseCollective=stabilizedCollective+stabilizedYaw

	output.setNumber(1,outputPitch)
	output.setNumber(2,outputClockwiseRoll)
	output.setNumber(3,outputCounterClockwiseRoll)
	output.setNumber(4,outputClockwiseCollective)
	output.setNumber(5,outputCounterClockwiseCollective)
end
