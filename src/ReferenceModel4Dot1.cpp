#include "ReferenceModel4Dot1.h"

/****************************************/
/****************************************/

ReferenceModel4Dot1::ReferenceModel4Dot1() {
  m_pcRng = CRandom::CreateRNG("argos");
  m_pcRabMessageBuffer = RabMessageBuffer();
  m_pcRabMessageBuffer.SetTimeLife(10);
  m_fMaxVelocity = 12;
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_bGroundLEDsState = 0;
}

/****************************************/
/****************************************/

ReferenceModel4Dot1::~ReferenceModel4Dot1() {}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::Reset() {
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_pcRabMessageBuffer.Reset();
  m_bGroundLEDsState = 0;
  m_deqGroundInput.clear();
}

/****************************************/
/****************************************/

CCI_EPuckProximitySensor::SReading ReferenceModel4Dot1::GetProximityReading() {
  CCI_EPuckProximitySensor::SReading cOutputReading;
  CVector2 cSumProxi(0, CRadians::ZERO);
  for (UInt8 i = 0; i < m_sProximityInput.size(); i++) {
    cSumProxi += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle.SignedNormalize());
  }

  cOutputReading.Value = (cSumProxi.Length() > 1) ? 1 : cSumProxi.Length();
  cOutputReading.Angle = cSumProxi.Angle().SignedNormalize();

  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {
  m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_EPuckLightSensor::SReading ReferenceModel4Dot1::GetLightReading() {
  CCI_EPuckLightSensor::SReading cOutputReading;
  CVector2 cSumLight(0, CRadians::ZERO);
	for (UInt8 i = 0; i < m_sLightInput.size(); i++) {
    if (m_sLightInput[i].Value > 0.2) {
      cOutputReading.Value = 1;
    }
		cSumLight += CVector2(m_sLightInput[i].Value, m_sLightInput[i].Angle.SignedNormalize());
	}
  if (cOutputReading.Value == 1) {
    cOutputReading.Angle = cSumLight.Angle().SignedNormalize();
  }
  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {
  m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

Real ReferenceModel4Dot1::GetGroundReading() {
  std::deque<CCI_EPuckGroundSensor::SReadings>::iterator it;
  UInt32 unBlackWhiteCounter[2] = {0,0};  //unBlackWhiteCounter[0] -> Black; unBlackWhiteCounter[1] -> White.
  //float fBlackThreshold = 0.03;
  float fBlackThreshold = 0.1;
  float fWhiteThreshold = 0.95;

  if (m_deqGroundInput.size() < 5){
      CCI_EPuckGroundSensor::SReadings s_ground_input;
      s_ground_input.Center = 1.0;
      s_ground_input.Left = 1.0;
      s_ground_input.Right = 1.0;

      m_deqGroundInput.pop_front();
      m_deqGroundInput.push_back(s_ground_input);
      m_deqGroundInput.pop_front();
      m_deqGroundInput.push_back(s_ground_input);

      m_deqGroundInput.push_back(s_ground_input);
      m_deqGroundInput.push_back(s_ground_input);
      m_deqGroundInput.push_back(s_ground_input);
  }


  for (it = m_deqGroundInput.begin(); it != m_deqGroundInput.end(); it++) {
    if (it->Left < fBlackThreshold) {
      unBlackWhiteCounter[0] += 1;
    }
    else if (it->Left > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }
    if (it->Center < fBlackThreshold) {
      unBlackWhiteCounter[0] +=1;
    }
    else if (it->Center > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }
    if (it->Right < fBlackThreshold) {
      unBlackWhiteCounter[0] +=1;
    }
    else if (it->Right > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }

  }

  if (unBlackWhiteCounter[0] > 10) {
    return 0.0f;
  }
  else if (unBlackWhiteCounter[1] > 10) {
    return 1.0f;
  }
  else {
    return 0.5f;
  }
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {

  m_deqGroundInput.push_back(s_ground_input);

  if (m_deqGroundInput.size() > 5) {
    m_deqGroundInput.pop_front();
  }
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel4Dot1::GetNumberNeighbors() const {
  return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetNumberNeighbors(const UInt8& un_number_neighbors){
  m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket ReferenceModel4Dot1::GetAttractionVectorToNeighbors(Real f_alpha_parameter) {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  CVector2 sRabVectorSum(0,CRadians::ZERO);

  for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
    if (((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier())) {
      sRabVectorSum += CVector2(f_alpha_parameter/(1 + (*it)->Range),(*it)->Bearing.SignedNormalize());
    }
  }

  CCI_EPuckRangeAndBearingSensor::SReceivedPacket cRaBReading;
  cRaBReading.Range = sRabVectorSum.Length();
  cRaBReading.Bearing = sRabVectorSum.Angle().SignedNormalize();

  return cRaBReading;
}


/****************************************/
/****************************************/

std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> ReferenceModel4Dot1::GetRangeAndBearingMessages() {
  return m_pcRabMessageBuffer.GetMessages();
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> mapRemainingMessages;
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*>::iterator mapIt;
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  m_unNumberNeighbors = 0;
  for (it = s_packets.begin(); it < s_packets.end(); ++it) {
    if ((*it)->Data[0] != m_unRobotIdentifier) {
      if (mapRemainingMessages.find((*it)->Data[0]) != mapRemainingMessages.end()) {  // If ID not in map, add message.
        mapRemainingMessages[(*it)->Data[0]] = (*it);
      } else if ((*it)->Bearing != CRadians::ZERO){  // If ID there, overwrite only if the message is valid (correct range and bearing information)
        mapRemainingMessages[(*it)->Data[0]] = (*it);
      }
    }
  }
  for (mapIt = mapRemainingMessages.begin(); mapIt != mapRemainingMessages.end(); ++mapIt) {
    m_pcRabMessageBuffer.AddMessage((*mapIt).second);
    m_unNumberNeighbors += 1;
  }
  m_pcRabMessageBuffer.Update();
}

/****************************************/
/****************************************/

CCI_EPuckOmnidirectionalCameraSensor::SReadings ReferenceModel4Dot1::GetCameraInput() const {
    return m_sCameraInput;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetCameraInput(CCI_EPuckOmnidirectionalCameraSensor::SReadings s_cam_input) {
    m_sCameraInput = s_cam_input;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetLEDsColor(const CColor& c_color) {
    m_cLEDsColor = c_color;
}

/****************************************/
/****************************************/

const CColor& ReferenceModel4Dot1::GetLEDsColor() const {
    return m_cLEDsColor;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot1::SetGroundLEDsState(size_t b_state) {
    m_bGroundLEDsState = b_state;
}

/****************************************/
/****************************************/

size_t ReferenceModel4Dot1::GetGroundLEDsState() {
    return m_bGroundLEDsState;
}
