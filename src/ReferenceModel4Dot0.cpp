#include "ReferenceModel4Dot0.h"

/****************************************/
/****************************************/

ReferenceModel4Dot0::ReferenceModel4Dot0() {
  m_pcRng = CRandom::CreateRNG("argos");
  m_pcRabMessageBuffer = RabMessageBuffer();
  LOG << "New m_pcRabMessageBuffer" << std::endl;
  m_pcRabMessageBuffer.SetTimeLife(10);
  m_fMaxVelocity = 12;
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

ReferenceModel4Dot0::~ReferenceModel4Dot0() {}

/****************************************/
/****************************************/

void ReferenceModel4Dot0::Reset() {
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_pcRabMessageBuffer.Reset();
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel4Dot0::GetNumberNeighbors() const {
  return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModel4Dot0::SetNumberNeighbors(const UInt8& un_number_neighbors){
  m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

Real TransformRangeToProximity(Real fRange) {
  if (fRange <= 0.01) {
    return 1.0;
  } else if (fRange >= 0.7) {
    return 0.0;
  } else {
    return 1.0 - ((fRange - 0.01) / (0.7 - 0.01));
  }
}

/****************************************/
/****************************************/

CCI_EPuckProximitySensor::SReading ReferenceModel4Dot0::GetRABReading(){
	CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
	CVector2 cSumRAB(0, CRadians::ZERO);

	for (auto it = sRabPackets.begin(); it != sRabPackets.end(); ++it) {
		if ((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier() && (*it)->Data[1] != 0) {
			// For this vector which is used to do the random walk, we don't want the information from the patch epucks
			Real fProximityValue = TransformRangeToProximity((*it)->Range);
			cSumRAB += CVector2(fProximityValue, (*it)->Bearing.SignedNormalize());
		}
	}

	Real fProximityValue = (cSumRAB.Length() > 1) ? 1 : cSumRAB.Length();
	
	CCI_EPuckProximitySensor::SReading cOutputReading;
	cOutputReading.Value = fProximityValue;
	cOutputReading.Angle = cSumRAB.Angle().SignedNormalize();

	return cOutputReading;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket ReferenceModel4Dot0::GetAttractionVectorToNeighbors(Real f_alpha_parameter) {
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

CCI_EPuckRangeAndBearingSensor::SReceivedPacket ReferenceModel4Dot0::GetAttractionVectorToPatch(Real f_alpha_parameter, UInt8 f_delta_parameter) {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  CVector2 sRabVectorSum(0,CRadians::ZERO);

  for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
    if (((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier()) && ((*it)->Data[1] == f_delta_parameter)) {
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

Real ReferenceModel4Dot0::GetMinimumRangeFromPatch(UInt32 color) {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;

  Real minimum = 10.0;
  LOG << "GetMinimum" << std::endl;
  for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
    LOG << "Color sent " << (*it)->Data[1] << std::endl;
    if (((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier()) && ((*it)->Data[1] == color)) {
      LOG << "Message from someone else and black" << std::endl;
      if ((*it)->Range) {
        minimum = (*it)->Range;
      }
    }
  }
  LOG << "Minimum is " << minimum << std::endl;
  return minimum;
}

/****************************************/
/****************************************/

std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> ReferenceModel4Dot0::GetRangeAndBearingMessages() {
  return m_pcRabMessageBuffer.GetMessages();
}

/****************************************/
/****************************************/

void ReferenceModel4Dot0::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
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
