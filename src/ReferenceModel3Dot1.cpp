#include "ReferenceModel3Dot1.h"

/****************************************/
/****************************************/

ReferenceModel3Dot1::ReferenceModel3Dot1() {
  m_pcRng = CRandom::CreateRNG("argos");
  m_pcRabMessageBuffer = RabMessageBuffer();
  m_pcRabMessageBuffer.SetTimeLife(10);
  m_fMaxVelocity = 12;
  m_fMaxOmega = 0.3f*m_fMaxVelocity;  // 4.5 [r/s] max
  m_fLengthEpuckAxis = 5.3f;          // [cm]
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_cLEDsColor = CColor::BLACK;
}

/****************************************/
/****************************************/

ReferenceModel3Dot1::~ReferenceModel3Dot1() {}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::Reset() {
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_cLEDsColor = CColor::BLACK;
  m_pcRabMessageBuffer.Reset();
}

/****************************************/
/****************************************/

CCI_EPuckProximitySensor::SReading ReferenceModel3Dot1::GetProximityReading() {
  CCI_EPuckProximitySensor::SReading cOutputReading;
  CVector2 cSumProxi(0, CRadians::ZERO);
  for (UInt8 i = 0; i < m_sProximityInput.size(); i++) {
      if (m_sProximityInput[i].Value > 0.25f) {
          // threshold 25%: Obstacle avoidance bhv, sum it!
          cSumProxi += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle.SignedNormalize());

      }
  }

  cOutputReading.Value = cSumProxi.Length();
  cOutputReading.Angle = cSumProxi.Angle().SignedNormalize();

  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {
  m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_EPuckLightSensor::SReading ReferenceModel3Dot1::GetLightReading() {
  CCI_EPuckLightSensor::SReading cOutputReading;
  CVector2 cSumLight(0, CRadians::ZERO);
  for (UInt8 i = 0; i < m_sLightInput.size(); i++) {
        if (m_sLightInput[i].Value > 0.25f) {
            // threshold 25%
          cSumLight += CVector2(m_sLightInput[i].Value, m_sLightInput[i].Angle.SignedNormalize());
        }


  }

  cOutputReading.Value = cSumLight.Length();
  cOutputReading.Angle = cSumLight.Angle().SignedNormalize();

  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {
  m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

Real ReferenceModel3Dot1::GetGroundReading() {
  std::deque<CCI_EPuckGroundSensor::SReadings>::iterator it;
  UInt32 unBlackWhiteCounter[2] = {0,0};  //unBlackWhiteCounter[0] -> Black; unBlackWhiteCounter[1] -> White.
  float fBlackThreshold = 0.03;
  float fWhiteThreshold = 0.85;
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

void ReferenceModel3Dot1::SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {
  m_deqGroundInput.push_back(s_ground_input);
  if (m_deqGroundInput.size() > 5) {
    m_deqGroundInput.pop_front();
  }

}

/****************************************/
/****************************************/

const UInt8 ReferenceModel3Dot1::GetNumberNeighbors() const {
  return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::SetNumberNeighbors(const UInt8& un_number_neighbors){
  m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket ReferenceModel3Dot1::GetNeighborsCenterOfMass() {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  CVector2 sRabVectorSum(0,CRadians::ZERO);

  if(!sRabPackets.empty()) {
      for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
        if ((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier()) {
          sRabVectorSum += CVector2(1.0f/((*it)->Range + 1),(*it)->Bearing.SignedNormalize());
        }
      }
      //sRabVectorSum /= sRabPackets.size();
  }

  CCI_EPuckRangeAndBearingSensor::SReceivedPacket cRaBReading;
  cRaBReading.Range = sRabVectorSum.Length();
  cRaBReading.Bearing = sRabVectorSum.Angle().SignedNormalize();

  return cRaBReading;
}

/****************************************/
/****************************************/

CCI_EPuckOmnidirectionalCameraSensor::SBlob ReferenceModel3Dot1::GetNeighborsDirection() {
    CCI_EPuckOmnidirectionalCameraSensor::SReadings sReadings = m_sCameraInput;
    CVector2 sCamVectorSum(0,CRadians::ZERO);

    if(! sReadings.BlobList.empty()) {
        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
            if (sReadings.BlobList[i]->Color == CColor::CYAN) {
                sCamVectorSum += CVector2(1.0f / (sReadings.BlobList[i]->Distance + 1),sReadings.BlobList[i]->Angle);
            }
        }
    }

    CCI_EPuckOmnidirectionalCameraSensor::SBlob cCamReading;
    cCamReading.Distance = sCamVectorSum.Length();
    cCamReading.Angle = sCamVectorSum.Angle().SignedNormalize();

    return cCamReading;
}

/****************************************/
/****************************************/

CCI_EPuckOmnidirectionalCameraSensor::SBlob ReferenceModel3Dot1::GetNeighborsCoesion(Real fGain, Real fTargetDistance, Real fExp) {
    CCI_EPuckOmnidirectionalCameraSensor::SReadings sReadings = m_sCameraInput;
    CVector2 sCamVectorSum(0,CRadians::ZERO);

    CCI_EPuckOmnidirectionalCameraSensor::SBlob cCamReading;
    UInt32 unNumMsg = 0;
    Real fLJ;

    if(! sReadings.BlobList.empty()) {
        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
            if (sReadings.BlobList[i]->Color == CColor::CYAN) {
                /* Lennard-Jones interaction force */


                    fLJ = LJMagnitude(sReadings.BlobList[i]->Distance,
                                                                fGain,
                                                                fTargetDistance,
                                                                fExp);



                /* Sum the contributions of the neighbors */
                sCamVectorSum += CVector2(fLJ,sReadings.BlobList[i]->Angle);

                ++unNumMsg;

            }
        }

        if(unNumMsg > 0) {
           /* Divide the accumulator by the number of messeges */
           sCamVectorSum /= unNumMsg;
           /* Limit the interaction force */
           if(sCamVectorSum.Length() > 1.0f) {
              sCamVectorSum.Normalize();
           }
        }
        /* All done */

        cCamReading.Distance = sCamVectorSum.Length();
        cCamReading.Angle = sCamVectorSum.Angle().SignedNormalize();
        return cCamReading;
    }
    else {
        /* Alone: go straight ahead */

        cCamReading.Distance = 1.0f;
        cCamReading.Angle = CRadians();
        return cCamReading;
    }
}

/****************************************/
/****************************************/

const CColor& ReferenceModel3Dot1::GetLEDsColor() const {
    return m_cLEDsColor;
}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::SetLEDsColor(const CColor& c_color) {
    m_cLEDsColor = c_color;
}

/****************************************/
/****************************************/


UInt8 ReferenceModel3Dot1::GetNumberOfRobotsInState(UInt8 un_message) {
    CCI_EPuckOmnidirectionalCameraSensor::SReadings sReadings = m_sCameraInput;
    UInt8 unNumberMessagingNeighbors = 0;

    CCI_EPuckOmnidirectionalCameraSensor::SBlob cCamReading;
    UInt32 unNumMsg = 0;


    if(! sReadings.BlobList.empty()) {
        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
            if (sReadings.BlobList[i]->Color == CColor::CYAN) {
                unNumberMessagingNeighbors+=1;
            }
        }
    }


    return unNumberMessagingNeighbors/3;
}

/****************************************/
/****************************************/

Real ReferenceModel3Dot1::LJMagnitude(Real fDistance, Real fGain, Real fTargetDistance, Real fExp) {

    //Real fExp = m_sCoesionParams.Exponent;
    Real fDa = fTargetDistance / fDistance;
    Real fDaN = ::pow(fDa,fExp);

    Real fMagnitude = -(fGain/fDistance)*(fDaN*fDaN - fDaN);

    return fMagnitude;

}

/****************************************/
/****************************************/

std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> ReferenceModel3Dot1::GetRangeAndBearingMessages() {
  return m_pcRabMessageBuffer.GetMessages();
}

/****************************************/
/****************************************/

void ReferenceModel3Dot1::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> mapRemainingMessages;
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*>::iterator mapIt;
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  m_unNumberNeighbors = 0;
  for (it = s_packets.begin(); it < s_packets.end(); ++it) {
    if ((*it)->Data[0] != m_unRobotIdentifier) {
      if (mapRemainingMessages.find((*it)->Data[0]) != mapRemainingMessages.end()) {  // If ID not in map, add message.
        mapRemainingMessages[(*it)->Data[0]] = (*it);
      //} else if (((*it)->Bearing != CRadians::ZERO) && ((*it)->Range >= 0.0f)){  // If ID there, overwrite only if the message is valid (correct range and bearing information)
      } else if (((*it)->Bearing != CRadians::ZERO)){  // If ID there, overwrite only if the message is valid (correct range and bearing information)
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

void ReferenceModel3Dot1::SetCameraInput(CCI_EPuckOmnidirectionalCameraSensor::SReadings s_camera_input) {

    CCI_EPuckOmnidirectionalCameraSensor::SReadings s_camera_filter;

    // Filter blobs to avoid each robot see themselves
    if(! s_camera_input.BlobList.empty()) {
        for(size_t i = 0; i < s_camera_input.BlobList.size(); ++i) {
            if ((s_camera_input.BlobList[i]->Distance >= 5.0f) && (s_camera_input.BlobList[i]->Distance <= 30.0f)) {
            //if (s_camera_input.BlobList[i]->Distance >= 5.0f) {
                s_camera_filter.BlobList.push_back(s_camera_input.BlobList[i]);
            }

        }
    }


    m_sCameraInput = s_camera_filter;
}


/****************************************/
/****************************************/

CCI_EPuckOmnidirectionalCameraSensor::SReadings ReferenceModel3Dot1::GetCameraInput() const {
    return m_sCameraInput;
}
