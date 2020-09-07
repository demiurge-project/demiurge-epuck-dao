#ifndef REFERENCE_MODEL_3_1_H
#define REFERENCE_MODEL_3_1_H

#include "EpuckDAO.h"
#include "RabMessageBuffer.h"

using namespace argos;

class ReferenceModel3Dot1: public EpuckDAO {
  public:
    /*
     *  Class constructor.
     */
    ReferenceModel3Dot1();

    /*
     * Class destructor.
     */
    virtual ~ReferenceModel3Dot1();

    /*
     * Reset function.
     */
    virtual void Reset();

    /*
     * Getter for the proximity reading.
     */
    CCI_EPuckProximitySensor::SReading GetProximityReading();

    /*
     * Setter for the proximity input.
     */
    void SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input);

    /*
     * Getter for the light reading.
     */
    CCI_EPuckLightSensor::SReading GetLightReading();

    /*
     * Setter for the light input.
     */
    void SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input);

    /*
     * Getter for the ground input.
     */
    Real GetGroundReading();

    /*
     * Setter for the ground input.
     */
    void SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input);

    /*
     * Getter for the number of surrounding robots.
     */
    const UInt8 GetNumberNeighbors() const;

    /*
     * Setter for the number of surrounding robots.
     */
    virtual void SetNumberNeighbors(const UInt8& un_number_neighbors);

    /*
     * Getter the number of messages of neighbors.
     */
    UInt8 GetNumberOfRobotsInState(UInt8 un_message);

    /*
     * Getter for the range-and-bearing input.
     */
    std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> GetRangeAndBearingMessages() ;

    /*
     * Getter for the center of mass of neighbors computed with RaB messages
     */
    CCI_EPuckRangeAndBearingSensor::SReceivedPacket GetNeighborsCenterOfMass();

    /*
     * Getter for the center of mass of neighbors computed with the camera
     */
    CCI_EPuckOmnidirectionalCameraSensor::SBlob GetNeighborsDirection();

    /*
     * Getter for the center of mass of neighbors computed with the camera
     */
    CCI_EPuckOmnidirectionalCameraSensor::SBlob GetNeighborsCoesion(Real fGain, Real fTargetDistance, Real fExp);

    /*
     * Setter for the range-and-bearing input.
     */
    void SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets);

    /*
     * Setter for the camera input.
     */
    void SetCameraInput(CCI_EPuckOmnidirectionalCameraSensor::SReadings s_camera_input);

    /*
     * Getter for the camera input.
     */
    CCI_EPuckOmnidirectionalCameraSensor::SReadings GetCameraInput() const;

    /*
     * Setter for the RGB LEDs color.
     */
    void SetLEDsColor(const CColor& c_color);

    /*
     * Getter for the RGB LEDs color.
     */
    const CColor& GetLEDsColor() const;


  private:

    /* Lenard-Jones potencial funtcion */
    Real LJMagnitude(Real fDistance, Real fGain, Real fTargetDistance, Real fExp);

    /*
     * The camera sensors input.
     */
    CCI_EPuckOmnidirectionalCameraSensor::SReadings m_sCameraInput;

    /*
     * The proximity sensors input.
     */
    CCI_EPuckProximitySensor::TReadings m_sProximityInput;

    /*
     * The light sensors input.
     */
    CCI_EPuckLightSensor::TReadings m_sLightInput;

    /*
     * The ground sensors input.
     */
    std::deque<CCI_EPuckGroundSensor::SReadings> m_deqGroundInput;

    /*
     * The ground sensors input.
     */
    CCI_EPuckGroundSensor::SReadings m_sGroundInput;

    /*
     * The number of surrounding robots.
     */
    UInt8 m_unNumberNeighbors;

    /*
     * Pointer to the range-and-bearing messages buffer.
     */
    RabMessageBuffer m_pcRabMessageBuffer;

    /*
     * The color of RGB LEDs  (output variable).
     */
    CColor m_cLEDsColor;

};

#endif
