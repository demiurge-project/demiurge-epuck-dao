#ifndef REFERENCE_MODEL_1_1_H
#define REFERENCE_MODEL_1_1_H

#include "EpuckDAO.h"
#include "RabMessageBuffer.h"

using namespace argos;

class ReferenceModel4Dot0: public EpuckDAO {
  public:
    /*
     *  Class constructor.
     */
    ReferenceModel4Dot0();

    /*
     * Class destructor.
     */
    virtual ~ReferenceModel4Dot0();

    /*
     * Reset function.
     */
    virtual void Reset();

    /*
     * Getter for the ground input.
     */
    CCI_EPuckGroundSensor::SReadings GetGroundInput();

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
     * Getter for the range-and-bearing input.
     */
    std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> GetRangeAndBearingMessages() ;

    /*
     * Getter for the vector representing the attraction force to the neighbors computed with RaB messages
     */
    CCI_EPuckRangeAndBearingSensor::SReceivedPacket GetAttractionVectorToNeighbors(Real f_alpha_parameter);

    /*
     * Getter for the center of mass of neighbors computed with RaB messages
     */
    CCI_EPuckRangeAndBearingSensor::SReceivedPacket GetNeighborsCenterOfMass();

    /*
     * Setter for the range-and-bearing input.
     */
    void SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets);


  private:
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

};

#endif
