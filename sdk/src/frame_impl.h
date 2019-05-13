#ifndef FRAME_IMPL
#define FRAME_IMPL

#include "frame_definitions.h"
#include "status_definitions.h"

#include <stdint.h>

class FrameImpl {
  public:
    FrameImpl();
    ~FrameImpl();

  public: // from TofFrame
    aditof::Status setDetails(const aditof::FrameDetails &details);
    aditof::Status getDetails(aditof::FrameDetails &details) const;
    aditof::Status getData(aditof::FrameDataType dataType, uint16_t **dataPtr);

  private:
    aditof::FrameDetails m_details;
    uint16_t *m_depthData;
    uint16_t *m_irData;
    uint16_t *m_rawData;
};

#endif // FRAME_IMPL
