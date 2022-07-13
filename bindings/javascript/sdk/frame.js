class FrameImpl {

    m_details; // FrameDetails
    m_depthData; //uint16_t *
    m_irData; //uint16_t *
    m_rgbData; //uint16_t *
    m_fullData; //uint16_t *
    constructor() {
        this.m_details = new FrameDetails(0, 0, 0, 0, 0, 0, "");
        this.m_depthData = null;
        this.m_irData = null;
        this.m_rgbData = null;
        this.m_fullData = null;
    }

    // aditof::Status setDetails(const aditof::FrameDetails &details);
    setDetails(details) {
            let status = Status.OK;

            if (details == this.m_details) {
                console.log('INFO: Same details provided. Doing nothing.');
                return status;
            }

            if (this.m_fullData) {
                this.m_fullData = null;
            }

            this.m_details = details;
            allocFrameData(this.m_details);


            return status;
        }
        // aditof::Status getDetails(aditof::FrameDetails &details) const;
    getDetails() {
        return [Status.OK, this.m_details];
    }

    // aditof::Status getData(aditof::FrameDataType dataType, uint16_t **dataPtr);
    getData(dataType) {
        let status = Status.OK;
        let data;

        switch (dataType) {
            case FrameDataType.FULL_DATA:
                {
                    data = this.m_fullData;
                    break;
                }
            case FrameDataType.IR:
                {
                    data = this.m_irData;
                    break;
                }
            case FrameDataType.DEPTH:
                {
                    data = this.m_depthData;
                    break;
                }
            case FrameDataType.RGB:
                {
                    data = this.m_rgbData;
                    break;
                }
        }
        return [status, data];
    }

    // void allocFrameData(const aditof::FrameDetails &details);
    allocFrameData(details) {
        this.m_fullData = new Array(details.fullDataWidth * details.fullDataHeight + details.rgbWidth * details.rgbHeight);
        this.m_depthData = this.m_fullData;
        if (details.fullDataHeight == details.height)
            this.m_irData = this.m_depthData;
        else
            this.m_irData = this.m_fullData + (details.width * details.height);

        this.m_rgbData = this.m_fullData + (details.fullDataWidth * details.fullDataHeight);
    }

}


class Frame extends FrameImpl {
    constructor() {
        super();
    }
}

window.FrameImpl = FrameImpl;
window.Frame = Frame;